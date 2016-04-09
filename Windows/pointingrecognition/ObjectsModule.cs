namespace Microsoft.Samples.Kinect.PointingRecognition
{
    using System;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using OpenCvSharp;
    using OpenCvSharp.CPlusPlus;
    using OpenCvSharp.Extensions;

    public class ObjectsModule : ParentModule
    {
        private WebSocket4Net.WebSocket socket;
        private string dataFormat;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>

        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Sets up format of messages between the Windows application and ROS
        /// </summary>
        public void setDataFormat(string format)
        {
            this.dataFormat = format;
        }

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        private MultiSourceFrameReader multiFrameSourceReader = null;
        private WriteableBitmap bitmap = null;
        private uint bitmapBackBufferSize = 0;
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Intermediate storage for the color to depth mapping
        /// </summary>
        private CameraSpacePoint[] colorMappedToCameraPoints = null;


        /// <summary>
        /// Pixel coordinates of object in color bitmap frame
        /// </summary>
        private ColorCoordinates[] objColorCoords = null;
        private readonly Brush objectBrush = new SolidColorBrush(Color.FromArgb(255, 255, 255, 0));

        private uint numObjects;
        private CvScalar[,] objThresholdValArray = null;

        struct ColorCoordinates
        {
            public int col;
            public int row;
        };

        private int displayWidth;
        private int displayHeight;
        private System.Windows.Rect displayRect;

        private int frameNumber = 0;

        ///--- Parent Module Methods ---///

        /// <summary>
        /// Sets up the Kinect sensor
        /// </summary>
        public void setSensor(KinectSensor sensor)
        {
            this.kinectSensor = sensor;
        }

        /// <summary>
        /// Checks whether data should be sent before sending using socket
        /// </summary>
        public void SendData(string data)
        {
            if (Microsoft.Samples.Kinect.PointingRecognition.Properties.Settings.Default.SendData)
            {
                                                                                                                                             this.socket.Send(data);
            }
        }

        /// <summary>
        /// Sets up the WebSocket between the Windows application and ROS
        /// </summary>
        public void setSendingSocket(WebSocket4Net.WebSocket socket)
        {
            this.socket = socket;
        }

        /// <summary>
        /// Returns the imageSource of ObjectsModule
        /// </summary>
        public ImageSource getImageSource()
        {
            return this.imageSource;
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        public void MainWindow_Loaded()
        {
            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);

            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            this.displayWidth = colorFrameDescription.Width;
            this.displayHeight = colorFrameDescription.Height;
            this.colorMappedToCameraPoints = new CameraSpacePoint[this.displayWidth * this.displayHeight];
            this.displayRect = new System.Windows.Rect(0.0, 0.0, this.displayWidth, this.displayHeight);

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            this.bitmap = new WriteableBitmap(this.displayWidth, this.displayHeight, 96.0, 96.0, PixelFormats.Bgra32, null);
            this.bitmapBackBufferSize = (uint)((this.bitmap.BackBufferStride * (this.bitmap.PixelHeight - 1)) + (this.bitmap.PixelWidth * this.bytesPerPixel));

            // Object Color Segmentation
            // Get object threshold values from settings file
            // string format: b,g,r,b*,g*,r*|b,g,r,b*,g*,r* where * corresponds to upper threshold values
            // WARNING object threshold values are not sanitized or validated!
            this.numObjects = Properties.Settings.Default.NumObjects;
            this.objThresholdValArray = new CvScalar[numObjects, 2]; // 2 columns for lower and upper thresholds
            string objThresholdValues = Microsoft.Samples.Kinect.PointingRecognition.Properties.Settings.Default.ObjectRGBThresholds;
            string[] split = objThresholdValues.Split(new Char[] { '|' });
            this.objColorCoords = new ColorCoordinates[numObjects];
            for (int i = 0; i < numObjects; i++)
            {
                // store object threshold values in array
                string[] thresholds = split[i].Split(new Char[] { ',' });
                CvScalar lower = new CvScalar(Convert.ToInt32(thresholds[0]), Convert.ToInt32(thresholds[1]), Convert.ToInt32(thresholds[2]));
                CvScalar upper = new CvScalar(Convert.ToInt32(thresholds[3]), Convert.ToInt32(thresholds[4]), Convert.ToInt32(thresholds[5]));
                this.objThresholdValArray[i, 0] = lower;
                this.objThresholdValArray[i, 1] = upper;

                // init center coordinates of objects in color space
                this.objColorCoords[i].row = 0;
                this.objColorCoords[i].col = 0;
            }

            if (this.multiFrameSourceReader != null)
            {
                this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;
            }
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            ColorFrame colorFrame = null;
            DepthFrame depthFrame = null;
            bool isBitmapLocked = false;

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // if the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            try
            {
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();

                // if any frame has expired by the time we process this event, return.
                // the "finally" statement will Dispose any that are not null.
                if ((depthFrame == null) || (colorFrame == null))
                {
                    return;
                }

                // Process Depth

                // access the depth frame data directly via LockImageBuffer to avoid making a copy
                using (KinectBuffer depthFrameData = depthFrame.LockImageBuffer())
                {
                    this.coordinateMapper.MapColorFrameToCameraSpaceUsingIntPtr(
                        depthFrameData.UnderlyingBuffer,
                        depthFrameData.Size,
                        this.colorMappedToCameraPoints);
                }

                // we're done with the DepthFrame 
                depthFrame.Dispose();
                depthFrame = null;

                // Process Color

                // lock the bitmap for writing
                this.bitmap.Lock();
                isBitmapLocked = true;

                colorFrame.CopyConvertedFrameDataToIntPtr(this.bitmap.BackBuffer, this.bitmapBackBufferSize, ColorImageFormat.Bgra);

                // we're done with the ColorFrame 
                colorFrame.Dispose();
                colorFrame = null;

                this.bitmap.AddDirtyRect(new Int32Rect(0, 0, this.bitmap.PixelWidth, this.bitmap.PixelHeight));

                this.bitmap.Unlock();
                isBitmapLocked = false;

                // Segment Object

                if (this.frameNumber % 4 == 0)
                {
                    Mat mat = this.bitmap.ToMat();
                    Cv2.CvtColor(mat, mat, ColorConversion.RgbaToRgb);

                    for (int i = 0; i < this.numObjects; i++)
                    {
                        this.thresholdImage(ref mat, i, this.objThresholdValArray[i, 0], this.objThresholdValArray[i, 1]);
                    }

                    mat.Dispose();
                }
                this.frameNumber = this.frameNumber > 1000 ? 0 : this.frameNumber + 1;

                // Send data
                this.SendData(this.serializeObjects(this.objColorCoords));

                // Draw objects
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    dc.DrawRectangle(Brushes.Transparent, null, new System.Windows.Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    foreach (ColorCoordinates objCoords in this.objColorCoords)
                    {
                        if (objCoords.col != 0 && objCoords.row != 0)
                        {
                            System.Windows.Point point = new System.Windows.Point(objCoords.col, objCoords.row);
                            dc.DrawEllipse(this.objectBrush, null, point, 10, 10);
                        }
                    }
                }
            }
            finally
            {
                if (isBitmapLocked)
                {
                    this.bitmap.Unlock();
                }

                if (colorFrame != null)
                {
                    colorFrame.Dispose();
                }

                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                }
            }
        }

        private void thresholdImage(ref Mat mat, int obj, CvScalar lower, CvScalar upper)
        {
            // threshold image and get center of thresholded image to locate object center
            Mat thresholded = new Mat();
            Cv2.InRange(mat, lower, upper, thresholded);
            Moments moments = Cv2.Moments(thresholded, true);
            double moment10 = moments.M10;
            double moment01 = moments.M01;
            double area = moments.M00;
            if (area == 0.0)
            {
                return;
            }

            // update object coordinates
            this.objColorCoords[obj].col = (int)moment10 / (int)area;
            this.objColorCoords[obj].row = (int)moment01 / (int)area;

            thresholded.Dispose();
        }

        private string serializeObjects(ColorCoordinates[] objectColorCoordinates)
        {
            string data = "";
            foreach (ColorCoordinates objCoords in objectColorCoordinates)
            {
                // get x,y,z coordinates using color coordinates
                int objIndexInCameraArray = objCoords.row * (int)this.bitmap.Width + objCoords.col;
                CameraSpacePoint objPoint = this.colorMappedToCameraPoints[objIndexInCameraArray];

                // send object coordinates to Linux using websocket if coordinates are valid
                if (!float.IsNegativeInfinity(objPoint.X) &&
                    !float.IsNegativeInfinity(objPoint.Y) &&
                    !float.IsNegativeInfinity(objPoint.Z))
                {
                    data += objPoint.X.ToString() + "," + objPoint.Y.ToString() + "," + objPoint.Z.ToString() + ":";
                }
                else
                {
                    data += "0,0,0:";
                }
            }
            if (data.Length > 0)
            {
                data = data.Remove(data.Length - 1);
            }
            string format = this.dataFormat;
            format = format.Replace("out_topic", "objects_info");
            string serialized = format.Replace("XXXX", data);
            return serialized;
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        public void MainWindow_Closing()
        {
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            if (this.kinectSensor != null)
            {
                // on failure, set the status text
                this.statusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                                : Properties.Resources.SensorNotAvailableStatusText;
            }
        }
    }
}
