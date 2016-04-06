//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
namespace Microsoft.Samples.Kinect.RGBThresholder
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using OpenCvSharp;
    using OpenCvSharp.CPlusPlus;
    using OpenCvSharp.Extensions;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : System.Windows.Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// RGB thresholds for color segmentation
        /// </summary>
        private int lowerB = 0;
        private int upperB = 255;
        private int lowerG = 0;
        private int upperG = 255;
        private int lowerR = 0;
        private int upperR = 255;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            // initialize OpenCV window to display info from object segmentation
            this.InitializeCv();
        }

        private void InitializeCv()
        {
            CvWindow objectSeg = new CvWindow("Thresholded View");
            CvWindow colorPick = new CvWindow("Color Picker");
            CvTrackbarCallback lowerHCallback = delegate(int pos)
            {
                if (this.upperB > pos)
                {
                    this.lowerB = pos;
                }
            };
            CvTrackbarCallback upperHCallback = delegate(int pos)
            {
                if (pos > this.lowerB)
                {
                    this.upperB = pos;
                }
            };
            CvTrackbarCallback lowerSCallback = delegate(int pos)
            {
                if (this.upperG > pos)
                {
                    this.lowerG = pos;
                }
            };
            CvTrackbarCallback upperSCallback = delegate(int pos)
            {
                if (pos > this.lowerG)
                {
                    this.upperG = pos;
                }
            };
            CvTrackbarCallback lowerVCallback = delegate(int pos)
            {
                if (this.upperR > pos)
                {
                    this.lowerR = pos;
                }
            };
            CvTrackbarCallback upperVCallback = delegate(int pos)
            {
                if (pos > this.lowerR)
                {
                    this.upperR = pos;
                }
            };
            CvTrackbar lowerH = colorPick.CreateTrackbar("lower B", this.lowerB, 255, lowerHCallback);
            CvTrackbar upperH = colorPick.CreateTrackbar("upper B", this.upperB, 255, upperHCallback);
            CvTrackbar lowerS = colorPick.CreateTrackbar("lower G", this.lowerG, 255, lowerSCallback);
            CvTrackbar upperS = colorPick.CreateTrackbar("upper G", this.upperG, 255, upperSCallback);
            CvTrackbar lowerV = colorPick.CreateTrackbar("lower R", this.lowerR, 255, lowerVCallback);
            CvTrackbar upperV = colorPick.CreateTrackbar("upper R", this.upperR, 255, upperVCallback);
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.colorBitmap;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                // ColorFrameReder is IDisposable
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                        }

                        this.colorBitmap.Unlock();
                    }

                    // Segment Object

                    Mat mat = this.colorBitmap.ToMat();
                    Cv2.CvtColor(mat, mat, ColorConversion.RgbaToRgb);

                    Scalar lower = new Scalar(this.lowerB, this.lowerG, this.lowerR);
                    Scalar upper = new Scalar(this.upperB, this.upperG, this.upperR);
                    Cv2.InRange(mat, lower, upper, mat);

                    CvSize size = new CvSize(mat.Width / 2, mat.Height / 2);
                    Cv2.Resize(mat, mat, size, 0, 0, Interpolation.Linear);

                    Cv2.ImShow("Thresholded View", mat);

                    mat.Dispose();
                }
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
