//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.PointingRecognition
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Documents;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Media.Media3D;
    using Microsoft.Kinect;
    using RosbridgeSender;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : System.Windows.Window, INotifyPropertyChanged
    {
        private WebSocket4Net.WebSocket wsClient;
        private string dataFormat;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private MultiSourceFrameReader multiFrameSourceReader = null;
        private WriteableBitmap bitmap = null;
        private uint bitmapBackBufferSize = 0;
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        private int displayWidth;
        private int displayHeight;
        private System.Windows.Rect displayRect;

        ///<summary>
        /// Array of modules
        ///</summary>
        private ParentModule[] moduleList = new ParentModule[4];

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // Instantiate all of the modules
            SpeechModule speechModule = new SpeechModule();
            ObjectsModule objectsModule = new ObjectsModule();
            FaceModule faceModule = new FaceModule();
            BodyModule bodyModule = new BodyModule();

            // build the list of modules
            moduleList[0] = speechModule;
            moduleList[1] = objectsModule;
            moduleList[2] = faceModule;
            moduleList[3] = bodyModule;

            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            //Checks if a kinectSensor is connected
            if (this.kinectSensor != null)
            {
                //Open the sensor
                this.kinectSensor.Open();

                // set sensors on all the modules
                foreach (ParentModule module in moduleList)
                {
                    module.setSensor(this.kinectSensor);
                }
            }
            else
            {
                // On failure, set the status text
                //this.statusBarText.Text = Properties.Resources.NoSensorStatusText;
                return;
            }

            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color);

            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            this.displayWidth = colorFrameDescription.Width;
            this.displayHeight = colorFrameDescription.Height;
            this.displayRect = new System.Windows.Rect(0.0, 0.0, this.displayWidth, this.displayHeight);

            this.bitmap = new WriteableBitmap(this.displayWidth, this.displayHeight, 96.0, 96.0, PixelFormats.Bgra32, null);
            this.bitmapBackBufferSize = (uint)((this.bitmap.BackBufferStride * (this.bitmap.PixelHeight - 1)) + (this.bitmap.PixelWidth * this.bytesPerPixel));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            // Create the RosBridge Client Sender
            RosbridgeSender rosbridgeSender = new RosbridgeSender();
            this.wsClient = rosbridgeSender.CreateClient(true);
            this.dataFormat = Microsoft.Samples.Kinect.PointingRecognition.Properties.Settings.Default.DataFormat;

            //Sets up the sending socket for each module
            foreach (ParentModule module in moduleList)
            {
                module.setSendingSocket(this.wsClient);
                module.setDataFormat(this.dataFormat);
            }

             //Test connection
        }

        private void SendData(string data)
        {
            if (Microsoft.Samples.Kinect.PointingRecognition.Properties.Settings.Default.SendData)
            {
                this.wsClient.Send(data);
            }
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

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
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.multiFrameSourceReader != null)
            {
                this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;
            }

            //Sets up the sending socket for each module
            foreach (ParentModule module in moduleList)
            {
                module.MainWindow_Loaded();
            }

            this.ColorDisplay.Source = this.getImageSource();
            this.ObjectsDisplay.Source = this.moduleList[1].getImageSource();
            this.FacesDisplay.Source = this.moduleList[2].getImageSource();
            this.BodiesDisplay.Source = this.moduleList[3].getImageSource();
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            // shut down each module in the moduleList
            foreach (ParentModule module in moduleList)
            {
                module.MainWindow_Closing();
            }

            if (this.multiFrameSourceReader != null)
            {
                this.multiFrameSourceReader.Dispose();
                this.multiFrameSourceReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Returns the imageSource of ObjectsModule
        /// </summary>
        public ImageSource getImageSource()
        {
            return this.bitmap;
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            ColorFrame colorFrame = null;
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

                // if any frame has expired by the time we process this event, return.
                // the "finally" statement will Dispose any that are not null.
                if (colorFrame == null)
                {
                    return;
                }

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
            }
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
                this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                                : Properties.Resources.SensorNotAvailableStatusText;
            }
        }
    }
}
