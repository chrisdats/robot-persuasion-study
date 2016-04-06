using System;
using System.Windows.Media;
using Microsoft.Kinect;

namespace Microsoft.Samples.Kinect.PointingRecognition
{
    interface ParentModule
    {
        ///<summary>
        /// Sets up Kinect Sensor
        ///</summary>
        void setSensor(KinectSensor sensor);

        ///<summary>
        ///Set up socket for network connection between the client and server
        ///</summary>
        void setSendingSocket(WebSocket4Net.WebSocket socket);

        /// <summary>
        /// Sets up format of messages between the Windows application and ROS
        /// </summary>
        void setDataFormat(string format);

        /// <summary>
        /// Returns the imageSource of module
        /// </summary>
        ImageSource getImageSource();

        /// <summary>
        /// Checks whether data should be sent before sending using socket
        /// </summary>
        void SendData(string data);

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        void MainWindow_Loaded();

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        void MainWindow_Closing();
    }
}
