namespace Microsoft.Samples.Kinect.PointingRecognition
{
    using System;
    using System.Collections.Generic;
    using System.Windows.Media;
    using System.IO;
    using System.Text;
    using System.ComponentModel;
    using System.Runtime.InteropServices;
    using Microsoft.Speech.AudioFormat;
    using Microsoft.Speech.Recognition;
    using Microsoft.Kinect;

    public class SpeechModule : ParentModule
    {
        private WebSocket4Net.WebSocket socket;

        private string dataFormat;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Stream for 32b-16b conversion.
        /// </summary>
        private KinectAudioStream convertStream = null;

        /// <summary>
        /// Speech recognition engine using audio data from Kinect.
        /// </summary>
        private SpeechRecognitionEngine speechEngine = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

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
        /// Sets up format of messages between the Windows application and ROS
        /// </summary>
        public void setDataFormat(string format)
        {
            this.dataFormat = format;
        }

        /// <summary>
        /// Returns the ImageSource of the SpeechModule
        /// </summary>
        public ImageSource getImageSource()
        {
            // No video display for SpeechModule
            return null;
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        public void MainWindow_Loaded()
        {
            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // set the status text
            this.statusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            if (this.kinectSensor != null)
            {
                // grab the audio stream
                IReadOnlyList<AudioBeam> audioBeamList = this.kinectSensor.AudioSource.AudioBeams;
                System.IO.Stream audioStream = audioBeamList[0].OpenInputStream();

                // create the convert stream
                this.convertStream = new KinectAudioStream(audioStream);
            }
            else
            {
                return;
            }

            RecognizerInfo ri = TryGetKinectRecognizer();
            if (null != ri)
            {
                this.speechEngine = new SpeechRecognitionEngine(ri.Id);

                // Create a grammar from grammar definition XML file.
                using (var memoryStream = new MemoryStream(Encoding.ASCII.GetBytes(Properties.Resources.SpeechGrammar)))
                {
                    var g = new Grammar(memoryStream);
                    this.speechEngine.LoadGrammar(g);
                }

                this.speechEngine.SpeechRecognized += this.SpeechRecognized;
                this.speechEngine.SpeechRecognitionRejected += this.SpeechRejected;

                // let the convertStream know speech is going active
                this.convertStream.SpeechActive = true;

                // For long recognition sessions (a few hours or more), it may be beneficial to turn off adaptation of the acoustic model. 
                // This will prevent recognition accuracy from degrading over time.
                ////speechEngine.UpdateRecognizerSetting("AdaptationOn", 0);

                this.speechEngine.SetInputToAudioStream(
                    this.convertStream, new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
                this.speechEngine.RecognizeAsync(RecognizeMode.Multiple);
            }
        }

        /// <summary>
        /// Gets the metadata for the speech recognizer (acoustic model) most suitable to
        /// process audio from Kinect device.
        /// </summary>
        /// <returns>
        /// RecognizerInfo if found, <code>null</code> otherwise.
        /// </returns>
        private static RecognizerInfo TryGetKinectRecognizer()
        {
            IEnumerable<RecognizerInfo> recognizers;

            // This is required to catch the case when an expected recognizer is not installed.
            // By default - the x86 Speech Runtime is always expected. 
            try
            {
                recognizers = SpeechRecognitionEngine.InstalledRecognizers();
            }
            catch (COMException)
            {
                return null;
            }

            foreach (RecognizerInfo recognizer in recognizers)
            {
                string value;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                if ("True".Equals(value, StringComparison.OrdinalIgnoreCase) && "en-US".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                {
                    return recognizer;
                }
            }

            return null;
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        public void MainWindow_Closing()
        {
            if (null != this.convertStream)
            {
                this.convertStream.SpeechActive = false;
            }

            if (null != this.speechEngine)
            {
                this.speechEngine.SpeechRecognized -= this.SpeechRecognized;
                this.speechEngine.SpeechRecognitionRejected -= this.SpeechRejected;
                this.speechEngine.RecognizeAsyncStop();
            }
        }

        /// <summary>
        /// Handler for recognized speech events.
        /// </summary>
        /// <param name="sender">object sending the event.</param>
        /// <param name="e">event arguments.</param>
        private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            // Speech utterance confidence below which we treat speech as if it hadn't been heard
            const double ConfidenceThreshold = 0.3;

            if (e.Result.Confidence >= ConfidenceThreshold)
            {
                string format = this.dataFormat;
                string message = format.Replace("out_topic", "speech_info");
                switch (e.Result.Semantics.Value.ToString())
                {
                    case "YES":
                        message = message.Replace("XXXX", "yes");
                        this.SendData(message);
                        break;
                    case "NEXT":
                        message = message.Replace("XXXX", "next");
                        this.SendData(message);
                        break;
                    case "REPEAT":
                        message = message.Replace("XXXX", "repeat");
                        this.SendData(message);
                        break;
                    case "THIS ONE":
                        message = message.Replace("XXXX", "this one");
                        this.SendData(message);
                        break;
                }
            }
        }

        /// <summary>
        /// Handler for rejected speech events.
        /// </summary>
        /// <param name="sender">object sending the event.</param>
        /// <param name="e">event arguments.</param>
        private void SpeechRejected(object sender, SpeechRecognitionRejectedEventArgs e)
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
