using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Reflection;
using System.Threading;
using System.Configuration;
using WebSocket4Net;

namespace RosbridgeSender 
{
    class RosbridgeSender
    {
        protected AutoResetEvent OpenedEvent = new AutoResetEvent(false);
        protected AutoResetEvent CloseEvent = new AutoResetEvent(false);
        protected AutoResetEvent MessageReceiveEvent = new AutoResetEvent(false);
        protected AutoResetEvent DataReceiveEvent = new AutoResetEvent(false);
        protected string CurrentMessage { get; private set; }
        protected byte[] CurrentData { get; private set; }

        public WebSocket4Net.WebSocket CreateClient(bool autoConnect)
        {
            WebSocketVersion version = WebSocketVersion.Rfc6455;
            return this.CreateClient(version, autoConnect);
        }

        public WebSocket4Net.WebSocket CreateClient(WebSocketVersion version, bool autoConnect)
        {
            String rosbridgeIP = Microsoft.Samples.Kinect.PointingRecognition.Properties.Settings.Default.IpAddress;
            var webSocketClient = new WebSocket(string.Format("ws://" + rosbridgeIP + ":{0}", 9090));
            webSocketClient.Opened += new EventHandler(webSocketClient_Opened);
            webSocketClient.Closed += new EventHandler(webSocketClient_Closed);
            webSocketClient.DataReceived += new EventHandler<DataReceivedEventArgs>(webSocketClient_DataReceived);
            webSocketClient.MessageReceived += new EventHandler<MessageReceivedEventArgs>(webSocketClient_MessageReceived);
            webSocketClient.Error += new EventHandler<SuperSocket.ClientEngine.ErrorEventArgs>(webSocketClient_ErrorReceived);
            if (autoConnect && Microsoft.Samples.Kinect.PointingRecognition.Properties.Settings.Default.SendData)
            {
                webSocketClient.Open();

                if (!OpenedEvent.WaitOne(3000))
                    ///if(webSocketClient.Error
                    Console.WriteLine("Failed");
            }

            return webSocketClient;
        }

        static void Main(string[] args)
        {
            Console.WriteLine("STARTING ROSBRIDGE CONNECTION");
            RosbridgeSender c1 = new RosbridgeSender();
            var appClient = c1.CreateClient(WebSocketVersion.Rfc6455, true);
            string json = @"{""op"":""advertise"",""topic"":""/chatter"",""type"":""std_msgs/String""}";

            //appClient.Send("{'op': 'advertise', 'topic': '/chatter', 'type': 'std_msgs/String'}");
            appClient.Send(json);
            for (int i = 0; i < 10; i++)
            {
                string json2 = @"{""op"":""publish"",""topic"":""/chatter"",""msg"":{""data"":""Hello World!""}}";
                appClient.Send(json2);
                //appClient.Send("{'op': 'publish', 'topic': '/chatter', 'msg': {'data': 'Hello world!'}}");
                Thread.Sleep(1000);
            }

            if (!c1.MessageReceiveEvent.WaitOne(1000)) // can remove if not receiving msgs
                Console.WriteLine("Cannot get response in time!");
        }

        void webSocketClient_MessageReceived(object sender, MessageReceivedEventArgs e)
        {
            CurrentMessage = e.Message;
            MessageReceiveEvent.Set();
        }

        void webSocketClient_DataReceived(object sender, DataReceivedEventArgs e)
        {
            CurrentData = e.Data;
            DataReceiveEvent.Set();
        }

        void webSocketClient_Closed(object sender, EventArgs e)
        {
            CloseEvent.Set();
        }

        void webSocketClient_Opened(object sender, EventArgs e)
        {
            OpenedEvent.Set();
            Console.WriteLine("YAY ws client opened!");
        }

        void webSocketClient_ErrorReceived(object sender, SuperSocket.ClientEngine.ErrorEventArgs e)
        {
            Console.WriteLine("HOT DAMN! AIN'T NO FUN TONIGHT. ERROR RECEIVED!!");
            Console.WriteLine(((SuperSocket.ClientEngine.ErrorEventArgs)e).Exception.Message);
            Thread.Sleep(4000);
        }
    }
}
