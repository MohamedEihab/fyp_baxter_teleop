using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json;
using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace RosSharp.RosBridgeClient.MessageTypes.Std
{
    public class ObjectsStamped : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "find_object_2d/ObjectsStamped";
        public Header header;
        public Float32MultiArray objects;

        //objects format: 
        //[ObjectId1, objectWidth, objectHeight, h11, h12, h13, h21, h22, h23, h31, h32, h33, ObjectId2...] 
        //where h## is a 3x3 homography matrix (h31 = dx and h32 = dy, see QTransform)

        public ObjectsStamped()
        {
            this.header = new Header();
            this.objects = new Float32MultiArray();
        }
        public ObjectsStamped(Header header, Float32MultiArray objects)
        {
            this.header = header;
            this.objects = objects;
        }
    }
}

namespace RosSharp.RosBridgeClient
{
    public class ObjectStampedSubscriber : Subscriber<MessageTypes.Std.ObjectsStamped>
    {


        private Float32MultiArray objects;
        private bool isMessageReceived;
        private float[] data;

        protected override void Start()
        {
            Debug.Log("Start ObjectSubscriber");

            base.Start();
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        private void ProcessMessage()
        {
            data = objects.data;
            Debug.Log("data:");
            Debug.Log(data[0].ToString());
        }

        protected override void ReceiveMessage(ObjectsStamped message)
        {
            objects = message.objects;
            isMessageReceived = true;
            //throw new System.NotImplementedException();
        }
    }
}