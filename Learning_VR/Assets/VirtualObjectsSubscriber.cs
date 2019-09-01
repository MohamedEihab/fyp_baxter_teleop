using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System.Linq;

namespace RosSharp.RosBridgeClient
{
    public class VirtualObjectsSubscriber : Subscriber<MessageTypes.Geometry.PoseStamped>
    {
        private Transform gameObjectTransform;
        private Header header;
        private Vector3 position;
        private Quaternion rotation;
        private bool isMessageReceived;
        static List<GameObject> virtualObjects = new List<GameObject>();

        public List<GameObject> thePrefabs;
        public Transform deskTransform;


        protected override void Start()
        {
            base.Start();

        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(MessageTypes.Geometry.PoseStamped message)
        {
            position = GetPosition(message).Ros2Unity();
            rotation = GetRotation(message).Ros2Unity();
            isMessageReceived = true;
            header = message.header;

        }

        private void ProcessMessage()
        {
            if (header.frame_id.Contains("desk"))
            {
                Vector3 desk_position = deskTransform.position;
                desk_position[1] = position.y;
                deskTransform.position = desk_position;
                return;
            }
            else {
                if (virtualObjects.Exists(x => x.name == header.frame_id))
                {
                    virtualObjects.Find(x => x.name == header.frame_id).transform.position = position;
                    virtualObjects.Find(x => x.name == header.frame_id).transform.rotation = rotation;
                }
                else
                {
                    int pre;
                    if (header.frame_id.Contains("sphere"))
                    {
                        pre = 0;
                    }
                    else
                    {
                        pre = 1;
                    }
                    GameObject instance = Instantiate(thePrefabs[pre], transform.position, transform.rotation);
                    instance.name = header.frame_id;
                    virtualObjects.Add(instance);
                    Debug.Log(virtualObjects.Capacity);
                }

            }
//Debug.Log(header.frame_id);
        }

        private Vector3 GetPosition(MessageTypes.Geometry.PoseStamped message)
        {
            return new Vector3(
                (float)message.pose.position.x,
                (float)message.pose.position.y,
                (float)message.pose.position.z);
        }

        private Quaternion GetRotation(MessageTypes.Geometry.PoseStamped message)
        {
            return new Quaternion(
                (float)message.pose.orientation.x,
                (float)message.pose.orientation.y,
                (float)message.pose.orientation.z,
                (float)message.pose.orientation.w);
        }
    }
}
