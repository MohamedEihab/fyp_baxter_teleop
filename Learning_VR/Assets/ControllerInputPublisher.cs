using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;

namespace RosSharp.RosBridgeClient
{
    public class ControllerInputPublisher : Publisher<MessageTypes.Geometry.PoseStamped>
    {

        public SteamVR_Input_Sources handType; // 1
        public SteamVR_Action_Boolean teleopAction; // 2
        public SteamVR_Action_Single grabAction; // 3
        public SteamVR_Action_Boolean demoAction; // 4

        private Vector3 controller_input = new Vector3();
        private Quaternion quaternion  = new Quaternion();

        private MessageTypes.Geometry.PoseStamped message;

        public static bool teleop_toggle = false;
        public static bool demo_toggle = false;


        protected override void Start()
        {
            base.Start();
            InitializeMessage();

        }

        private void Update()
        {
            UpdateMessage();

            if (teleopAction.GetStateDown(handType))
            {
                teleop_toggle = !teleop_toggle;
                //print("Teleop_toggle " + teleop_toggle);
            }

            if (demoAction.GetStateDown(handType))
            {
                demo_toggle = !demo_toggle;
                //print("Teleop_toggle " + teleop_toggle);
            }

            //  float TriggerValue = grabAction.GetAxis(handType);
            //if (TriggerValue > 0)
            // {
            //      Debug.Log(TriggerValue);
            //  }
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.PoseStamped
            {
                header = new MessageTypes.Std.Header()
                {
                    frame_id = handType.ToString()
                }
            };
        }


        private void UpdateMessage()
        {
            message.header.Update();
            message.pose.position = GetGeometryPoint(controller_input.Unity2Ros());
            message.pose.orientation = GetGeometryQuaternion(quaternion.Unity2Ros());

            Publish(message);
        }
        // x for teleop
        // y for grab
        private MessageTypes.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            MessageTypes.Geometry.Point geometryPoint = new MessageTypes.Geometry.Point();
            if (teleop_toggle)
            {
                geometryPoint.x = 100;
            }
            else
            {
                geometryPoint.x = 0;
            }

            geometryPoint.y = (1 - grabAction.GetAxis(handType));

            if (demo_toggle)
            {
                geometryPoint.z = 100;
            }
            else
            {
                geometryPoint.z = 0;
            }

            return geometryPoint;
        }

        private MessageTypes.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
        {
            MessageTypes.Geometry.Quaternion geometryQuaternion = new MessageTypes.Geometry.Quaternion();
            return geometryQuaternion;
        }
    }
}

