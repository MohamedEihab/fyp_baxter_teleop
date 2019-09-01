#!/usr/bin/env python

import argparse
import rospy
import baxter_interface
import tf
import tf2_ros
import struct
import numpy as np

from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    TransformStamped
)

from baxter_core_msgs.msg import (
   EndpointState
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

def from_raw_to_transform(pose, frame_id, child_frame_id):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id

    t.transform.translation = pose.position
    t.transform.rotation = pose.orientation
    return t


pitch_rot = [0, 0.7071068, 0, 0.7071068]
br = tf2_ros.TransformBroadcaster()

#GLOBAL CONTROL
DEMO_RECORDING = False

#CONSTANTS
MATH_PI = 3.141592653589793238462643383279502884197169399375105820974944
PRECISION_TOLERANCE = 0.5
CONTROLLER_MAP = [0.640694, 0.908686, 0.503144] #FORWARD X, SIDE Y, UP Z
BAXTER_MAP = [1.0948, 1.2927, 1.3085]  
BAXTER_POSITION_TRANSFORM = [
    BAXTER_MAP[0]/CONTROLLER_MAP[0],
    BAXTER_MAP[1]/CONTROLLER_MAP[1],
    BAXTER_MAP[2]/CONTROLLER_MAP[2]
    ]

# Adjust the human's movements to baxter's limbs
def adjustPoseToBaxter(data, limb):
    pose = data

    quaternion_angles= (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)

    euler_angles= euler_from_quaternion(quaternion_angles)

    roll = euler_angles[0]#  + MATH_PI/2# red
    pitch = euler_angles[1] #+ MATH_PI/4
    yaw = euler_angles[2]   #

    pitch_rot = [0., 0.7071068, 0., 0.7071068]  # [ 0, 0, 0, 1 ]
    z_rot = [0., 0.7071068,  0., 0.7071068]
    pitch_rot2 = [0., 0.7071068, 0., -0.7071068]
    rot_to_y = tf.transformations.quaternion_multiply(quaternion_angles, pitch_rot)
    correction = tf.transformations.quaternion_multiply(rot_to_y, z_rot)
    corrected_quat = tf.transformations.quaternion_multiply(correction, pitch_rot2)
    # corrected_quat = tf.transformations.quaternion_multiply(pitch_rot , y_axis_quat)  # quaternion_angles)

    #corrected_quat = quaternion_from_euler(roll, pitch, yaw, 'sxyz')
    pose.orientation.x = corrected_quat[0]
    pose.orientation.y = corrected_quat[1]
    pose.orientation.z = corrected_quat[2]
    pose.orientation.w = corrected_quat[3]

    pose.position.x = pose.position.x * BAXTER_POSITION_TRANSFORM[0] #* tr[0]#pose.position.x + v2[2] - v1[2]) * 1.5
    pose.position.y = pose.position.y * BAXTER_POSITION_TRANSFORM[1]#* 1.5
    pose.position.z = pose.position.z * BAXTER_POSITION_TRANSFORM[2]#+ (v2[2] - v1[2])) * 1.6
    
    br.sendTransform(from_raw_to_transform(pose, "world", limb + "_as_goal"))

    return pose


class LimbTeleoperator():
    
    def __init__(self, limb):
        rospy.loginfo("Initiated LimbTeleoperator for limb: " + limb)
        self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self.limb = baxter_interface.Limb(limb)
        self.gripper = baxter_interface.Gripper(limb)
        self.limbTitle = limb;

        self.currentPose = Pose()
        self.goalPose = Pose()

        rospy.loginfo("Moving Limbs to neutral...")
        self.limb.set_joint_position_speed(1)
        self.limb.move_to_neutral()

        rospy.loginfo("Calibrating grippers...")
        self.gripper.set_velocity(100)
        self.gripper.calibrate()

        #Subscribers
        self.currentPoseSub = rospy.Subscriber("/robot/limb/" + limb + "/endpoint_state", EndpointState, self.currentPoseCallBack, queue_size = 1)
        self.goalPoseSub = rospy.Subscriber(limb + "_controller_as_posestamped", PoseStamped, self.goalPoseCallBack, queue_size = 1)

        #Testin
        #self.controller_sub = rospy.Subscriber(limb + "_controller_pose", PoseStamped, self.controller_cb)
        #self.controller_pose = Pose()
    #CallBack Functions
    def currentPoseCallBack(self, data):
        self.currentPose = data.pose
        #if (self.limbTitle == "left"):
            #print(self.currentPose)

    #
    def goalPoseCallBack(self, data):
        #data.pose.orientation = self.controller_pose.orientation
        self.goalPose = adjustPoseToBaxter(data.pose, self.limbTitle)

    #def controller_cb(self, posestamp):
    #    self.controller_pose = posestamp.pose

    def requestIK(self):
        #rospy.loginfo("Handling teleoperation for " + self.limbTitle + " limb")
        iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')

        goalPoseStamped = PoseStamped(hdr, self.goalPose)
        ikreq.pose_stamp.append(goalPoseStamped)

        try:
            rospy.wait_for_service(self.ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return

        if (resp.isValid[0]):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else:
            return self.limb.joint_angles()
        return
    
    def goalPoseAchieved(self):
        diff_position_x = abs(self.currentPose.position.x - self.goalPose.position.x) <= 0.05
        diff_position_y = abs(self.currentPose.position.y - self.goalPose.position.y) <= 0.05
        diff_position_z = abs(self.currentPose.position.z - self.goalPose.position.z) <= 0.05

        diff_orientation_x = (abs(self.currentPose.orientation.x - self.goalPose.orientation.x) <= 0.05)
        diff_orientation_y = (abs(self.currentPose.orientation.y - self.goalPose.orientation.y) <= 0.05)
        diff_orientation_z = (abs(self.currentPose.orientation.z - self.goalPose.orientation.z) <= 0.05)
        diff_orientation_w = (abs(self.currentPose.orientation.w - self.goalPose.orientation.w) <= 0.05)

        position_achieved = diff_position_x and diff_position_y and diff_position_z
        orientation_achieved = diff_orientation_x and diff_orientation_y and diff_orientation_z and diff_orientation_w
        return (position_achieved and orientation_achieved);

    def use_gripper(self, value):
        self.gripper.command_position(value * 100)

    def move(self):
        #if (self.goalPoseAchieved() == False):
        positions = self.requestIK()
        self.limb.set_joint_positions(positions)
        

class ControllerManager():
    def __init__(self):
        rospy.loginfo("Initiated ControllerManager")
        self.ControllerSub = rospy.Subscriber("/controllerInput", PoseStamped, self.controllerSubCb, queue_size = 5)
        self.Teleop_Allowed = False;
        self.DemoRecord = False;

        self.LeftHand = 0;
        self.RightHand = 0;
        

    def controllerSubCb(self, data):
        if (data.pose.position.x == 100.0):
            self.Teleop_Allowed = True
        else:
            self.Teleop_Allowed = False

        if (data.pose.position.z == 100.0):
            self.DemoRecord = True
        else:
            self.DemoRecord = False

        if (data.header.frame_id == "LeftHand"):
            self.LeftHand = data.pose.position.y
        elif (data.header.frame_id == "RightHand"):
            self.RightHand = data.pose.position.y

    def getAllowedToTeleop(self):
        return self.Teleop_Allowed;

    def getLeftHand(self):
        return self.LeftHand

    def getRightHand(self):
        return self.RightHand



def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class = arg_fmt, description = main.__doc__)

    rospy.loginfo("Initilizing BaxterClient Node")
    parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("BaxterClient")

    #Head
    baxterHead = baxter_interface.Head()
    baxterHead.set_pan(0)

    #Limb
    leftTeleoperator = LimbTeleoperator("left")
    rightTeleoperator = LimbTeleoperator("right")
    
    cManager = ControllerManager()
    #rospy.sleep(5)         
    rospy.loginfo("Ready!")
    while not rospy.is_shutdown():
        #print("running")
        if (cManager.getAllowedToTeleop() == True):
            print("Allowed_Teleop")
            leftTeleoperator.move()
            rightTeleoperator.move()
        else:
            print("Not allowed to Teleop")

        leftTeleoperator.use_gripper(cManager.getLeftHand())
        rightTeleoperator.use_gripper(cManager.getRightHand())

        #print("about to handle")

    #rospy.spin()


if __name__ == '__main__':
    main()

'''    quaternion_angles= (
        pose.orientation.x,
        -pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)

    euler_angles= euler_from_quaternion(quaternion_angles)

    roll = euler_angles[0]
    pitch = euler_angles[1] + MATH_PI/4
    yaw = euler_angles[2]

    corrected_quat = quaternion_from_euler(roll, pitch, yaw, 'sxyz')
    pose.orientation.x = corrected_quat[0]
    pose.orientation.y = corrected_quat[1]
    pose.orientation.z = corrected_quat[2]
    pose.orientation.w = corrected_quat[3]

    pose.position.x = pose.position.x * 1.5
    pose.position.y = pose.position.y * 1.5
    pose.position.z = pose.position.z + (v2[2] - v1[2])'''