#!/usr/bin/env python

import sys
import argparse

import rospy
import tf2_ros

import baxter_interface

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    TransformStamped
)

from baxter_interface import CHECK_VERSION

def from_raw_to_transform(pose, stamp, frame_id, child_frame_id,
                             to_ros_reference_frame=True):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id

    t.transform.translation = pose.position
    t.transform.rotation = pose.orientation
    return t


class ControllerListener():

    def __init__(self, title):
        rospy.loginfo("Initiated controller listener for: " + title)
        self.controller_title = title
        self.controller_pose = Pose()
        self.controller_sub = rospy.Subscriber(title + "_controller_pose", PoseStamped, self.controller_cb)#, queue_size = 1)

    def controller_cb(self, posestamp):
        self.controller_pose = posestamp.pose
        #print("RAW Controller Data: " + self.controller_title)
        #print(self.controller_pose)
        #print(self.return_transformation())

    def return_transformation(self):
        transformation = from_raw_to_transform(self.controller_pose, rospy.Time.now(), "world", self.controller_title + "_controller")
        return transformation

class HMDListener():

    def __init__(self):
        rospy.loginfo("Initiated HMD Listener")
        self.hmd_pose = Pose()
        self.hmd_sub = rospy.Subscriber("head_pose", PoseStamped, self.hmd_cb)#, queue_size = 1)

    def hmd_cb(self, posestamp):
        self.hmd_pose = posestamp.pose
        #print("RAW HMD Data:")
        #print(self.hmd_pose)
        #print(self.return_transformation())

    def return_transformation(self):
        transformation = from_raw_to_transform(self.hmd_pose, rospy.Time.now(), "world", "hmd")
        return transformation

def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("vive_client")

    leftControllerListener = ControllerListener("left")
    rightControllerListener = ControllerListener("right")
    headListener = HMDListener()

    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rate.sleep()
        transforms = []

        transforms.append(headListener.return_transformation())        
        transforms.append(rightControllerListener.return_transformation())        
        transforms.append(leftControllerListener.return_transformation())     

        br.sendTransform(transforms)

    print("Exiting - Vive client")

if __name__ == "__main__":
    main()
