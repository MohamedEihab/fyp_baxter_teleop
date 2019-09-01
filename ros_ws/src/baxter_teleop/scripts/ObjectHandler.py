#!/usr/bin/python2


import argparse
import sys
import tf
import rospy
from find_object_2d.msg import ObjectsStamped
import baxter_interface
from geometry_msgs.msg import (
    PoseStamped,
    Pose)
from std_msgs.msg import (
    Float32MultiArray
    )
from std_msgs.msg import Header
import os
import fnmatch

OBJECTS_PATH = '/home/mohamed/ros_ws/find_object_session/objects/'

# Matching Object_Id to Object_Name
objects_ids = {}
#Matches Object_name to IDs
objects_names = {}
active_objects = {} # zero if no object present

def GetObjectIds(path):
    obj_folders = next(os.walk(path))[1]
    for obj_name in obj_folders:
        objects_names[obj_name] = []
        for root, dirs, files in os.walk(path + obj_name):
            for file in files:
                objs = os.path.splitext(file)[0]
                objects_ids[objs] = obj_name
                objects_names[obj_name].append(objs)

class gameObject():
    def __init__(self, obj_id, width, height, pose):
        self.obj_id = obj_id
        self.width = width
        self.height = height
        self.pose = pose
        self.active = False

    def printInfo(self):
        print("ID:" + self.obj_id + " width:" + str(self.width) + " height:" + str(self.height) + " pose:" + self.pose)

class objectHandler():
    def __init__(self):
        self.objectsSubscriber = rospy.Subscriber("objectsStamped", ObjectsStamped, self.objectsCallback, queue_size = 1)
        self.tf_l = tf.TransformListener()
        self.gameObjects = {}
        self.pub = rospy.Publisher('virtualObjects', PoseStamped, queue_size=10)

    def objectsCallback(self, data):
        #obj_pose = PoseStamped()
        #obj_pose =self.tf_l.lookupTransform("world", "object_23", rospy.Time())#, obj_pose)
        #print(obj_pose)
        for x in range(0, len(data.objects.data), 12):

            obj_id = str(int(data.objects.data[x]))
            obj_name = objects_ids[obj_id]

            if (self.gameObjects.has_key(obj_id)):
                try:
                    obj_pose = self.tf_l.lookupTransform("world", "object_" + obj_id, rospy.Time())#, obj_pose)
                    self.gameObjects[obj_id].pose = obj_pose
                    active_objects[obj_name] = obj_id
                except:
                    print("FAILURE_1")
            else:
                try:
                    obj_pose = self.tf_l.lookupTransform("world", "object_" + obj_id, rospy.Time())#, obj_pose)
                    self.gameObjects[obj_id] = gameObject(obj_id, data.objects.data[x + 1], data.objects.data[x + 2], obj_pose)
                    active_objects[obj_name] = obj_id
                except:
                    print("FAILURE_2")
            #print(data.objects.data)
            #print(self.gameObjects[obj_id].obj_id, self.gameObjects[obj_id].pose)

    def objectsPublish(self):
        for k in active_objects.values():#self.gameObjects:
            key = str(k)
            name = objects_ids[key]
            width = str(self.gameObjects[key].width)
            height = str(self.gameObjects[key].height)
            if (name == "desk"):
                hdr = Header(stamp=rospy.Time.now(), frame_id=str(name))
            else:
                hdr = Header(stamp=rospy.Time.now(), frame_id=str(name+"_"+width+"_"+height))

            pose = Pose()
            pose.position.x = self.gameObjects[key].pose[0][0]
            pose.position.y = self.gameObjects[key].pose[0][1]
            pose.position.z = self.gameObjects[key].pose[0][2]
            pose.orientation.x = self.gameObjects[key].pose[1][0]
            pose.orientation.y = self.gameObjects[key].pose[1][1]
            pose.orientation.z = self.gameObjects[key].pose[1][2]
            pose.orientation.w = self.gameObjects[key].pose[1][3]

            _poseStamped = PoseStamped(hdr, pose)
            print(_poseStamped.header.frame_id)

            self.pub.publish(_poseStamped)

        #print(data.objects.data[0],data.objects.data[1],data.objects.data[2]);

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class = arg_fmt, description = main.__doc__)

    rospy.loginfo("Initilizing node")
    print("Initilizing node - objectHandler")
    parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("object_handler")
    #fakeObject = gameObject("1", 100, 100, Pose())
    handler = objectHandler()
    #fakeObject.printInfo()
    #while not rospy.is_shutdown():

        # Python code to search .mp3 files in current 
    # folder (We can change file type/name and path 
    # according to the requirements. 
    # This is to get the directory that the program 
    # is currently running in. 
    #dir_path = os.path.dirname(os.path.realpath(__file__)) 
    #print(dir_path)
    
    GetObjectIds(OBJECTS_PATH)

    #for key, val in objects_names.items():
        #print(key, val)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        handler.objectsPublish()
        GetObjectIds(OBJECTS_PATH) # just incase new objects added
        r.sleep()

if __name__ == '__main__':
    main()
