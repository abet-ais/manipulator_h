#!/usr/bin/env python
# coding: UTF-8

import rospy
import tf
import math
import numpy as np
# from time import sleep
import time
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from manipu_h_motion.srv import motion_traj
from manipulator_h_base_module_msgs.msg import JointPose



class Manipulator_h:
    def __init__(self):
        # rospy.init_node('execute_plan_of_moveit')
        self.ini_pose_msg_pub = rospy.Publisher('/robotis/base/ini_pose_msg',String,queue_size=0)
        self.set_mode_msg_pub = rospy.Publisher('/robotis/base/set_mode_msg',String,queue_size=0)
        rospy.Subscriber('/robotis/is_moving_triger',Bool,self.is_moving_triger_CB)

        robot = moveit_commander.RobotCommander()
        self.manipu = moveit_commander.MoveGroupCommander("arm")

        self.q = Quaternion()
        self.target_pose = Pose()
        self.triger = Bool()
        self.triger.data = False


    def is_moving_triger_CB(self,is_moving_triger):
        self.triger.data = is_moving_triger.data
        

    def set_mode(self):
        set_mode_msg = String()
        set_mode_msg = 'set_mode'
        time.sleep(4)
        self.set_mode_msg_pub.publish(set_mode_msg)


    def init_mode(self):
        print "Go to init pose"
        manipu_init_pose = self.manipu.get_current_pose().pose
        self.manipu.set_named_target("initial_pose")
        self.manipu.go()
        manipu_init_pose = self.manipu.get_current_pose().pose
        print "Now is Init Pose"
        
        self.ini_msg = String()
        self.ini_msg = 'ini_pose'
        self.ini_pose_msg_pub.publish(self.ini_msg)
        time.sleep(5)


    def motion(self,x,y,z,roll,pitch,yaw):
        manipu_init_pose = self.manipu.get_current_pose().pose

        self.x = x
        self.y = y
        self.z = z + 0.006
        self.roll  = math.radians(roll)
        self.pitch = math.radians(pitch)
        self.yaw   = math.radians(yaw)

        self.q = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.target_pose.position.x = self.x
        self.target_pose.position.y = self.y
        self.target_pose.position.z = self.z
        self.target_pose.orientation.x = self.q[0]
        self.target_pose.orientation.y = self.q[1]
        self.target_pose.orientation.z = self.q[2]
        self.target_pose.orientation.w = self.q[3]
        self.manipu.set_pose_target(self.target_pose)
        plan = self.manipu.plan()

        if self.manipu.execute(plan):
            print "This plan is succes!"
            joint_msg = rospy.Publisher('/robotis/base/joint_pose_msg',JointPose,queue_size=100)
            jointpose = JointPose()
            jointpose.name = plan.joint_trajectory.joint_names

            start = time.time()
            jointpose.value = plan.joint_trajectory.points[0].positions
            joint_msg.publish(jointpose)
            plan_count = 1
            while (plan_count < len(plan.joint_trajectory.points)):
                if (  (time.time()-start)>=0.8  ):
                    jointpose.value = plan.joint_trajectory.points[plan_count].positions
                    joint_msg.publish(jointpose)
                    plan_count = plan_count + 1
                    start = time.time()

            print "finished to send all jointTrajectory"
            
        else:
            print "Thin plan is NOT succes"
            return
        
