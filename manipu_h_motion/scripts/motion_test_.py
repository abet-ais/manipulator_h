#!/usr/bin/env python
# coding: UTF-8

import rospy
import tf
import math
import numpy as np
from time import sleep
# from geometry_msgs.msg import Quaternion
# from geometry_msgs.msg import Pose
# from moveit_msgs.msg import RobotTrajectory
# from manipu_h_motion.srv import motion_traj
from send_orbit_to_gazebo_2 import Manipulator_h



#  補足　#
# manipulatorHのZ軸方向は＋0.006[m]するとうまくいく。。。笑
#そのためsend_orbit_to_gazebo_2に0.006を足す機能を搭載した(2018/12/18)


def manipu_h_motion():
    rospy.init_node('manipu_motion')

    #--- インスタンス化 ---#
    manipu_h = Manipulator_h()


    #--- 初期化 ---#
    print "\n"*3
    print "initilaize"
    manipu_h.set_mode()
    manipu_h.init_mode()


    #--- ハンド関係パラメータ ---#
    # hand_length = 118.5 * 0.001
    hand_length = (119.00) * 0.001


    #--- 周辺部品パラメータ ---#
    sponge = 0.031
    steel_plate = 0.01
    deg_machine = 0.04
    washer_tray = 0.003


    #--- 部品ごとの高さパラメータ ---#
    M6_nat = 0.006   #スポンジの時は0.003出やるとちょうど良かった
    M6_washer = 0.001  #誤差として見られゼロでやるのが一番良かった
    M12_washer= 0.002
    spacer = 0.04
    fai6_spacer = 0.01

    #--------- motion -----------#


    print "\n","#--------- motion start ----------#","\n"

    print "Do you want to proceed to the next action? yes->y   no->n"
    input_word = raw_input(">>> ")
    if input_word == "y":
        print "go motion"
        z = sponge+0.04
        manipu_h.motion(0.20, 0.0, z+hand_length , 0 , 90.0 , 0 )

        print 
        #(x,y,z,先端回転,下向く,横向く)
    else:
        print "end motion"
        return
    
    sleep(2)

    print "Do you want to proceed to the next action? yes->y   no->n"
    input_word = raw_input(">>> ")
    if input_word == "y":
        print "go motion"
        z = sponge + fai6_spacer
        manipu_h.motion(0.20, 0.0 , z+hand_length , 0 , 90.0 , 0)
    else:
        print "end motion"
        return

    sleep(2)

    print "Do you want to proceed to the next action? yes->y   no->n"
    input_word = raw_input(">>> ")
    if input_word == "y":
        print "go motion"
        z = sponge + 0.02
        manipu_h.motion(0.20, 0.0, z+hand_length , 0 , 90.0 , 0 )
    else:
        print "end motion"
        return



    print "\n","#----------- motion end ----------#","\n"



if __name__=='__main__':
    try:
        manipu_h_motion()
    except rospy.ROSInterruptException: pass
