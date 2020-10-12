#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Header


import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped



import tf_conversions

import tf2_ros




#
import ars_lib_helpers





class ArsSimSensorVelRobotRos:

  #######

  # Robot velocity subscriber
  robot_velocity_sub = None

  # Meas robot velocity pub
  meas_robot_velocity_pub = None


  # Robot Pose
  # TODO
  flag_robot_velocity_set = False
  robot_frame_id = ''
  robot_velocity_timestamp = rospy.Time()
  robot_posi = None
  robot_atti_quat_simp = None


  # Measurement sensor loop
  # freq
  meas_sens_loop_freq = None
  # Timer
  meas_sens_loop_timer = None
  


  #########

  def __init__(self):


    #
    self.flag_robot_velocity_set = False
    self.robot_frame_id = ''
    self.robot_velocity_timestamp = rospy.Time()
    self.robot_posi = np.zeros((3,), dtype=float)
    self.robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()


    # Measurement sensor loop
    # freq
    self.meas_sens_loop_freq = 1.0
    # Timer
    self.meas_sens_loop_timer = None


    # end
    return


  def init(self, node_name='ars_sim_sensor_vel_robot_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_sensor_vel_robot')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###


    
    # End
    return


  def open(self):


    # Subscribers

    # 
    self.robot_velocity_sub = rospy.Subscriber('robot_velocity', TwistStamped, self.robotVelocityCallback)
    

    # Publishers

    # 
    self.meas_robot_velocity_pub = rospy.Publisher('meas_robot_velocity', TwistStamped, queue_size=1)


    # Timers
    #
    self.meas_sens_loop_timer = rospy.Timer(rospy.Duration(1.0/self.meas_sens_loop_freq), self.measSensorLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def robotVelocityCallback(self, robot_velocity_msg):

    #
    self.flag_robot_velocity_set = True


    
    #
    return


  def measSensorLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    if(self.flag_robot_velocity_set == False):
      return

    #
    meas_robot_velocity_msg = TwistStamped()

    #
    meas_robot_velocity_msg.header.frame_id = self.robot_frame_id
    meas_robot_velocity_msg.header.stamp = self.robot_velocity_timestamp

    # Linear
    # TODO
    meas_robot_velocity_msg.twist.linear.x = 0.0
    meas_robot_velocity_msg.twist.linear.y = 0.0
    meas_robot_velocity_msg.twist.linear.z = 0.0


    # Angular
    # TODO
    meas_robot_velocity_msg.twist.angular.x = 0.0
    meas_robot_velocity_msg.twist.angular.y = 0.0
    meas_robot_velocity_msg.twist.angular.z = 0.0
    


    #
    self.meas_robot_velocity_pub.publish(meas_robot_velocity_msg)

    #
    return
