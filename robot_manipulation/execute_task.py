#!/usr/bin/env python

from __future__ import print_function

import time
import os
import sys
import glob
import math

import rospy
import tf2_ros
import ros_numpy
import numpy as np
import cv2

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_ros
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_seq import MoveBaseSeq as base_controller

'''
Base class for all high-level task inferitance
'''
class ExecuteTaskNode():
    def __init__(self):
        self.joint_state = None
        self.point_cloud = None
        self.base_controller = base_controller

    def move_to_position(self, position):
        # send target x,y, position and angle of the 
        # robot to move_base action server
        points_seq = [-1.5,0.9,0, -1.6,0.4,0, -0.02,-1.6,0]
        angles_seq = [0,20,45]
        # send target position
        self. base_controller.set_target_position(points_seq, angles_seq)
        self.base_controller.move_to_pose()
        # clear_points_
        self.clear_points_seq()

    def point_cloud_callback(self, point_cloud):
        self.point_cloud = point_cloud

    def get_robot_floor_pose_xya(self, floor_frame='odom'):
        # Returns the current estimated x, y position and angle of the
        # robot on the floor. This is typically called with respect to
        # the odom frame or the map frame. x and y are in meters and
        # the angle is in euler angle degress.
        
        # Navigation planning is performed with respect to a height of
        # 0.0, so the heights of transformed points are 0.0. The
        # simple method of handling the heights below assumes that the
        # frame is aligned such that the z axis is normal to the
        # floor, so that ignoring the z coordinate is approximately
        # equivalent to projecting a point onto the floor.
        
        # Query TF2 to obtain the current estimated transformation
        # from the robot's base_link frame to the frame.
        robot_to_odom_mat, timestamp = get_p1_to_p2_matrix('base_link', floor_frame, self.tf2_buffer)

        # Find the robot's current location in the frame.
        r0 = np.array([0.0, 0.0, 0.0, 1.0])
        r0 = np.matmul(robot_to_odom_mat, r0)[:2]

        # Find the current angle of the robot in the frame.
        r1 = np.array([1.0, 0.0, 0.0, 1.0])
        r1 = np.matmul(robot_to_odom_mat, r1)[:2]
        robot_forward = r1 - r0
        r_ang = np.arctan2(robot_forward[1], robot_forward[0])

        return [r0[0], r0[1], r_ang], timestamp



def get_p1_to_p2_matrix(p1_frame_id, p2_frame_id, tf2_buffer, lookup_time=None, timeout_s=None):
    # If the necessary TF2 transform is successfully looked up, this
    # returns a 4x4 affine transformation matrix that transforms
    # points in the p1_frame_id frame to points in the p2_frame_id.
    try:
        if lookup_time is None:
            lookup_time = rospy.Time(0) # return most recent transform
        if timeout_s is None:
            timeout_ros = rospy.Duration(0.1)
        else:
            timeout_ros = rospy.Duration(timeout_s)
        stamped_transform =  tf2_buffer.lookup_transform(p2_frame_id, p1_frame_id, lookup_time, timeout_ros)
        # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TransformStamped.html
        p1_to_p2_mat = ros_numpy.numpify(stamped_transform.transform)
        return p1_to_p2_mat, stamped_transform.header.stamp
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print('WARNING: get_p1_to_p2_matrix failed to lookup transform from p1_frame_id =', p1_frame_id, ' to p2_frame_id =', p2_frame_id)
        print('         exception =', e)
        return None, None

def angle_diff_rad(target_rad, current_rad):
    # I've written this type of function many times before, and it's
    # always been annoying and tricky. This time, I looked on the web:
    # https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    diff_rad = target_rad - current_rad
    diff_rad = ((diff_rad + math.pi) % (2.0 * math.pi)) - math.pi
    return diff_rad
