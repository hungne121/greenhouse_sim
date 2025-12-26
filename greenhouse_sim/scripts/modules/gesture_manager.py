#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class GestureManager:
    """
    Layer 4: Action Layer
    Handles body language (Gestures) for Pepper Robot.
    """
    def __init__(self):
        self.enabled = True
        rospy.loginfo("[GestureManager] Initializing connection to joint controllers...")
        
        # Connect to Head and Arm Controllers individually
        self.head_client = actionlib.SimpleActionClient('/pepper_dcm/Head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.r_arm_client = actionlib.SimpleActionClient('/pepper_dcm/RightArm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        # Check connection (non-blocking mainly, but good to wait a bit)
        if self.head_client.wait_for_server(timeout=rospy.Duration(1.0)):
            rospy.loginfo("[GestureManager] Connected to Head Controller.")
        else:
            rospy.logwarn("[GestureManager] Head Controller NOT found.")
            
        if self.r_arm_client.wait_for_server(timeout=rospy.Duration(1.0)):
             rospy.loginfo("[GestureManager] Connected to Right Arm Controller.")
        else:
             rospy.logwarn("[GestureManager] Right Arm Controller NOT found.")

    def perform_gesture(self, gesture_name):
        """
        Execute a predefined gesture.
        """
        rospy.loginfo(f"🤖 [GestureManager] Performing gesture: {gesture_name}")
        
        if gesture_name == "wave_hand":
            self._wave_hand()
        elif gesture_name == "nod_head":
            self._nod_head()
        elif gesture_name == "look_around":
            self._look_around()
        elif gesture_name == "reset_pose":
            self._reset_pose()
    
    def _send_trajectory(self, client, joint_names, points):
        # We assume client is connected based on __init__ check, or we just fail gracefully if not.
        if client is None: 
             return

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = points
        # goal.trajectory.header.stamp = rospy.Time.now()
        client.send_goal(goal)

    def _wave_hand(self):
        # REQUIRED: [RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw]
        joints = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        
        # Wave: Lift arm and move elbow
        # P1: Unused joints set to 0.0 or neutral
        p1 = JointTrajectoryPoint()
        p1.positions = [-0.5, -0.5, 0.0, 0.5, 0.0]
        p1.time_from_start = rospy.Duration(1.0)
        
        p2 = JointTrajectoryPoint()
        p2.positions = [-0.5, -0.5, 0.0, 1.0, 0.0]
        p2.time_from_start = rospy.Duration(2.0)
        
        self._send_trajectory(self.r_arm_client, joints, [p1, p2])

    def _nod_head(self):
        # REQUIRED: [HeadYaw, HeadPitch]
        joints = ['HeadYaw', 'HeadPitch']
        
        # Nod: Pitch moves, Yaw stays 0
        p1 = JointTrajectoryPoint(); p1.positions = [0.0, 0.2]; p1.time_from_start = rospy.Duration(0.5)
        p2 = JointTrajectoryPoint(); p2.positions = [0.0, -0.1]; p2.time_from_start = rospy.Duration(1.0)
        p3 = JointTrajectoryPoint(); p3.positions = [0.0, 0.0]; p3.time_from_start = rospy.Duration(1.5)
        self._send_trajectory(self.head_client, joints, [p1, p2, p3])

    def _look_around(self):
        # REQUIRED: [HeadYaw, HeadPitch]
        joints = ['HeadYaw', 'HeadPitch']
        
        # Look Around: Yaw moves, Pitch stays 0
        p1 = JointTrajectoryPoint(); p1.positions = [0.5, 0.0]; p1.time_from_start = rospy.Duration(1.0)
        p2 = JointTrajectoryPoint(); p2.positions = [-0.5, 0.0]; p2.time_from_start = rospy.Duration(3.0)
        p3 = JointTrajectoryPoint(); p3.positions = [0.0, 0.0]; p3.time_from_start = rospy.Duration(4.0)
        self._send_trajectory(self.head_client, joints, [p1, p2, p3])

    def _reset_pose(self):
        """Reset both Head and Arms to neutral"""
        # Reset Head
        joints_head = ['HeadYaw', 'HeadPitch']
        p_head = JointTrajectoryPoint(); p_head.positions = [0.0, 0.0]; p_head.time_from_start = rospy.Duration(1.0)
        self._send_trajectory(self.head_client, joints_head, [p_head])
        
        # Reset Arm
        self._set_walk_pose()

    def _set_walk_pose(self):
        """Arms down, safe for walking. Does NOT touch Head."""
        joints_arm = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        # Arms close to body
        p_arm = JointTrajectoryPoint(); p_arm.positions = [1.5, -0.1, 0.0, 0.1, 0.0]; p_arm.time_from_start = rospy.Duration(1.0)
        self._send_trajectory(self.r_arm_client, joints_arm, [p_arm])

    def perform_gesture(self, gesture_name):
        """
        Execute a predefined gesture.
        """
        rospy.loginfo(f"🤖 [GestureManager] Performing gesture: {gesture_name}")
        
        if gesture_name == "wave_hand":
            self._wave_hand()
        elif gesture_name == "nod_head":
            self._nod_head()
        elif gesture_name == "look_around":
            self._look_around()
        elif gesture_name == "reset_pose":
            self._reset_pose()
        elif gesture_name == "walk_pose":
            self._set_walk_pose()
