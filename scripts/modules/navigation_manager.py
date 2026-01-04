#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Navigation Manager - Layer 2: Functional Module
Non-blocking wrapper for move_base with callback API
"""

import rospy
import actionlib
import math
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

class NavigationManager:
    """
    Non-blocking Navigation Manager for SMACH integration
    """
    
    # Status constants (mirror actionlib)
    IDLE = 0
    ACTIVE = 1
    SUCCEEDED = 3
    ABORTED = 4
    
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.loginfo("[NavigationManager] Waiting for move_base...")
        self.client.wait_for_server(timeout=rospy.Duration(60))
        
        self.current_status = self.IDLE
        
        # Odom for rotation
        self._last_odom_msg = None
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_cb)
        
        # Callbacks
        self.done_callback = None
        self.active_callback = None
        
    def _odom_cb(self, msg):
        """Callback to store latest odometry"""
        self._last_odom_msg = msg

    def backup(self, duration=1.0, speed=0.2):
        """
        Move backward safely
        """
        rospy.loginfo(f"[NavigationManager] Backing up for {duration}s...")
        cmd = Twist()
        cmd.linear.x = -abs(speed)
        
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start).to_sec() < duration:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
            
        # Stop
        self.cmd_vel_pub.publish(Twist())
        return self.client.get_state() == GoalStatus.SUCCEEDED
        self.client.wait_for_server(timeout=rospy.Duration(10))
        rospy.loginfo("[NavigationManager] Connected to move_base.")
        
        self.current_status = self.IDLE
        
        # Callbacks
        self.done_callback = None
        self.active_callback = None
    
    def send_goal(self, pose, done_cb=None, active_cb=None):
        """
        Send navigation goal (NON-BLOCKING)
        
        Args:
            pose: dict with 'position' and 'orientation'
            done_cb: function(status, result) called when goal finishes
            active_cb: function() called when goal becomes active
        
        Returns:
            None (use callbacks or get_current_status() to monitor)
        """
        # Store callbacks
        self.done_callback = done_cb
        self.active_callback = active_cb
        
        # Create goal
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = pose['position']['x']
        goal.target_pose.pose.position.y = pose['position']['y']
        goal.target_pose.pose.position.z = pose['position']['z']
        
        goal.target_pose.pose.orientation.x = pose['orientation']['x']
        goal.target_pose.pose.orientation.y = pose['orientation']['y']
        goal.target_pose.pose.orientation.z = pose['orientation']['z']
        goal.target_pose.pose.orientation.w = pose['orientation']['w']
        
        rospy.loginfo(f"[NavigationManager] Sending goal: ({pose['position']['x']:.1f}, {pose['position']['y']:.1f})")
        
        # Send goal with callbacks
        self.client.send_goal(
            goal,
            done_cb=self._internal_done_cb,
            active_cb=self._internal_active_cb
        )
        
        self.current_status = self.ACTIVE
    
    def _internal_active_cb(self):
        """Internal callback when goal becomes active"""
        rospy.loginfo("[NavigationManager] Goal is now active.")
        if self.active_callback:
            self.active_callback()
    
    def _internal_done_cb(self, status, result):
        """Internal callback when goal finishes"""
        self.current_status = status
        
        status_text = {
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED",
            GoalStatus.PREEMPTED: "PREEMPTED"
        }.get(status, "UNKNOWN")
        
        rospy.loginfo(f"[NavigationManager] Goal finished with status: {status_text}")
        
        if self.done_callback:
            self.done_callback(status, result)
    
    def cancel_goal(self):
        """
        Cancel current navigation goal (immediate)
        """
        rospy.loginfo("[NavigationManager] Cancelling goal...")
        self.client.cancel_goal()
        self.current_status = self.IDLE

    def backup(self, duration=1.0, speed=0.2):
        """Move backward safely"""
        rospy.loginfo(f"[NavigationManager] Backing up...")
        cmd = Twist()
        cmd.linear.x = -abs(speed)
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
            
        self.cmd_vel_pub.publish(Twist()) # Stop
    
    def send_velocity(self, linear_x, angular_z):
        """
        Send raw velocity command manually
        """
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
    
    def get_current_status(self):
        """
        Get current navigation status
        
        Returns:
            int: IDLE/ACTIVE/SUCCEEDED/ABORTED
        """
        return self.current_status
    
    def is_navigating(self):
        """Quick check if currently navigating"""
        return self.current_status == self.ACTIVE
    
    def wait_for_result(self, timeout=None):
        if timeout:
            self.client.wait_for_result(rospy.Duration(timeout))
        else:
            self.client.wait_for_result()
        
        return self.client.get_state() == GoalStatus.SUCCEEDED

    # ==========================
    # CLOSED-LOOP ROTATION
    # ==========================

    def rotate_relative(self, target_rad, speed=0.5, timeout=10.0):
        """
        Rotate relative to current matching Odom
        """
        return self.rotate_relative_with_interrupt(target_rad, speed, timeout, check_fn=None)

    def rotate_relative_with_interrupt(self, target_rad, speed=0.5, timeout=10.0, check_fn=None):
        """
        Rotate relative to current odom, but stop if check_fn() returns True.
        Returns: 'done' (reached target), 'interrupted' (check_fn True), 'timeout', 'failed'
        """
        start_yaw = self._get_yaw_from_odom()
        if start_yaw is None:
            rospy.logwarn("[NAV] Odom not ready for rotation")
            return 'failed'
            
        rate = rospy.Rate(10)
        start_t = rospy.Time.now()
        
        # Determine direction
        cmd = Twist()
        # Shortest path logic? 
        # Just simple sign for now since we usually do 3.14
        cmd.angular.z = abs(speed) if target_rad > 0 else -abs(speed)
        
        while (rospy.Time.now() - start_t).to_sec() < timeout:
            current_yaw = self._get_yaw_from_odom()
            if current_yaw is None: 
                rate.sleep()
                continue
            
            # Calc delta
            # Handle wrapping: (curr - start + PI) % 2PI - PI
            diff = current_yaw - start_yaw
            if diff > math.pi: diff -= 2*math.pi
            if diff < -math.pi: diff += 2*math.pi
             
            # Check progress
            # We want to turn target_rad. If target_rad is positive, we want diff to increase.
            # If target_rad is negative, we want diff to decrease.
            # So, we check if the signed difference has reached or passed the signed target.
            if target_rad > 0:
                if diff >= target_rad - 0.05: # Threshold
                    self.send_velocity(0, 0)
                    rospy.loginfo(f"[NAV] Rotation Complete. Turned: {diff:.2f}")
                    return 'done'
            else: # target_rad < 0
                if diff <= target_rad + 0.05: # Threshold
                    self.send_velocity(0, 0)
                    rospy.loginfo(f"[NAV] Rotation Complete. Turned: {diff:.2f}")
                    return 'done'
            
            # Check Interrupt
            if check_fn and check_fn():
                 self.send_velocity(0, 0)
                 rospy.loginfo(f"[NAV] Rotation Interrupted (User Found). Turned: {diff:.2f}")
                 return 'interrupted'
                
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
            
        self.send_velocity(0, 0)
        rospy.logwarn("[NAV] Rotation timed out")
        return 'timeout'

    def get_robot_pose(self):
        """
        Get current robot pose (x, y, yaw) from odometry
        Returns: (x, y, yaw) or None if not ready
        """
        if self._last_odom_msg is None:
            return None
            
        x = self._last_odom_msg.pose.pose.position.x
        y = self._last_odom_msg.pose.pose.position.y
        
        from tf.transformations import euler_from_quaternion
        q = self._last_odom_msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        return x, y, yaw

    def _get_yaw_from_odom(self):
        """
        Helper to get current yaw from the last received odometry message.
        Returns None if no odometry message has been received yet.
        """
        if self._last_odom_msg is None:
            return None
        
        from tf.transformations import euler_from_quaternion
        q = self._last_odom_msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

# Test
if __name__ == "__main__":
    rospy.init_node('nav_test')
    nav = NavigationManager()
    
    def on_done(status, result):
        print(f"Navigation done! Status: {status}")
    
    test_pose = {
        'position': {'x': 0.0, 'y': 2.0, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    }
    
    nav.send_goal(test_pose, done_cb=on_done)
    rospy.loginfo("Goal sent, main thread is free!")
    rospy.spin()
