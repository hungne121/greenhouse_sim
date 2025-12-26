#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Navigation Manager - Layer 2: Functional Module
Non-blocking wrapper for move_base with callback API
"""

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

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
        # ...

    # ... (existing methods) ...

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
        """
        Blocking wait for current goal to finish (use sparingly in SMACH)
        
        Returns:
            bool: True if succeeded
        """
        if timeout:
            self.client.wait_for_result(rospy.Duration(timeout))
        else:
            self.client.wait_for_result()
        
        return self.client.get_state() == GoalStatus.SUCCEEDED

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
