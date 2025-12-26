#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import yaml
import os
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped

class TourTestNode:
    def __init__(self):
        rospy.init_node('tour_test_node', anonymous=False)
        
        # Find waypoints file
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('greenhouse_sim')
        yaml_path = os.path.join(pkg_path, 'param', 'waypoints.yaml')
        
        # Load Waypoints
        self.waypoints = {}
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('waypoints', {})
            rospy.loginfo(f"Loaded {len(self.waypoints)} waypoints from {yaml_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load waypoints: {e}")
            return

        # Move Base Client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

    def navigate_to(self, waypoint_id):
        wp_key = f"wp_{waypoint_id}"
        wp_data = self.waypoints.get(wp_key)
        
        if not wp_data:
            rospy.logerr(f"Waypoint {wp_key} not found in yaml!")
            return False
            
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        p = wp_data['pose']['position']
        o = wp_data['pose']['orientation']
        
        goal_x = p['x']
        goal_y = p['y']
        goal_qx = o['x']
        goal_qy = o['y']
        goal_qz = o['z']
        goal_qw = o['w']
        
        goal.target_pose.pose.position = Point(goal_x, goal_y, p['z'])
        goal.target_pose.pose.orientation = Quaternion(goal_qx, goal_qy, goal_qz, goal_qw)
        
        rospy.loginfo(f"Navigating to {wp_key} (X={goal_x}, Y={goal_y})...")
        self.client.send_goal(goal)
        
        # Set tolerances based on waypoint
        # Match with TEB planner tolerances (0.25m, 0.2rad)
        if waypoint_id == 1:
            dist_tolerance = 0.2       # 20cm
            angle_tolerance = 0.15     # ~8.6 degrees
            stable_time = 3.0          # 3 seconds stable
        else:
            dist_tolerance = 0.3       # 30cm (matching TEB's 0.25 + buffer)
            angle_tolerance = 0.25     # ~14.3 degrees (matching TEB's 0.2 + buffer)
            stable_time = 5.0          # 5 seconds stable
        
        rospy.loginfo(f"{wp_key}: Required precision - distance<{dist_tolerance}m, angle<{angle_tolerance}rad")
        
        # Manual Limit: Wait up to 60s
        start_time = rospy.Time.now()
        success_time = None
        
        while (rospy.Time.now() - start_time).to_sec() < 60.0:
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                return True
                
            # Check manual distance and orientation
            try:
                msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=0.1)
                curr_x = msg.pose.pose.position.x
                curr_y = msg.pose.pose.position.y
                curr_qx = msg.pose.pose.orientation.x
                curr_qy = msg.pose.pose.orientation.y
                curr_qz = msg.pose.pose.orientation.z
                curr_qw = msg.pose.pose.orientation.w
                
                # Calculate position distance
                dist = ((curr_x - goal_x)**2 + (curr_y - goal_y)**2)**0.5
                
                # Calculate orientation difference using quaternion dot product
                # dot = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w
                # angle_diff = 2 * acos(|dot|)
                import math
                dot = abs(curr_qx*goal_qx + curr_qy*goal_qy + curr_qz*goal_qz + curr_qw*goal_qw)
                dot = min(1.0, dot)  # Clamp to avoid numerical errors
                angle_diff = 2.0 * math.acos(dot)
                
                # Check BOTH position AND orientation for ALL waypoints
                if dist < dist_tolerance and angle_diff < angle_tolerance:
                    if success_time is None:
                        success_time = rospy.Time.now()
                        rospy.loginfo(f"{wp_key}: ✓ Within tolerance! dist={dist:.3f}m, angle={angle_diff:.3f}rad")
                    elif (rospy.Time.now() - success_time).to_sec() > stable_time:
                        rospy.logwarn(f"{wp_key}: ✅ GOAL REACHED! (stable for {stable_time}s, dist={dist:.3f}m, angle={angle_diff:.3f}rad)")
                        self.client.cancel_goal()
                        return True
                else:
                    # Provide feedback on what's not met
                    if dist >= dist_tolerance or angle_diff >= angle_tolerance:
                        feedback_parts = []
                        if dist >= dist_tolerance:
                            feedback_parts.append(f"dist={dist:.3f}m (need<{dist_tolerance}m)")
                        if angle_diff >= angle_tolerance:
                            feedback_parts.append(f"angle={angle_diff:.3f}rad (need<{angle_tolerance}rad)")
                        rospy.loginfo_throttle(3.0, f"{wp_key}: ⏳ " + ", ".join(feedback_parts))
                    success_time = None
                    
            except Exception as e:
                pass
                
            rospy.sleep(0.5)
            
        rospy.logerr(f"{wp_key}: ❌ TIMEOUT after 60s!")
        self.client.cancel_goal()
        return False

    def run_tour(self):
        rospy.loginfo("Starting Sequential Tour...")

        # Extract waypoint indices
        wp_indices = []
        for key in self.waypoints.keys():
            if key.startswith('wp_'):
                try:
                    idx = int(key.split('_')[1])
                    wp_indices.append(idx)
                except ValueError:
                    continue
        
        # Sort indices to ensure order 1, 2, 3...
        wp_indices.sort()
        
        if not wp_indices:
            rospy.logwarn("No waypoints found (keys starting with 'wp_')")
            return

        rospy.loginfo(f"Found {len(wp_indices)} waypoints in sequence: {wp_indices}")

        for i in wp_indices:
            if not self.run_step(i):
                rospy.logerr(f"Tour interrupted at waypoint {i}")
                return
            rospy.sleep(1.0) # Short pause at each waypoint
                
        rospy.loginfo("Tour Completed Successfully.")

    def run_step(self, wp_id):
        rospy.loginfo(f"--- Navigating to Waypoint {wp_id} ---")
        success = self.navigate_to(wp_id)
        if success:
            rospy.loginfo(f"Reached Waypoint {wp_id}.")
            return True
        else:
            rospy.logwarn(f"Failed to reach Waypoint {wp_id}.")
            return False

if __name__ == '__main__':
    try:
        node = TourTestNode()
        node.run_tour()
    except rospy.ROSInterruptException:
        pass
