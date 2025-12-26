#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Perception Manager - Layer 2: Functional Module
Handles sensor fusion for obstacle/human detection
UPDATED: Uses Standard Leg Detector (LIDAR)
"""

import rospy
import threading
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from people_msgs.msg import People # Standard output of simple_leg_detector

class PerceptionManager:
    """
    Perception Manager for obstacle and human detection
    """
    
    def __init__(self):
        self.nearest_obstacle_distance = float('inf')
        self.human_distance = float('inf')
        self.human_angle = 0.0 # Angle to human (rad)
        self.human_velocity = 0.0
        self.lock = threading.Lock()
        
        # Subscribe to Lidar (for general obstacles)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        
        # Subscribe to People (Standard)
        # Topic: /people (Published by simple_leg_detector.py)
        self.leg_sub = rospy.Subscriber('/people', People, self._leg_callback)
        
        rospy.loginfo("[PerceptionManager] Initialized (Lidar + Simple Leg Detector).")
    
    def _scan_callback(self, msg):
        """Process laser scan for obstacles"""
        valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        if valid_ranges:
            with self.lock:
                self.nearest_obstacle_distance = min(valid_ranges)
    
    def _leg_callback(self, msg):
        """
        Process detected people.
        """
        people = msg.people
        if not people:
            with self.lock:
                self.human_distance = float('inf')
            return
            
        closest_dist = float('inf')
        closest_angle = 0.0
        
        for person in people:
            # Position relative to fixed_frame (base_link)
            # Person.position is Point
            x = person.position.x
            y = person.position.y
            dist = math.sqrt(x*x + y*y)
            
            if dist < closest_dist:
                closest_dist = dist
                closest_angle = math.atan2(y, x)
                # Store velocity magnitude
                vx = person.velocity.x
                vy = person.velocity.y
                target_v = math.hypot(vx, vy)
        
        with self.lock:
            self.human_distance = closest_dist
            self.human_angle = closest_angle
            if closest_dist < 50.0: # If valid person found
                self.human_velocity = target_v
            else:
                self.human_velocity = 0.0
                
    def is_human_moving(self, threshold=0.15):
        """Check if the nearest human is moving (v > threshold)"""
        with self.lock:
            return self.human_velocity > threshold

    def get_nearest_obstacle_distance(self):
        """Get distance to nearest physical obstacle (Lidar)"""
        with self.lock:
            return self.nearest_obstacle_distance
    
    def get_nearest_human_distance(self):
        """
        Get distance to closest human (Leg Detector)
        Returns: float (meters)
        """
        with self.lock:
            return self.human_distance
            
    def get_human_angle(self):
        """Get angle to closest human (rad)"""
        with self.lock:
            return self.human_angle
    
    def is_path_blocked(self, threshold=0.5):
        """Check if path blocked"""
        return self.get_nearest_obstacle_distance() < threshold

# Test
if __name__ == "__main__":
    rospy.init_node('perception_test')
    perception = PerceptionManager()
    
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        dist = perception.get_nearest_human_distance()
        rospy.loginfo(f"Human Distance: {dist:.2f}m")
        rate.sleep()
