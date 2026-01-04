#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Perception Manager - Optimized for Greenhouse Interaction
- Front: MediaPipe Pose for Intent Detection
- Rear: Sensor Fusion (MediaPipe Pose + Depth) for Precision Tracking
"""

import rospy
import threading
import math
import numpy as np
import tf
import cv2
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range, LaserScan

class PerceptionManager:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        
        # --- STATE VARIABLES ---
        # Front Intent
        self.front_person_detected = False
        self.front_person_angle = 0.0
        self.front_person_last_seen = rospy.Time(0)
        self.front_intent_start_time = None
        self.front_intent_confirmed = False
        
        # Rear Tracking
        self.rear_person_dist = float('inf')
        self.rear_person_angle = 0.0
        self.rear_detected = False
        self.rear_last_image = None # For fusion
        
        # --- MEDIAPIPE SETUP ---
        self.mp_pose = mp.solutions.pose
        self.front_pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
            
        self.rear_pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False, # Segmentation is heavy
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

        # Internal State (Smoothing)
        self.rear_smooth_dist = None
        
        # --- SUBSCRIBERS ---
        
        # 1. Front RGB (Intent)
        self.sub_front_rgb = rospy.Subscriber(
            '/pepper_robot/camera/front/image_raw', 
            Image, 
            self._front_rgb_callback, 
            queue_size=1, 
            buff_size=2**24
        )
        
        # 2. Rear RGB (Fusion Step 1)
        self.sub_rear_rgb = rospy.Subscriber(
            '/pepper_robot/camera/rear/image_raw',
            Image,
            self._rear_rgb_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # 3. Rear Depth (Fusion Step 2)
        self.sub_rear_depth = rospy.Subscriber(
            '/pepper_robot/depth/rear/image_raw',
            Image,
            self._rear_depth_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # 4. Rear Sonar (Safety Blind Spot)
        self.sub_sonar_back = rospy.Subscriber(
            '/pepper_robot/sonar_back', 
            Range, 
            self._sonar_back_callback
        )
        
        rospy.loginfo("[Perception] Initialized: MediaPipe Pose + NumPy Depth Fusion")

    # ===========================
    # FRONT SENSOR LOGIC
    # ===========================
    def _front_rgb_callback(self, msg):
        """
        Detect person in Front RGB using MediaPipe Pose.
        Logic: Center ROI + Dwell Time > 3.0s
        """
        try:
            # 1. Convert
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = cv_img.shape[:2]
            
            # DEBUG: Save frame once every 5s
            if int(rospy.Time.now().to_sec()) % 5 == 0:
                 cv2.imwrite("/tmp/front_debug.jpg", cv_img)

            # 2. MediaPipe Process (Needs RGB)
            image_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            results = self.front_pose.process(image_rgb)
            
            found_center_person = False
            
            if results.pose_landmarks:
                # Get Nose Landmark (ID 0)
                nose = results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.NOSE]
                
                # Check Visibility
                if nose.visibility > 0.5:
                    cx = nose.x * width
                    cy = nose.y * height
                    
                    # Calculate Front Angle (Updating Tracking ALWAYS)
                    # FOV ~ 1.01 rad (58 deg) | Center is 0
                    fov = 1.01
                    # (width/2 - cx) -> Positive if Left (Correct for Robot Frame Y+)
                    angle = ((width/2 - cx) / (width/2)) * (fov/2)
                    
                    with self.lock:
                        self.front_person_angle = float(angle)
                        self.front_person_last_seen = rospy.Time.now()
                    
                    # Check Center ROI (Central 40% of screen) - For Intent Detection Only
                    if (width * 0.3) < cx < (width * 0.7):
                        found_center_person = True
                    else:
                        rospy.logwarn_throttle(2.0, f"[PERCEPTION] Person Outside ROI (cx={cx/width:.2f})")
                else:
                    rospy.logwarn_throttle(2.0, f"[PERCEPTION] Person Low Visibility ({nose.visibility:.2f})")
            else:
                 rospy.logwarn_throttle(2.0, "[PERCEPTION] No Pose Detected")
            
            # 3. Timer Logic
            with self.lock:
                if found_center_person:
                    if not self.front_person_detected:
                        self.front_person_detected = True
                        self.front_detection_start_time = rospy.Time.now()
                        rospy.loginfo("[PERCEPTION] Front Person Detected (MediaPipe)")
                    
                    # Check Duration
                    if self.front_detection_start_time:
                         duration = (rospy.Time.now() - self.front_detection_start_time).to_sec()
                         rospy.loginfo_throttle(1.0, f"[PERCEPTION] Intent Timer: {duration:.1f}s / 3.0s")
                         if duration > 3.0:
                             self.front_intent_confirmed = True
                else:
                    self.front_person_detected = False
                    self.front_detection_start_time = None
                    self.front_intent_confirmed = False
                    self.front_person_angle = 0.0 # Reset

        except Exception as e:
            rospy.logwarn_throttle(10, f"Front CV Error: {e}")

    # ===========================
    # REAR SENSOR FUSION
    # ===========================
    def _rear_rgb_callback(self, msg):
        """Buffer Rear RGB for Fusion"""
        try:
            # Use copy() to ensure we own the memory and it's writable
            self.rear_last_image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
        except:
            pass

    def _rear_depth_callback(self, msg):
        """
        Process Rear Depth using MediaPipe ROI + NumPy NaN-Median
        """
        if self.rear_last_image is None:
            return
            
        try:
            # 1. Convert Depth
            # Gazebo usually 32FC1 (meters), containing NaNs for invalid
            cv_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough").copy()
            
            # 2. MediaPipe on RGB
            # Use Local Copy for thread safety
            cv_rgb = self.rear_last_image.copy()
            rgb_h, rgb_w = cv_rgb.shape[:2]
            depth_h, depth_w = cv_depth.shape[:2]
            
            image_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
            results = self.rear_pose.process(image_rgb)
            
            best_dist = float('inf')
            best_angle = 0.0
            found = False
            
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                
                # Calculate Bounding Box from Landmarks
                x_vals = [lm.x for lm in landmarks if lm.visibility > 0.5]
                y_vals = [lm.y for lm in landmarks if lm.visibility > 0.5]
                
                if x_vals and y_vals:
                    min_x, max_x = min(x_vals), max(x_vals)
                    min_y, max_y = min(y_vals), max(y_vals)
                    
                    # Map to Depth Coordinates (Normalized -> Pixel)
                    dx = int(min_x * depth_w)
                    dy = int(min_y * depth_h)
                    dw = int((max_x - min_x) * depth_w)
                    dh = int((max_y - min_y) * depth_h)
                    
                    # Clamp
                    dx = max(0, dx); dy = max(0, dy)
                    dw = min(dw, depth_w - dx); dh = min(dh, depth_h - dy)
                    
                    if dw > 0 and dh > 0:
                        # Extract ROI
                        roi_depth = cv_depth[dy:dy+dh, dx:dx+dw]
                        
                        # --- ROBUST DEPTH CALCULATION (NumPy) ---
                        # 1. Replace 0.0 with NaN (if Gazebo uses 0.0 for invalid)
                        roi_depth[roi_depth == 0.0] = np.nan
                        
                        # 2. Check overlap
                        valid_pixels = np.count_nonzero(~np.isnan(roi_depth))
                        
                        if valid_pixels > 0:
                            # 3. Use Percentile (25%) to bias towards foreground (person)
                            # Median fails if person < 50% of ROI (thin model)
                            current_dist = np.nanpercentile(roi_depth, 25)
                            
                            # 4. Low-Pass Filter (EMA) to prevent jumps
                            if self.rear_smooth_dist is None or abs(current_dist - self.rear_smooth_dist) > 0.5:
                                # Reset or Jump cut (trust new value if large jump, likely tracking catch-up)
                                self.rear_smooth_dist = current_dist
                            else:
                                alpha = 0.4 
                                self.rear_smooth_dist = alpha * current_dist + (1 - alpha) * self.rear_smooth_dist
                            
                            dist = float(self.rear_smooth_dist)
                            
                            # Calculate Angle
                            center_x = dx + dw/2
                            fov = 1.01 # 58 deg
                            angle = ((depth_w/2 - center_x) / (depth_w/2)) * (fov/2)
                            
                            best_dist = dist
                            best_angle = float(angle)
                            found = True
            
            with self.lock:
                if found:
                    self.rear_person_dist = best_dist
                    self.rear_person_angle = best_angle
                else:
                    self.rear_person_dist = float('inf')
                    
                self.rear_detected = found
                
                if found:
                     rospy.loginfo_throttle(0.5, f"[PERCEPTION] Rear Track: {best_dist:.2f}m @ {best_angle:.2f} rad")
                
        except Exception as e:
            rospy.logwarn_throttle(10, f"Rear CV Error: {e}")

    def _sonar_back_callback(self, msg):
        with self.lock:
            self.sonar_back_dist = msg.range

    # ===========================
    # PUBLIC API
    # ===========================
    def check_front_intent(self):
        with self.lock:
            return self.front_intent_confirmed
            
    def get_front_angle(self):
        with self.lock:
            return self.front_person_angle
            
    def get_front_track(self):
        """
        Get latest front tracking info
        Returns: (angle, found)
        """
        with self.lock:
             # Check if data is fresh (< 0.5s)
             if (rospy.Time.now() - self.front_person_last_seen).to_sec() < 0.5:
                 return self.front_person_angle, True
             else:
                 return 0.0, False
                 
    def get_rear_track(self):
        with self.lock:
            return self.rear_person_dist, self.rear_person_angle, self.rear_detected
            
    def get_sonar_back(self):
        with self.lock:
            return self.sonar_back_dist
            
    def get_front_blob_status(self):
        with self.lock:
            return self.front_person_detected

if __name__ == "__main__":
    rospy.init_node('perception_v2')
    pm = PerceptionManager()
    rospy.spin()
