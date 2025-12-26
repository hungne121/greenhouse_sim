#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import os
import rospkg
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

class WaypointManager:
    def __init__(self):
        self.waypoints = {}
        self.plant_map = {} # Maps "plant name" -> wp_id
        self.load_waypoints()

    def load_waypoints(self):
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('greenhouse_sim')
            yaml_path = os.path.join(pkg_path, 'param', 'waypoints.yaml')
            
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                self.waypoints = data.get('waypoints', {})
                
            # Build plant map
            for key, val in self.waypoints.items():
                if 'plant_type' in val:
                    p_name = val['plant_type'].lower().strip()
                    self.plant_map[p_name] = key # e.g. "hoa hồng" -> "wp_1"
            
            rospy.loginfo(f"[WaypointManager] Loaded {len(self.waypoints)} waypoints and {len(self.plant_map)} plants.")
            
        except Exception as e:
            rospy.logerr(f"[WaypointManager] Failed to load waypoints: {e}")

    def get_waypoint_pose(self, wp_id_or_key):
        """
        Returns pose dictionary {'position':..., 'orientation':...}
        Accepts integer ID (1) or string key ("wp_1")
        """
        key = wp_id_or_key
        if isinstance(wp_id_or_key, int):
            key = f"wp_{wp_id_or_key}"
            
        wp_data = self.waypoints.get(key)
        if wp_data:
            return wp_data['pose']
        return None

    def get_plant_waypoint_key(self, plant_name):
        """
        Returns "wp_X" key for a given plant string.
        Fuzzy matching could be added here, but exact match for now.
        """
        plant_name = plant_name.lower().strip()
        # Direct lookup
        if plant_name in self.plant_map:
            return self.plant_map[plant_name]
        
        # Partial match fallback
        for name, key in self.plant_map.items():
            if plant_name in name or name in plant_name:
                return key
                
        return None

    def get_sorted_waypoint_keys(self):
        """Returns list of keys ['wp_1', 'wp_2'...] sorted by ID"""
        keys = []
        for k in self.waypoints.keys():
            if k.startswith('wp_'):
                keys.append(k)
        
        # Sort by integer suffix
        keys.sort(key=lambda x: int(x.split('_')[1]))
        return keys

    def get_waypoint_info(self, key):
        """Returns the 'plant_info' string if available"""
        if key in self.waypoints:
            return self.waypoints[key].get('plant_info', "")
        return ""

if __name__ == "__main__":
    # Test (requires ROS env usually, but we can verify logic if imports pass)
    try:
        wm = WaypointManager()
        print(f"Plants: {wm.plant_map.keys()}")
    except:
        print("ROS not active, simple test skipped")
