#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

class DisplayManager:
    """
    Layer 4: Action Layer
    Handles Visual Feedback on Pepper's Tablet.
    """
    def __init__(self):
        rospy.loginfo("[DisplayManager] Initialized. Publishing to /pepper/tablet/display")
        self.display_pub = rospy.Subscriber('/pepper/tablet/display', String, self._mock_cb) # Just to mock
        
        # Real implementation would publish to a web view or specific tablet topic
        self.pub = rospy.Publisher('/pepper/tablet/command', String, queue_size=1)
        self.status_pub = rospy.Publisher('/pepper/ui/status', String, queue_size=1)
        self.options_pub = rospy.Publisher('/pepper/ui/options', String, queue_size=1)

    def update_status(self, status):
        """Update robot status on UI"""
        rospy.loginfo(f"📺 [DisplayManager] Status: {status}")
        self.status_pub.publish(status)
        
    def update_options(self, options):
        """
        Update available buttons on UI
        Args: options (list of str): e.g. ["Tour", "Help"]
        """
        import json
        json_str = json.dumps(options)
        self.options_pub.publish(json_str)

    def _mock_cb(self, msg):
        pass

    def show_text(self, text):
        """Show text/subtitle on screen"""
        rospy.loginfo(f"📺 [DisplayManager] Showing Text: '{text}'")
        self.pub.publish(f"SHOW_TEXT:{text}")

    def show_image(self, image_name):
        """Show specific plant image"""
        rospy.loginfo(f"📺 [DisplayManager] Showing Image: {image_name}.jpg")
        self.pub.publish(f"SHOW_IMAGE:{image_name}")

    def show_web(self, url):
        rospy.loginfo(f"📺 [DisplayManager] Loading URL: {url}")
        self.pub.publish(f"SHOW_URL:{url}")
        
    def clear(self):
        rospy.loginfo("📺 [DisplayManager] Clearing screen")
        self.pub.publish("CLEAR")
