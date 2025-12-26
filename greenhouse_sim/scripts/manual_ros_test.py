#!/usr/bin/env python3
import rospy
from modules.gesture_manager import GestureManager
from modules.display_manager import DisplayManager

def main():
    rospy.init_node('manual_test_multimodal')
    
    print("=== MANUAL TEST: GESTURE & DISPLAY ===")
    
    # 1. Init
    display = DisplayManager()
    gesture = GestureManager()
    
    # 2. Test Sequence
    rospy.loginfo("--- Test 1: Introduction ---")
    display.show_text("Khoi dong he thong...")
    gesture.perform_gesture("nod_head")
    rospy.sleep(2)
    
    rospy.loginfo("--- Test 2: Show Plant ---")
    display.show_image("rose_plant")
    gesture.perform_gesture("wave_hand") # Pointing/Waving
    rospy.sleep(2)
    
    rospy.loginfo("--- Test 3: Active Listening ---")
    display.show_text("Ban muon hoi gi?")
    gesture.perform_gesture("look_around")
    rospy.sleep(2)
    
    rospy.loginfo("=== TEST COMPLETE ===")

if __name__ == "__main__":
    main()
