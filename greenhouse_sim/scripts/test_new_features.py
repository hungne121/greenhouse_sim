#!/usr/bin/env python3
import sys
import unittest
from unittest.mock import MagicMock, patch

# --- MOCKING ROS ENVIRONMENT ---
# We mock these libraries so the test runs instantly without needing a real robot or roscore
mock_rospy = MagicMock()
mock_actionlib = MagicMock()
mock_control_msgs = MagicMock()
mock_trajectory_msgs = MagicMock()
mock_std_msgs = MagicMock()

sys.modules['rospy'] = mock_rospy
sys.modules['actionlib'] = mock_actionlib
sys.modules['control_msgs.msg'] = mock_control_msgs
sys.modules['trajectory_msgs.msg'] = mock_trajectory_msgs
sys.modules['std_msgs.msg'] = mock_std_msgs

# Add the script directory to path so we can import modules
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Now import the modules we just wrote
from modules.gesture_manager import GestureManager
from modules.display_manager import DisplayManager

class TestNewFeatures(unittest.TestCase):
    """
    Unit verify the logic of new managers
    """
    
    def setUp(self):
        print("\n----------------------------------------------------------------------")
    
    def test_01_display_manager(self):
        print(">> Testing DisplayManager logic...")
        dm = DisplayManager()
        
        # Test Show Text
        dm.show_text("Hello Pepper")
        # Verify it tried to publish the correct string protocol
        dm.pub.publish.assert_called_with("SHOW_TEXT:Hello Pepper")
        print("   [PASS] show_text() generates correct message.")
        
        # Test Show Image
        dm.show_image("rose")
        dm.pub.publish.assert_called_with("SHOW_IMAGE:rose")
        print("   [PASS] show_image() generates correct message.")

    def test_02_gesture_manager_connected(self):
        print(">> Testing GestureManager (Simulated Connection)...")
        
        # Mock successful connection
        with patch('actionlib.SimpleActionClient') as mock_client:
            instance = mock_client.return_value
            instance.wait_for_server.return_value = True
            
            gm = GestureManager()
            self.assertTrue(gm.enabled)
            
            # Test Wave
            gm.perform_gesture("wave_hand")
            self.assertTrue(instance.send_goal.called)
            print("   [PASS] wave_hand() sends trajectory goal.")
            
            # Test Nod
            gm.perform_gesture("nod_head")
            self.assertTrue(instance.send_goal.called)
            print("   [PASS] nod_head() sends trajectory goal.")

if __name__ == '__main__':
    unittest.main()
