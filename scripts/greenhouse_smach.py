#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Greenhouse SMACH Controller - Layer 3: The Brain
Industrial-grade State Machine implementation
"""

import rospy
import smach
import smach_ros
import threading
import time
import math
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from modules.audio_manager import AudioManager
from modules.navigation_manager import NavigationManager
from modules.perception_manager import PerceptionManager
from modules.waypoint_manager import WaypointManager
from modules.knowledge_manager import KnowledgeManager
from modules.gesture_manager import GestureManager
from modules.display_manager import DisplayManager

# ============================================
# STATE 1: IDLE (Chờ lệnh)
# ============================================

class IdleState(smach.State):
    """
    IDLE State - Listening for initial commands
    Outcomes: ['start_tour', 'go_to_plant', 'shutdown']
    """
    def __init__(self, audio, knowledge, gesture, display, nav, perception):
        smach.State.__init__(self, 
                             outcomes=['start_tour', 'go_to_plant', 'shutdown'],
                             output_keys=['target_plant', 'handoff_cmd'], 
                             input_keys=['handoff_cmd'])
        self.audio = audio
        self.knowledge = knowledge
        self.gesture = gesture
        self.display = display
        self.nav = nav
        self.perception = perception
    
    def execute(self, userdata):
        rospy.loginfo("[IDLE] Waiting for command (or Front Intent)...")
        self.gesture.perform_gesture('reset_pose')
        
        # UI Update
        self.display.update_status("Đang chờ lệnh... (Hãy nói 'Xin chào')")
        self.display.update_options(["Bắt đầu Tour", "Tìm cây", "Trợ giúp"])
        self.display.show_text("Robot Pepper: Sẵn sàng phục vụ")
        
        # Check Handoff
        start_text = None
        if hasattr(userdata, 'handoff_cmd') and userdata.handoff_cmd:
             start_text = userdata.handoff_cmd
             userdata.handoff_cmd = None
        
        # Loop
        greeting_triggered = False
        
        while not rospy.is_shutdown():
            # 1. VISUAL GREETING (New Logic: Intent Timer)
            if not greeting_triggered and not start_text:
                # Use new API: check_front_intent() -> Returns True if > 3s
                has_intent = self.perception.check_front_intent()
                
                if has_intent:
                    rospy.loginfo("[IDLE] Front Intent Confirmed (3s Dwell). Greeting.")
                    self.nav.send_velocity(0, 0) # Ensure stop
                    self.gesture.perform_gesture('wave_hand')
                    self.audio.speak("Xin chào! Tôi có thể giúp gì cho bạn?")
                    greeting_triggered = True
                
                # Note: No alignment here in IDLE, simplified as per request.
                # Just wait for Intent -> Greet.

            # 2. AUDIO PROCESSING
            if start_text:
                text = start_text
                start_text = None
            elif greeting_triggered:
                text = self.audio.listen_once(timeout=5)
            else:
                 rospy.loginfo_throttle(5.0, "[IDLE] Waiting for user (Front Camera)...")
                 rospy.sleep(0.5)
                 continue
            
            if not text:
                continue
            
            intent, conf = self.audio.predict_intent(text)
            rospy.loginfo(f"[IDLE] Detected: {intent} ({conf:.2f})")
            
            # Confidence Threshold
            if conf < 0.75:
                # Fallback: specific check for keywords even if model is unsure?
                # For now just reject
                rospy.logwarn(f"[IDLE] Low confidence ({conf:.2f}). Rejecting.")
                self.audio.speak("Xin lỗi, tôi nghe chưa rõ.")
                continue

            if intent == 'quit_system':
                self.audio.speak("Tạm biệt!")
                return 'shutdown'
            elif intent == 'whole_tour':
                self.audio.speak("Mời bạn đi theo tôi.")
                return 'start_tour'
            elif intent == 'navigation_request':
                # Extract plant logic (Simplified)
                plant_name = None
                for p in self.knowledge.get_all_plant_names():
                    if p in text: plant_name = p; break
                
                if plant_name:
                    userdata.target_plant = plant_name
                    self.audio.speak(f"Ok, đi đến {plant_name}.")
                    return 'go_to_plant'
            
            # (Other intents omitted for brevity, logic remains similar)
            self.audio.speak("Tôi chưa rõ. Bạn muốn đi tour hay tìm cây?")
        
        return 'shutdown'


# ============================================
# STATE 2: NAVIGATE (Generic Leading Logic)
# ============================================
# Used for both Single Goal and Tour

class BaseNavigateState(smach.State):
    """Base class for Human-Aware Navigation"""
    def __init__(self, nav, perception, audio, display, outcomes, input_keys, output_keys=[]):
        smach.State.__init__(self, outcomes, input_keys, output_keys)
        self.nav = nav
        self.perception = perception
        self.audio = audio
        self.display = display
        

    current_rot_vel = 0.0
    
    def _cmd_vel_monitoring(self, msg):
        try:
            self.current_rot_vel = msg.angular.z
        except:
            pass

    def perform_leading(self, pose):
        """
        Execute navigation with Rear Sensor Fusion Logic
        Distinguishes User-Lost Timeout for Straight vs Turning
        """
        # Subscribe to monitor Twist
        import geometry_msgs.msg
        sub = rospy.Subscriber('/cmd_vel', geometry_msgs.msg.Twist, self._cmd_vel_monitoring)
        
        self.nav.send_goal(pose, done_cb=self._done_cb)
        self.done = False
        self.nav_status = None
        self.is_paused = False
        
        rate = rospy.Rate(5)
        last_seen_time = rospy.Time.now()
        
        # Timeouts specified by User (Reversed: Turn needs more patience)
        TIMEOUT_STRAIGHT = 2.0
        TIMEOUT_TURN = 5.0
        
        while not self.done and not rospy.is_shutdown():
            # --- REAR SENSOR FUSION ---
            dist, angle, found = self.perception.get_rear_track()
            
            if found:
                last_seen_time = rospy.Time.now()
            
            # --- SPEED / PAUSE CONTROL ---
            if not found:
                # Determine Context: Turning or Straight?
                is_turning = abs(self.current_rot_vel) > 0.4 # Threshold rad/s
                timeout = TIMEOUT_TURN if is_turning else TIMEOUT_STRAIGHT
                
                # Hysteresis: Only stop if lost for > timeout
                if (rospy.Time.now() - last_seen_time).to_sec() > timeout:
                    # STOP condition
                    if not self.is_paused:
                        mode_str = "Turning" if is_turning else "Straight"
                        rospy.logwarn_throttle(2.0, f"[NAV] User Lost ({mode_str}). Timeout {timeout}s. Stopping.")
                        self.nav.cancel_goal()
                        self.is_paused = True
                        self.audio.speak("Bạn đâu rồi?")
            else:
                # FOUND
                if dist > 2.5:
                    if not self.is_paused:
                        rospy.logwarn(f"[NAV] User lagging ({dist:.1f}m). Waiting.")
                        self.nav.cancel_goal()
                        self.is_paused = True
                        self.audio.speak("Nhanh lên nhé.")
                elif dist < 1.0:
                    if self.is_paused:
                        rospy.loginfo(f"[NAV] User caught up ({dist:.1f}m). Resuming.")
                        self.nav.send_goal(pose, done_cb=self._done_cb)
                        self.is_paused = False
                else:
                    if self.is_paused:
                        rospy.loginfo(f"[NAV] User in range ({dist:.1f}m). Resuming.")
                        self.nav.send_goal(pose, done_cb=self._done_cb)
                        self.is_paused = False
            
            # Check Preempt (if Tour)
            if self.preempt_requested():
                self.service_preempt()
                self.nav.cancel_goal()
                sub.unregister()
                return 'preempted'
                
        sub.unregister()
        return self.nav_status

    def _done_cb(self, status, result):
        if not self.is_paused:
            self.done = True
            self.nav_status = status

    def perform_arrival_turn(self):
        """
        ARRIVAL LOGIC:
        1. Check Rear
        2. Blind Turn 180 + Angle
        3. Front Lock
        """
        rospy.loginfo("[ARRIVAL] Performing 180 turn...")
        
        # 1. Check Rear (Last known position)
        dist, angle, found = self.perception.get_rear_track()
        if found:
            rospy.loginfo(f"[ARRIVAL] Last known rear user: {angle:.2f} rad")
        else:
            rospy.loginfo("[ARRIVAL] User not visible in rear. Doing blind 180.")
        
        # Calculate turn amount
        # Robot is facing Away. User is at 'angle' (relative to Rear).
        # We want Front to face User.
        # Turn = 180 (+/- adjustment)
        target_turn = 3.14159 
        if found:
            # Angle is from Rear Camera logic. 
            # If User is Left of Rear Cam center, they are Left of Robot Back?
            # Let's assume standard frame: +Y is Left.
            target_turn += angle # Simple compensation
            
        # 2. Executive Blind Turn
        # Use velocity command manually
        rospy.loginfo(f"[ARRIVAL] Turning {target_turn:.2f} rad")
        self.nav.send_velocity(0, 1.0) # Turn Left 1 rad/s
        duration = abs(target_turn / 1.0)
        rospy.sleep(duration)
        self.nav.send_velocity(0, 0)
        rospy.sleep(0.5)
        
        # 3. Front Lock (Switch to Front Sensor)
        rospy.loginfo("[ARRIVAL] Locking on Front...")
        found_front = self.perception.get_front_blob_status() # Boolean status
        if found_front:
             rospy.loginfo("[ARRIVAL] Front Lock Confirmed. User visible.")
        
        # Fine tune if possible (Simulation HOG is tricky for precise angle)
        # Just ensure we face roughly
        pass

class NavigateState(BaseNavigateState):
    """
    Subclass for specific navigation tasks (e.g. Go to Plant)
    """
    def __init__(self, nav, wp_manager, audio, knowledge, perception, display):
        BaseNavigateState.__init__(self, nav, perception, audio, display, 
                                   outcomes=['arrived', 'aborted'], 
                                   input_keys=['target_plant'])
        self.wp_manager = wp_manager
        self.knowledge = knowledge
        
    def execute(self, userdata):
        plant_name = userdata.target_plant
        rospy.loginfo(f"[NAV] Going to plant: {plant_name}")
        
        # UI Update
        self.display.update_status(f"Đang đi đến {plant_name}...")
        self.display.update_options(["Dừng lại", "Hủy bỏ"])
        
        # 1. Get Coordinates
        wp_key = self.wp_manager.get_plant_waypoint_key(plant_name)
        if not wp_key: return 'aborted'
        
        pose = self.wp_manager.get_waypoint_pose(wp_key)
        
        # 1. Lead
        status = self.perform_leading(pose)
        
        if status == GoalStatus.SUCCEEDED:
            # Announce
            # self.audio.speak(f"Chúng ta đã đến {plant}.") # Moved to ARRIVAL/TurnFaceUser
            return 'arrived'
        else:
            return 'aborted'

class TourNavigate(BaseNavigateState):
    """Tour Navigation Step"""
    def __init__(self, nav, wp_manager, audio, perception, gesture, display):
        BaseNavigateState.__init__(self, nav, perception, audio, display,
                                   outcomes=['arrived', 'tour_interrupted', 'tour_complete'],
                                   input_keys=['tour_index', 'tour_keys'],
                                   output_keys=['tour_index', 'current_plant'])
        self.wp_manager = wp_manager
        self.display = display
        self.gesture = gesture
        
    def execute(self, userdata):
        if userdata.tour_index >= len(userdata.tour_keys):
            return 'tour_complete'
            
        key = userdata.tour_keys[userdata.tour_index]
        plant = self.wp_manager.waypoints[key].get('plant_type', '')
        pose = self.wp_manager.get_waypoint_pose(key)
        userdata.current_plant = plant
        
        self.display.show_text(f"Đi đến {plant}...")
        self.gesture.perform_gesture("walk_pose")
        
        # 1. Lead
        status = self.perform_leading(pose)
        
        if status == 'preempted' or status == GoalStatus.PREEMPTED:
            return 'tour_interrupted'
        elif status == GoalStatus.SUCCEEDED:
            return 'arrived'
        else:
            return 'tour_interrupted'

# (Keep TourIntro and TourWaitUser mostly same, just ensure they don't break)
class TourIntro(smach.State):
    """Give detailed introduction at each plant"""
    def __init__(self, audio, knowledge, gesture, display, nav, perception):
        smach.State.__init__(self,
                             outcomes=['intro_done'],
                             input_keys=['current_plant', 'tour_index'],
                             output_keys=['tour_index'])
        self.audio = audio
        self.knowledge = knowledge
        self.gesture = gesture
        self.display = display
        self.nav = nav
        self.perception = perception
    
    def execute(self, userdata):
        plant = userdata.current_plant
        data = self.knowledge.plants_data.get(plant, {})
        
        # Visual & Gesture
        # helper to get image
        img_name = f"{plant}.jpg" # Simplified resolution
        # self.display.show_image(img_name) 
        self.gesture.perform_gesture('wave_hand') # Point to plant
        
        # Concise Vietnamese introduction
        intro = f"Chúng ta đã đến khu vực {plant}. "
        intro += f"Đây là giống {data.get('variety', 'chưa rõ')}. "
        # Use concise description instead of long characteristics
        intro += f"{data.get('description', '')}"
        
        # Face user logic
        # 1. Align
        dist, angle = self.perception.get_front_human_input()
        if dist < 3.0:
            if abs(angle) > 0.2:
                cmd_turn = 0.5 if angle > 0 else -0.5
                self.nav.send_velocity(0, cmd_turn)
                rospy.sleep(1.0)
                self.nav.send_velocity(0,0)
        
        self.audio.speak(intro, blocking=True)
        
        # Increment tour index for next iteration
        userdata.tour_index += 1
        
        return 'intro_done'

class TourWaitUser(smach.State):
    """Wait for user decision: continue tour or ask questions"""
    def __init__(self, audio, knowledge, gesture, display):
        smach.State.__init__(self,
                             outcomes=['continue_tour', 'answer_question', 'end_tour'],
                             input_keys=['current_plant'],
                             output_keys=['handoff_cmd']) 
        self.audio = audio
        self.knowledge = knowledge
        self.gesture = gesture
        self.display = display
    
    def execute(self, userdata):
        # Reset handoff
        userdata.handoff_cmd = None
        
        self.display.show_text("Bạn muốn đi tiếp hay tìm hiểu thêm?")
        self.gesture.perform_gesture('nod_head')
        self.audio.speak("Quý khách muốn tìm hiểu thêm về cây này hay tiếp tục tour?", blocking=True)
        self.gesture.perform_gesture('look_around') # Active listening
        
        # Listening Icon/Status
        self.display.show_text("Đang lằng nghe... (Bạn muốn đi tiếp?)")
        
        # Listen for response (Simplified)
        text = self.audio.listen_once(timeout=5)
        
        if not text:
            # Silence -> Implicit "Continue"
            rospy.loginfo("[TOUR_WAIT_USER] Timeout/Silence. Defaulting to Continue.")
            return 'continue_tour'
        
        intent, conf = self.audio.predict_intent(text)
        
        if conf < 0.65:
            # Low confidence -> Ignore/Continue (Don't annoy user)
            rospy.loginfo(f"[TOUR_WAIT_USER] Low confidence ({conf}). Defaulting to Continue.")
            return 'continue_tour'
            
        # Universal Intent Handling
            # 1. Cancel/Stop (Check FIRST for negative keywords)
        if any(w in text for w in ['dừng', 'thôi', 'không muốn', 'không', 'kết thúc', 'stop']):
             rospy.loginfo(f"[TOUR_WAIT_USER] User cancelled: {text}")
             userdata.handoff_cmd = None 
             return 'end_tour'

            # 2. Continue Tour
        elif intent == 'continue_tour' or 'tiếp tục' in text or 'đi tiếp' in text:
            return 'continue_tour'
        
        # 3. Ask Question (Stay in Loop)
        elif intent == 'list_plants':
            plant_names = ', '.join(self.knowledge.get_all_plant_names())
            self.audio.speak(f"Nhà kính có: {plant_names}.")
            return 'answer_question'
            
        elif intent in ['ask_variety', 'ask_technique', 'ask_care', 'plant_story']:
            self._answer_question(userdata.current_plant, intent)
            return 'answer_question'

        # 3. Switch Task (End Tour + Handoff) - SPECIFIC PLANT
        elif intent == 'navigation_request':
            rospy.loginfo(f"[TOUR_WAIT_USER] User requested navigation. Ending tour to handle.")
            userdata.handoff_cmd = text # Pass text to IDLE
            self.audio.speak("Vâng, tôi sẽ thực hiện ngay.")
            return 'end_tour'
        
        # 4. Unknown but valid command -> Handoff to IDLE (Universal Listening)
        elif intent == 'greeting':
             userdata.handoff_cmd = text
             return 'end_tour'
        
        # Fallback for truly unknown
        else:
             self.audio.speak("Xin lỗi, tôi chưa hiểu rõ. Bạn muốn tiếp tục hay hỏi gì khác?")
             return 'continue_tour'
             
        return 'continue_tour'
    
    def _answer_question(self, plant, intent):
        """Helper to answer agriculture questions"""
        data = self.knowledge.plants_data.get(plant, {})
        
        if intent == 'plant_story':
            story = data.get('folklore', 'Chưa có thông tin.')
            self.audio.speak(f"Chuyện kể rằng: {story}")
        
        elif intent == 'ask_variety':
            self.audio.speak(f"Giống: {data.get('variety', '')}. {data.get('origin', '')}")
        
        elif intent == 'ask_technique':
            self.audio.speak(f"Kỹ thuật: {data.get('planting_tech', '')}. Đất: {data.get('soil', '')}")
        
        elif intent == 'ask_care':
            self.audio.speak(f"Chăm sóc: {data.get('care_water', '')}. {data.get('fertilizer', '')}")

# ============================================
class HandlingObstacle(smach.State):
    """
    Handle obstacle situation (human too close)
    """
    def __init__(self, audio, perception, nav, display):
        smach.State.__init__(self, outcomes=['clear', 'still_blocked', 'cancelled'])
        self.audio = audio
        self.perception = perception
        self.nav = nav
        self.display = display
        
    def execute(self, userdata):
        # UI Update
        self.display.update_status("Gặp vật cản! Vui lòng tránh đường.")
        self.display.update_options(["Tôi đã tránh", "Hủy bỏ"])

class TurnFaceUser(smach.State):
    """
    State: Smart Arrival (Face User)
    Turns 180 (interruptible) -> Aligns with Front Camera
    """
    def __init__(self, nav, audio, perception, display):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.nav = nav
        self.audio = audio
        self.perception = perception
        self.display = display
        
    def execute(self, userdata):
        rospy.loginfo("[FACE_USER] Smart Arrival: Turning to face user...")
        
        # UI Update
        self.display.update_status("Đã đến nơi (Đang tìm bạn...)")
        self.display.update_options(["Chi tiết", "Đi tiếp"])
        
        self.audio.speak("Đến nơi rồi.")
        
        # 1. Smart Turn 180 (Check Front Camera)
        def check_front_fn():
            angle, found = self.perception.get_front_track()
            return found
            
        # Check if already found
        angle, found = self.perception.get_front_track()
        if found:
            rospy.loginfo("[FACE_USER] User already in front!")
            success = 'interrupted'
        else:
            success = self.nav.rotate_relative_with_interrupt(3.14159, speed=1.0, check_fn=check_front_fn)
            
        # 2. Scanning if not found
        angle, found = self.perception.get_front_track()
        if not found:
             rospy.loginfo("[FACE_USER] User not found. Scanning...")
             scan_angles = [0.7, -1.4] # Left 40, Right 80
             for scan in scan_angles:
                 self.nav.rotate_relative_with_interrupt(scan, speed=0.8, check_fn=check_front_fn)
                 if check_front_fn():
                     found = True
                     break
        
        # 3. Active Alignment (Front)
        if found:
            rospy.loginfo("[FACE_USER] Aligning with Front Camera...")
            align_timeout = rospy.Time.now() + rospy.Duration(5.0)
            rate = rospy.Rate(10)
            
            while rospy.Time.now() < align_timeout:
                angle, found = self.perception.get_front_track()
                if found:
                    err = angle # Positive (Left) -> Turn Left (+)
                    if abs(err) < 0.1:
                        rospy.loginfo("[FACE_USER] Aligned!")
                        break
                    
                    rot_vel = 1.5 * err
                    rot_vel = max(min(rot_vel, 0.8), -0.8) # Clamp
                    self.nav.send_velocity(0, rot_vel)
                else:
                    self.nav.send_velocity(0, 0)
                    
                rate.sleep()
                
            self.nav.send_velocity(0, 0)
            self.audio.speak("Mời bạn xem.")
            return 'succeeded'
        else:
            rospy.logwarn("[FACE_USER] Could not find user after turn.")
            self.audio.speak("Bạn đâu rồi?")
            return 'succeeded' # Proceed anyway


# ============================================
# STATE 4: ARRIVAL (Announce arrival)
# ============================================

class ArrivalState(smach.State):
    """State to announce arrival at plant"""
    def __init__(self, audio, knowledge):
        smach.State.__init__(self, 
                             outcomes=['done', 'return_idle'],
                             input_keys=['target_plant'])
        self.audio = audio
        self.knowledge = knowledge
    
    def execute(self, userdata):
        plant = userdata.target_plant
        data = self.knowledge.plants_data.get(plant, {})
        
        intro = f"Chúng ta đã đến {plant}. "
        intro += f"Đây là giống {data.get('variety', 'chưa rõ')}. "
        intro += f"Xuất xứ: {data.get('origin', 'chưa rõ')}. "
        intro += f"Đặc điểm: {data.get('characteristics', '')}."
        
        self.audio.speak(intro)
        
        return 'done'


# ============================================
# STATE NEW: PREPARE LEADING (Rotate & Verify)
# ============================================

class PrepareLeading(smach.State):
    """
    Transition State:
    1. Acknowledge Command
    2. Rotate 180 (Blind Turn)
    3. Verify Rear Camera
    4. Start Tour
    """
    def __init__(self, nav, audio, perception):
        smach.State.__init__(self, outcomes=['ready', 'failed'])
        self.nav = nav
        self.audio = audio
        self.perception = perception
        
    def execute(self, userdata):
        rospy.loginfo("[PREPARE] Preparing to lead...")
        
        # 1. Acknowledge
        self.audio.speak("Được rồi, đi theo tôi nhé.")
        rospy.sleep(1.0)
        
        # 2. Rough Turn 180 (Blind)
        # 2. Rough Turn 180 (Smart Interrupt)
        rospy.loginfo("[PREPARE] Rough Turn 180 (Smart)...")
        
        def check_rear_fn():
             d, a, f = self.perception.get_rear_track()
             return f
        
        # Check before turning (maybe already facing?)
        dist, angle, found = self.perception.get_rear_track()
        if found:
             rospy.loginfo("[PREPARE] User already found!")
             success = 'interrupted'
        else:
             success = self.nav.rotate_relative_with_interrupt(3.14159, speed=1.0, check_fn=check_rear_fn)
        
        aligned = False
        
        if success == 'interrupted' or success == 'done': # 'done' means finished turn, check if found now
             # Check if we should Scan or Align
             dist, angle, found = self.perception.get_rear_track()
             
             if not found:
                 # SEARCH BEHAVIOR (Scan +/- 45 deg)
                 rospy.loginfo("[PREPARE] User not found after turn. Scanning...")
                 scan_angles = [0.7, -1.4] # Left 40deg, then Right 80deg (net -40)
                 
                 for scan in scan_angles:
                     rospy.loginfo(f"[PREPARE] Scanning {scan} rad...")
                     self.nav.rotate_relative_with_interrupt(scan, speed=0.8, check_fn=check_rear_fn)
                     if check_rear_fn():
                         found = True
                         break
                         
        # 3. Active Alignment (Visual Servoing)
        if found:
            rospy.loginfo("[PREPARE] Aligning with Rear Camera...")
            align_timeout = rospy.Time.now() + rospy.Duration(5.0)
            
            rate = rospy.Rate(10)
            while rospy.Time.now() < align_timeout:
                dist, angle, found = self.perception.get_rear_track()
                
                if found:
                    err = angle
                    if abs(err) < 0.1:
                        rospy.loginfo(f"[PREPARE] Aligned! Error: {err:.2f}")
                        aligned = True
                        break
                    else:
                        kp = 1.5
                        rot_vel = kp * err
                        rot_vel = max(min(rot_vel, 1.0), -1.0)
                        self.nav.send_velocity(0, rot_vel)
                else:
                    self.nav.send_velocity(0, 0) # Lost during align?
                
                rate.sleep()
                
            self.nav.send_velocity(0, 0)
        
        if aligned or found: # Allow if found but maybe not 100% aligned
            self.audio.speak("Đã thấy bạn. Đi thôi!")
            return 'ready'
        else:
            self.audio.speak("Bạn đâu rồi?")
            # Fallback: Just go anyway? OR Return failed? 
            # User wants it smooth, likely user IS there.
            return 'ready' # Assume user will catch up


def create_tour_sm(nav, audio, knowledge, wp_manager, perception, gesture, display):
    """Create nested TOUR state machine"""
    
    # Added output_keys to expose handoff_cmd
    tour_sm = smach.StateMachine(outcomes=['tour_finished', 'tour_cancelled'],
                                 output_keys=['handoff_cmd'])
    
    tour_sm.userdata.tour_index = 0
    tour_sm.userdata.tour_keys = wp_manager.get_sorted_waypoint_keys()
    tour_sm.userdata.current_plant = None
    tour_sm.userdata.handoff_cmd = None # Init
    
    with tour_sm:
        # NEW INITIAL STATE
        smach.StateMachine.add(
            'PREPARE_LEADING',
            PrepareLeading(nav, audio, perception),
            transitions={
                'ready': 'TOUR_NAVIGATE',
                'failed': 'tour_cancelled'
            }
        )

        smach.StateMachine.add(
            'TOUR_NAVIGATE',
            TourNavigate(nav, wp_manager, audio, perception, gesture, display),
            transitions={
                'arrived': 'TURN_FACE_USER',
                'tour_interrupted': 'tour_cancelled',
                'tour_complete': 'tour_finished'
            }
        )
        
        smach.StateMachine.add(
            'TURN_FACE_USER',
            TurnFaceUser(nav, audio, perception, display),
            transitions={
                'succeeded': 'TOUR_INTRO',
                'failed': 'TOUR_INTRO'
            }
        )
        
        smach.StateMachine.add(
            'TOUR_INTRO',
            TourIntro(audio, knowledge, gesture, display, nav, perception),
            transitions={'intro_done': 'TOUR_WAIT_USER'}
        )
        
        smach.StateMachine.add(
            'TOUR_WAIT_USER',
            TourWaitUser(audio, knowledge, gesture, display),
            transitions={
                'continue_tour': 'TOUR_NAVIGATE',
                'answer_question': 'TOUR_WAIT_USER',  # Loop back after answering
                'end_tour': 'tour_finished'
            }
        )
    
    return tour_sm

# ============================================
# MAIN STATE MACHINE
# ============================================

def create_greenhouse_sm():
    """Create the main Greenhouse State Machine"""
    
    # Initialize managers
    audio = AudioManager()
    nav = NavigationManager()
    perception = PerceptionManager()
    wp_manager = WaypointManager()
    knowledge = KnowledgeManager()
    gesture = GestureManager()
    display = DisplayManager()
    
    # NOTE: Background listening DISABLED
    # States handle their own listening to avoid echo
    # audio.start_listening_background()  # DISABLED
    
    # Create top-level state machine
    sm = smach.StateMachine(outcomes=['shutdown'])
    sm.userdata.target_plant = None
    sm.userdata.handoff_cmd = None # Init handoff variable
    
    with sm:
        # Add IDLE state
        smach.StateMachine.add(
            'IDLE',
            IdleState(audio, knowledge, gesture, display, nav, perception),
            transitions={
                'start_tour': 'TOUR',
                'go_to_plant': 'PREPARE_SINGLE', 
                'shutdown': 'shutdown'
            }
        )
            
        # Add TOUR nested state machine
        tour_sm = create_tour_sm(nav, audio, knowledge, wp_manager, perception, gesture, display)
        smach.StateMachine.add(
            'TOUR',
            tour_sm,
            transitions={
                'tour_finished': 'IDLE',
                'tour_cancelled': 'IDLE'
            },
            remapping={'handoff_cmd': 'handoff_cmd'} # Remap output to global userdata
        )

        # Add PREPARE_SINGLE for Single Goal Navigation
        smach.StateMachine.add(
            'PREPARE_SINGLE',
            PrepareLeading(nav, audio, perception),
            transitions={
                'ready': 'EXECUTE',
                'failed': 'IDLE'
            }
        )
        
        # Add EXECUTE - Simple navigation (no concurrency)
        smach.StateMachine.add(
            'EXECUTE',
            NavigateState(nav, wp_manager, audio, knowledge, perception, display),  # Pass perception
            transitions={
                'arrived': 'TURN_FACE_USER',
                'aborted': 'IDLE'
            }
        )
        
        # Add HANDLING_OBSTACLE state
        smach.StateMachine.add(
            'HANDLING_OBSTACLE',
            HandlingObstacle(audio, perception, nav, display),
            transitions={
                'clear': 'EXECUTE',
                'still_blocked': 'HANDLING_OBSTACLE',
                'cancelled': 'IDLE'
            }
        )
        
        # Add TURN_FACE_USER
        smach.StateMachine.add(
            'TURN_FACE_USER',
            TurnFaceUser(nav, audio, perception, display),
            transitions={
                'succeeded': 'IDLE', # Or PREPARE_INTERACT
                'failed': 'IDLE'
            }
        )
        
        # Add ARRIVAL state
        smach.StateMachine.add(
            'ARRIVAL',
            ArrivalState(audio, knowledge),
            transitions={
                'done': 'IDLE',
                'return_idle': 'IDLE'
            }
        )
    
    return sm, audio

def main():
    rospy.init_node('greenhouse_smach_controller')
    
    # Create state machine
    sm, audio = create_greenhouse_sm()
    
    # Create and start introspection server (for smach_viewer)
    sis = smach_ros.IntrospectionServer('greenhouse_smach', sm, '/SM_ROOT')
    sis.start()
    
    rospy.loginfo("=== GREENHOUSE SMACH CONTROLLER STARTED ===")
    rospy.loginfo("To visualize: rosrun smach_viewer smach_viewer.py")
    
    # Execute state machine
    outcome = sm.execute()
    
    # Cleanup
    audio.stop_listening_background()
    sis.stop()
    
    rospy.loginfo(f"State machine exited with outcome: {outcome}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
