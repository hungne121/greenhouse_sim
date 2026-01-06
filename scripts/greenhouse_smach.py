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
    IDLE State - Passive Waiting
    Outcomes: ['detected']
    """
    def __init__(self, audio, gesture, display, perception):
        smach.State.__init__(self, outcomes=['detected'])
        self.audio = audio
        self.gesture = gesture
        self.display = display
        self.perception = perception
    
    def execute(self, userdata):
        rospy.loginfo("[IDLE] Passive Mode. Waiting for Wake Word or Visual Intent...")
        self.gesture.perform_gesture('reset_pose')
        
        # UI Update
        self.display.update_status("Đang chờ lệnh... (Hãy nói 'Pepper ơi')")
        self.display.update_options(["Gọi 'Pepper ơi'"])
        self.display.show_text("Zzz...")
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # 1. Visual Intent (> 3s)
            if self.perception.check_front_intent():
                rospy.loginfo("[IDLE] Front Intent Confirmed. Waking up.")
                return 'detected'
            
            # 2. Audio Wake Word (Simulated checks or blocking listen not ideal here, 
            # but if we use hotword detection it would be here. 
            # For now, we reuse listen_once with short timeout to catch 'Pepper' or large noise)
            # Actually, to be truly passive, we might just rely on Visual OR a trigger.
            # User request said: "Idle -> Greeting -> Communication".
            # Let's keep it simple: Visual Intent OR simple sound trigger.
            
            # For simulation responsiveness, we'll check if any valid voice command came in background?
            # Or just rely on visual for now as per "Intent" focus.
            # But let's add a quick listen check to prompt.
            
            # Simplified: Just wait for Visual Intent for now as primary wake up
            # Or if tablet button pressed (via internal mechanism not shown here but implies intent)
            
            rate.sleep()
        
        return 'detected'

class GreetingState(smach.State):
    """
    Greeting State - Wave and Say Hello
    Outcomes: ['succeeded']
    """
    def __init__(self, audio, gesture, display):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['last_interaction'],
                             output_keys=['last_interaction'])
        self.audio = audio
        self.gesture = gesture
        self.display = display
        
    def execute(self, userdata):
        # Check Debounce
        now = rospy.Time.now().to_sec()
        last = userdata.last_interaction if userdata.last_interaction else 0
        
        # FIX: Always greet if last == 0 (First run)
        if last != 0 and (now - last) < 60.0: # 60s Debounce
            rospy.loginfo("[GREETING] Recent interaction. Skipping greeting.")
            return 'succeeded'
            
        rospy.loginfo("[GREETING] Performing greeting...")
        self.display.update_status("Xin chào!")
        self.display.show_text("Chào bạn! Tôi là Pepper.")
        
        # Parallel: Wave + Speak
        self.gesture.perform_gesture('wave_hand')
        self.audio.speak("Xin chào! Tôi có thể giúp gì cho bạn?")
        
        # Update timestamp only on full greeting? Or keep it updated in Communication?
        # Let's update here to mark start of session
        userdata.last_interaction = now
        
        return 'succeeded'

class CommunicationState(smach.State):
    """
    Communication State - Central Hub
    Outcomes: ['start_tour', 'go_to_plant', 'idle']
    """
    def __init__(self, audio, knowledge, gesture, display):
        smach.State.__init__(self, 
                             outcomes=['start_tour', 'go_to_plant', 'idle'],
                             output_keys=['target_plant', 'handoff_cmd', 'last_interaction'],
                             input_keys=['handoff_cmd', 'current_location', 'last_interaction']) # Context
        self.audio = audio
        self.knowledge = knowledge
        self.gesture = gesture
        self.display = display
        
    def execute(self, userdata):
        rospy.loginfo("[COMM] Listening for commands...")
        self.gesture.perform_gesture('reset_pose')
        
        # Check Context
        context_plant = None
        if hasattr(userdata, 'current_location') and userdata.current_location:
            context_plant = userdata.current_location
            self.display.update_status(f"Đang ở {context_plant}. Bạn cần gì?")
        else:
            self.display.update_status("Đang lắng nghe...")
            
        self.display.update_options(["Đi Tour", "Tìm cây", "Kết thúc"])
        
        # Check Handoff
        text = None
        if hasattr(userdata, 'handoff_cmd') and userdata.handoff_cmd:
             text = userdata.handoff_cmd
             userdata.handoff_cmd = None
             rospy.loginfo(f"[COMM] Processing handoff: {text}")
        
        # --- LISTENING LOOP (With Soft Timeout) ---
        soft_timeout_triggered = False
        
        if not text:
            # First Try
            text = self.audio.listen_once(timeout=8)
            
            # If silence, try prompting ONCE (Soft Timeout)
            if not text:
                if context_plant:
                     self.audio.speak(f"Bạn có muốn biết thêm gì về {context_plant} không?")
                else:
                     self.audio.speak("Bạn cần tôi giúp gì nữa không?")
                     
                soft_timeout_triggered = True
                rospy.sleep(0.5)
                text = self.audio.listen_once(timeout=6) # Listen again
        
        if not text:
            rospy.loginfo("[COMM] Still silence. Returning to IDLE.")
            return 'idle'
            
        intent, conf = self.audio.predict_intent(text)
        rospy.loginfo(f"[COMM] Intent: {intent} ({conf:.2f})")
        
        # Update Timestamp
        userdata.last_interaction = rospy.Time.now().to_sec()
        
        if conf < 0.7: 
            self.audio.speak("Tôi nghe không rõ. Mời bạn nói lại.")
            return 'idle' # Ideally Loop here? But linear flow returns IDLE. IDLE will re-greet if > 60s.

        if intent == 'whole_tour':
            self.audio.speak("Tuyệt vời. Mời bạn đi theo tôi.")
            return 'start_tour'
        elif intent == 'navigation_request':
            for p in self.knowledge.get_all_plant_names():
                if p in text.lower():
                    userdata.target_plant = p
                    self.audio.speak(f"Được, đi đến {p}.")
                    return 'go_to_plant'
            self.audio.speak("Tôi chưa biết cây đó.")
            return 'idle'
        elif intent == 'quit_system' or 'stop' in text.lower():
            self.audio.speak("Tạm biệt.")
            return 'idle'
        elif intent.startswith('ask_'):
            # Contextual Q&A
            target = None
            # 1. Check if entity mentioned
            for p in self.knowledge.get_all_plant_names():
                if p in text.lower():
                    target = p; break
            
            # 2. If not, use Context
            if not target and context_plant:
                rospy.loginfo(f"[COMM] Using Context: {context_plant}")
                target = context_plant
            
            if target:
                # Answer logic
                # For simplicity here, calling knowledge manager directly or placeholder
                # We don't have a direct 'answer' method in KnowledgeManager shown here?
                # Usually we return 'answer_question' state or handle it.
                # Let's perform answer HERE for fluidity
                self.audio.speak(f"Về {target} thì...") # Placeholder phrasing
                # Actually knowledge manager has query?
                # Let's assume audio.speak handles it or we call _answer_helper
                
                # Retrieve data manually to speak
                k_data = self.knowledge.plants_data.get(target, {})
                if 'care' in intent: 
                    ans = f"Chăm sóc {target}: {k_data.get('care_water', '')} {k_data.get('care_light', '')} {k_data.get('fertilizer', '')}"
                elif 'variety' in intent: ans = k_data.get('variety', 'Giống này tôi chưa rõ.')
                elif 'origin' in intent: ans = k_data.get('origin', 'Xuất xứ chưa rõ.')
                else: ans = k_data.get('description', '')
                
                self.audio.speak(ans)
                self.audio.speak("Bạn muốn hỏi gì nữa không?")
                # Return 'idle' to loop back via IDLE->GREETING check (debounce) -> COMM
                # This works because greeting checks time < 60s.
                return 'idle'
            else:
                self.audio.speak("Bạn muốn hỏi về cây nào?")
                return 'idle'
            
        return 'idle'


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
            
            # --- NEW: Safety Stop Check (< 0.8m) ---
            if found and dist < 0.8:
                rospy.logwarn_throttle(2.0, f"[NAV] Safety Stop! User too close ({dist:.2f}m)")
                self.nav.cancel_goal() # Stop navigation
                self.nav.send_velocity(0, 0) # Ensure robot is stopped
                self.display.update_status("Bạn đi quá gần!")
                self.audio.speak("Bạn đi quá gần, vui lòng lùi lại.", blocking=False) # Add Audio Warning
                self.is_paused = True # Mark as paused due to safety
                last_seen_time = rospy.Time.now() # Reset last seen to avoid immediate lost user timeout
                rate.sleep()
                continue # Skip normal leading logic until user moves away
            # --- END NEW ---

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
                        self.audio.speak("Tôi đang tìm bạn...")
            else:
                # FOUND
                if dist > 2.5:
                    if not self.is_paused:
                        rospy.logwarn(f"[NAV] User lagging ({dist:.1f}m). Waiting.")
                        self.nav.cancel_goal()
                        self.is_paused = True
                        self.audio.speak("Nhanh lên nhé.")
                elif dist < 1.0: 
                    # User < 1.0m. 
                    # If Paused (Safety Stop or Too Far), DO NOT RESUME yet.
                    # User must step back > 1.0m to resume.
                    pass
                else: # User is in optimal range (1.0m - 2.5m)
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
                
            rate.sleep() # Ensure loop runs at desired rate
                
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
        dist, angle, found = self.perception.get_front_track() # Use get_front_track for consistency
        if found and dist < 3.0:
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
        # 1. Smart Turn 180 (Check Front Camera)
        def check_front_fn():
            dist, angle, found = self.perception.get_front_track()
            return found
            
        # Check if already found
        dist, angle, found = self.perception.get_front_track()
        if found:
            rospy.loginfo("[FACE_USER] User already in front!")
            success = 'interrupted'
        else:
            success = self.nav.rotate_relative_with_interrupt(3.14159, speed=1.0, check_fn=check_front_fn)
            
        # 2. Scanning if not found
        dist, angle, found = self.perception.get_front_track()
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
            align_timeout = rospy.Time.now() + rospy.Duration(10.0) # More time
            stable_start = None
            rate = rospy.Rate(10)
            
            while rospy.Time.now() < align_timeout:
                dist, angle, found = self.perception.get_front_track()
                
                if found:
                    err = angle # Positive (Left) -> Turn Left (+)
                    
                    # Log error for debug
                    # rospy.loginfo_throttle(0.5, f"[FACE_USER] Err: {err:.2f}")
                    
                    if abs(err) < 0.06: # Tighter tolerance (~3.5 deg)
                        if stable_start is None:
                            stable_start = rospy.Time.now()
                        
                        # Require stability for 1s
                        if (rospy.Time.now() - stable_start).to_sec() > 1.0:
                             rospy.loginfo("[FACE_USER] Aligned and Stable!")
                             break
                    else:
                        stable_start = None # Reset if drifts
                        
                        # PID Control (Aggressive)
                        kp = 1.2 
                        rot_vel = kp * err
                        
                        # Min Moving Speed (Overcome Friction)
                        if abs(rot_vel) < 0.15: 
                            rot_vel = 0.15 * (1.0 if err > 0 else -1.0)
                        
                        rot_vel = max(min(rot_vel, 0.6), -0.6) 
                        self.nav.send_velocity(0, rot_vel)
                else:
                    self.nav.send_velocity(0, 0)
                    stable_start = None
                    
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
    def __init__(self, audio, gesture, knowledge, display):
        smach.State.__init__(self, 
                             outcomes=['done'],
                             input_keys=['target_plant'],
                             output_keys=['current_location']) # Added output
        self.audio = audio
        self.gesture = gesture
        self.knowledge = knowledge
        self.display = display
    
    def execute(self, userdata):
        plant = userdata.target_plant
        data = self.knowledge.plants_data.get(plant, {})
        
        # Update Context
        userdata.current_location = plant 
        rospy.loginfo(f"[ARRIVAL] Context updated: current_location = {plant}")
        
        # Display & Gesture
        self.display.show_image(data.get('image', 'placeholder.jpg'))
        self.gesture.perform_gesture('show_plant')
        
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
             
             if found:
                 # FIX: Wait a bit for user to react/move into frame?
                 rospy.sleep(1.5) # Give user time to catch up after robot spin
                 
                 # Align Logic (Simplified for Prepare)
                 # For PrepareLeading, we just need to ensure user is generally in view.
                 # Full alignment can be done by the leading state.
                 # If found, we consider it 'aligned enough' for starting.
                 aligned = True # Mark as aligned if found after turn/interrupt
                 
                 if aligned or found: 
                     self.audio.speak("Đã thấy bạn. Đi thôi!")
                     return 'ready'
                 else:
                     # Fallback: Search Scan?
                     # For now proceed if found once
                     return 'ready'
        else:
             # Search Scan
             rospy.loginfo("[PREPARE] User not found. Scanning...")
             self.audio.speak("Tôi đang tìm bạn...")
             scan_angles = [0.7, -1.4] 
             found_user = False
             
             for scan in scan_angles:
                 self.nav.rotate_relative_with_interrupt(scan, speed=0.8, check_fn=lambda: self.perception.get_rear_track()[2])
                 rospy.sleep(0.5)
                 if self.perception.get_rear_track()[2]:
                     found_user = True
                     break
             
             if found_user:
                 self.audio.speak("À, thấy bạn rồi. Đi thôi.")
                 return 'ready'
             
             self.audio.speak("Bạn đâu rồi?")
             # Fallback
             return 'ready'


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
    audio_manager = AudioManager()
    nav_manager = NavigationManager()
    perception_manager = PerceptionManager()
    wp_manager = WaypointManager()
    knowledge_manager = KnowledgeManager()
    gesture_manager = GestureManager()
    display_manager = DisplayManager()
    
    # NOTE: Background listening DISABLED
    # States handle their own listening to avoid echo
    # audio.start_listening_background()  # DISABLED
    
    # Create top-level state machine
    sm = smach.StateMachine(outcomes=['shutdown'])
    sm.userdata.target_plant = None
    sm.userdata.handoff_cmd = None # Init handoff variable
    sm.userdata.last_interaction = 0 # Init timestamp
    sm.userdata.current_location = None # Init context
    
    with sm:
        # 1. IDLE (Passive)
        smach.StateMachine.add('IDLE', IdleState(audio_manager, gesture_manager, display_manager, perception_manager), 
                               transitions={'detected':'GREETING'})
        
        # 2. GREETING (Active - Wave/Hello)
        smach.StateMachine.add('GREETING', GreetingState(audio_manager, gesture_manager, display_manager),
                               transitions={'succeeded':'COMMUNICATION'})
                               
        # 3. COMMUNICATION (Hub)
        # Self-loop logic can be handled by transition back to COMMUNICATION? 
        # But here we defined outcomes=['idle', 'start_tour', 'go_to_plant']
        # If Q&A happens, we might want to stay in COMMUNICATION. 
        # Let's assume CommunicationState handles Q&A internally or we add a transition 'self_loop'.
        # For this refactor, let's map 'idle' to 'IDLE'.
        smach.StateMachine.add('COMMUNICATION', CommunicationState(audio_manager, knowledge_manager, gesture_manager, display_manager),
                               transitions={'idle':'IDLE', 
                                            'start_tour':'TOUR', 
                                            'go_to_plant':'PREPARE_SINGLE'},
                               remapping={'handoff_cmd': 'handoff_cmd', 'target_plant': 'target_plant'})

        # 4. TOUR (Nested)
        tour_sm = create_tour_sm(nav_manager, audio_manager, knowledge_manager, wp_manager, perception_manager, gesture_manager, display_manager)
        smach.StateMachine.add('TOUR', tour_sm, 
                               transitions={'tour_finished':'COMMUNICATION',  # Return to Hub
                                            'tour_cancelled':'COMMUNICATION'},
                               remapping={'handoff_cmd': 'handoff_cmd'})

        # 5. SINGLE GOAL
        # PREPARE -> EXECUTE -> ARRIVAL
        smach.StateMachine.add('PREPARE_SINGLE', PrepareLeading(nav_manager, audio_manager, perception_manager),
                               transitions={'ready':'EXECUTE',
                                            'failed':'COMMUNICATION'})
                                            
        smach.StateMachine.add('EXECUTE', NavigateState(nav_manager, wp_manager, audio_manager, knowledge_manager, perception_manager, display_manager),
                               transitions={'arrived':'TURN_FACE_USER_SINGLE', # Changed to point to new state
                                            'aborted':'COMMUNICATION'})

        # Added missing State for Single Goal
        smach.StateMachine.add('TURN_FACE_USER_SINGLE', TurnFaceUser(nav_manager, audio_manager, perception_manager, display_manager),
                               transitions={'succeeded':'ARRIVAL_SINGLE',
                                            'failed':'ARRIVAL_SINGLE'})

        smach.StateMachine.add('ARRIVAL_SINGLE', ArrivalState(audio_manager, gesture_manager, knowledge_manager, display_manager),
                               transitions={'done':'COMMUNICATION'})
        
        # Add HANDLING_OBSTACLE state
        smach.StateMachine.add(
            'HANDLING_OBSTACLE',
            HandlingObstacle(audio_manager, perception_manager, nav_manager, display_manager),
            transitions={
                'clear': 'EXECUTE',
                'still_blocked': 'HANDLING_OBSTACLE',
                'cancelled': 'COMMUNICATION' # Changed from IDLE
            }
        )
        
    return sm, audio_manager

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
