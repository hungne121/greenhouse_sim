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
from actionlib_msgs.msg import GoalStatus
from modules.audio_manager import AudioManager
from modules.navigation_manager import NavigationManager
from modules.perception_manager import PerceptionManager
from modules.waypoint_manager import WaypointManager
from modules.knowledge_manager import KnowledgeManager
from modules.gesture_manager import GestureManager
from modules.display_manager import DisplayManager

# ============================================
# CONSTANTS & MAPPINGS
# ============================================
PLANT_IMAGE_MAP = {
    'hoa hồng': 'rose_plant',
    'tulip': 'tulip_plant',
    'xương rồng': 'cactus_plant',
    # Fallback to rose if not found, or add more
}

def get_plant_image(plant_name):
    """Normalize plant name to image file key"""
    for key, val in PLANT_IMAGE_MAP.items():
        if key in plant_name.lower():
            return val
    return 'rose_plant' # Default/Fallback

# ============================================
# STATE 1: IDLE (Chờ lệnh)
# ============================================

class IdleState(smach.State):
    """
    IDLE State - Listening for initial commands
    
    Outcomes:
        - 'start_tour': User requested full tour
        - 'go_to_plant': User requested specific plant
        - 'shutdown': User requested quit
    """
    def __init__(self, audio, knowledge, gesture, display):
        smach.State.__init__(self, 
                             outcomes=['start_tour', 'go_to_plant', 'shutdown'],
                             output_keys=['target_plant', 'handoff_cmd'], 
                             input_keys=['handoff_cmd'])
        self.audio = audio
        self.knowledge = knowledge
        self.gesture = gesture
        self.display = display
    
    def execute(self, userdata):
        rospy.loginfo("[IDLE] Waiting for command...")
        self.gesture.perform_gesture('reset_pose')
        self.display.show_text("Robot Pepper: Sẵn sàng phục vụ")
        """Helper to answer agriculture questions"""
        data = self.knowledge.plants_data.get(plant, {})
        
        if intent == 'plant_story':
            story = data.get('folklore', 'Chưa có thông tin.')
            self.audio.speak(f"Chuyện kể về {plant}: {story}")
        
        elif intent == 'ask_variety':
            variety = data.get('variety', 'Chưa rõ')
            origin = data.get('origin', '')
            self.audio.speak(f"{plant} là giống {variety}. Xuất xứ: {origin}.")
        
        elif intent == 'ask_technique':
            tech = data.get('planting_tech', 'Chưa có thông tin')
            soil = data.get('soil', '')
            self.audio.speak(f"Kỹ thuật trồng: {tech}. Đất: {soil}.")
        
        elif intent == 'ask_care':
            water = data.get('care_water', '')
            fert = data.get('fertilizer', '')
            self.audio.speak(f"Chăm sóc {plant}: {water}. {fert}.")

    
    def execute(self, userdata):
        rospy.loginfo("[IDLE] Waiting for command...")
        
        # Check Handoff Command from Tour
        start_text = None
        if hasattr(userdata, 'handoff_cmd') and userdata.handoff_cmd:
             rospy.loginfo(f"[IDLE] Processing Handoff Command: {userdata.handoff_cmd}")
             start_text = userdata.handoff_cmd
             userdata.handoff_cmd = None # Clear after consuming
        
        # Loop listening
        while not rospy.is_shutdown():
            if start_text:
                text = start_text
                start_text = None # Use once
            else:
                text = self.audio.listen_once(timeout=5)
            
            if not text:
                continue  # Keep listening
            
            intent, conf = self.audio.predict_intent(text)
            rospy.loginfo(f"[IDLE] Detected: {intent} ({conf:.2f})")
            
            if conf < 0.65:
                # Only ask to repeat if it's not a short/garbage noise (e.g. > 2 words)
                if len(text.split()) > 2:
                    self.audio.speak("Xin nhắc lại?")
                continue
            
            # Process intent
            if intent == 'quit_system':
                self.audio.speak("Tạm biệt!")
                return 'shutdown'
            
            elif intent == 'whole_tour':
                self.audio.speak("Tuyệt vời! Mời bạn đi theo tôi, chúng ta sẽ tham quan toàn bộ nhà kính.")
                return 'start_tour'
            
            elif intent == 'list_plants' or intent == 'ask_variety':
                # User asks what plants are here (general question)
                plant_names = ', '.join(self.knowledge.get_all_plant_names())
                self.audio.speak(f"Nhà kính có: {plant_names}.")
                # Continue listening
            
            elif intent == 'navigation_request':
                # Extract plant name
                plant_name = None
                for p in self.knowledge.get_all_plant_names():
                    if p in text:
                        plant_name = p
                        break
                
                if plant_name:
                    userdata.target_plant = plant_name
                    self.audio.speak(f"Được rồi, tôi sẽ dẫn bạn đến khu vực {plant_name}.")
                    return 'go_to_plant'
                else:
                    self.audio.speak("Bạn muốn đi đến cây nào? Có hồng, cúc, lan...")
            
            elif intent == 'greeting':
                self.audio.speak("Chào bạn! Tôi sẵn sàng phục vụ.")
            
            elif intent in ['ask_care', 'ask_technique', 'ask_variety', 'plant_story']:
                # User asks agriculture question without specifying plant
                self.audio.speak("Anh muốn biết về cây nào? Có hồng, cúc, lan, xương rồng...")
                
                # Wait for plant name
                plant_text = self.audio.listen_once(timeout=5)
                if plant_text:
                    plant_name = None
                    for p in self.knowledge.get_all_plant_names():
                        if p in plant_text:
                            plant_name = p
                            break
                    
                    if plant_name:
                        # Answer the question
                        self._answer_agriculture_question(plant_name, intent)
                    else:
                        self.audio.speak("Tôi không tìm thấy cây đó trong danh sách.")
            
            else:
                # Unknown intent
                self.audio.speak("Xin lỗi, tôi chưa hiểu. Thử nói lại nhé.")

# ============================================
# STATE 2: EXECUTE - Simple Navigation (No Concurrence)
# ============================================

class NavigateState(smach.State):
    """Simple navigation with Human-Aware Logic (Lead and Wait)"""
    def __init__(self, nav, wp_manager, audio, knowledge, perception):
        smach.State.__init__(self,
                             outcomes=['arrived', 'aborted'],
                             input_keys=['target_plant'])
        self.nav = nav
        self.wp_manager = wp_manager
        self.audio = audio
        self.knowledge = knowledge
        self.perception = perception
        
        # Robustness State
        self.last_speech_time = rospy.Time(0)
        self.start_exec_time = rospy.Time(0)
        self.human_stop_start_time = None
        
        # Robustness Constants
        self.SPEECH_COOLDOWN = rospy.Duration(8.0)
        self.WAIT_TIMEOUT = rospy.Duration(60.0)
        self.GRACE_PERIOD = rospy.Duration(3.0)
    
    def execute(self, userdata):
        plant = userdata.target_plant
        rospy.loginfo(f"[NAV] Going to {plant}...")
        
        # Get waypoint
        wp_key = self.wp_manager.get_plant_waypoint_key(plant)
        if not wp_key:
            self.audio.speak("Không tìm thấy vị trí.")
            return 'aborted'
        
        pose = self.wp_manager.get_waypoint_pose(wp_key)
        
        # Notify user (Already spoken in IDLE, but good for context)
        # self.audio.speak(f"Đang đi đến {plant}. Hãy đi theo tôi nhé.")
        
        # --- NAVIGATION LOOP WITH LOGIC ---
        self.nav.send_goal(pose, done_cb=self._done_cb)
        self.done = False
        self.nav_status = None
        self.is_paused = False
        self.pause_reason = None # 'blocked' or 'lagging'
        self.pause_start_time = None # Reset pause timer
        self.is_stop_timer_active = False # Reset human stop timer
        
        rate = rospy.Rate(5)
        
        while not self.done and not rospy.is_shutdown():
            # --- HUMAN AWARE LOGIC ---
            human_dist = self.perception.get_nearest_human_distance()
            human_angle = self.perception.get_human_angle()
            rospy.loginfo_throttle(1.0, f"[NAV] Human Dist: {human_dist:.2f}m, Angle: {human_angle:.2f}rad")
            
            can_speak = (rospy.Time.now() - self.last_speech_time) > self.SPEECH_COOLDOWN
            
            # --- RESUME LOGIC (Check FIRST) ---
            if self.is_paused:
                if self.pause_reason == 'blocked':
                    # If blocked, resume as soon as cleared (> 0.8m) or lost (None -> >5.0)
                    if human_dist > 0.8:
                        rospy.loginfo("[NAV] Obstacle cleared. Resuming.")
                        self.audio.speak("Được rồi, tôi đi tiếp.")
                        self.nav.send_goal(pose, done_cb=self._done_cb)
                        self.is_paused = False
                        self.pause_reason = None
                
                elif self.pause_reason == 'lagging':
                    # If lagging, resume when close enough (< 2.0m)
                    if human_dist < 2.0:
                         rospy.loginfo("[NAV] Human caught up. Resuming.")
                         self.audio.speak("Đi tiếp nào.")
                         self.nav.send_goal(pose, done_cb=self._done_cb)
                         self.is_paused = False
                         self.pause_reason = None
            
            # --- PAUSE LOGIC ---
            else:
                # 1. TOO CLOSE (< 0.8m)
                if human_dist < 0.8:
                    if abs(human_angle) > 1.5:
                         # Human is behind -> Ignore/Maintain Speed
                         rospy.loginfo_throttle(2.0, "[NAV] Human close behind. Keeping course.")
                    else:
                        # Human is front -> Critical -> Back Up
                        rospy.logwarn(f"[NAV] Blocked front ({human_dist:.2f}m)! Backing up.")
                        self.is_paused = True
                        self.pause_reason = 'blocked'
                        self.pause_start_time = rospy.Time.now()
                        self.nav.cancel_goal()
                        
                        if can_speak:
                            self.audio.speak("Bạn đứng gần quá. Tôi lùi lại nhé.")
                            self.last_speech_time = rospy.Time.now()
                        
                        self.nav.backup(duration=1.5)

                # 2. TOO FAR (> 2.0m) -> Only check if NOT already paused
                elif human_dist > 2.0:
                    is_moving = self.perception.is_human_moving()
                    
                    if is_moving:
                         # Human lagging but moving
                         rospy.logwarn(f"[NAV] Human lagging ({human_dist:.2f}m). Waiting.")
                         self.nav.cancel_goal()
                         self.audio.speak("Tôi đi chậm lại nhé.")
                         self.is_paused = True
                         self.pause_reason = 'lagging'
                    
                    else:
                        # Human STOPPED -> Something wrong?
                        if not self.is_stop_timer_active:
                            self.human_stop_start_time = rospy.Time.now()
                            self.is_stop_timer_active = True
                        
                        if (rospy.Time.now() - self.human_stop_start_time).to_sec() > 2.0:
                            rospy.logwarn(f"[NAV] Human stopped ({human_dist:.2f}m). Checking.")
                            self.nav.cancel_goal()
                            self.audio.speak("Có chuyện gì vậy?")
                            self.is_paused = True
                            self.pause_reason = 'lagging' # Treat same as lagging for resume purposes
            
            rate.sleep()
        
        if self.nav_status == GoalStatus.SUCCEEDED:
            return 'arrived'
        
        # Check for PREEMPTED vs ABORTED
        if self.nav_status == GoalStatus.PREEMPTED:
             if self.is_paused:
                 # Check for Timeout
                 if self.pause_start_time is None:
                     self.pause_start_time = rospy.Time.now() # Safety callback
                 
                 wait_duration = (rospy.Time.now() - self.pause_start_time).to_sec()
                 
                 # If waiting too long (> 60s)
                 if wait_duration > self.WAIT_TIMEOUT.to_sec():
                     rospy.logwarn(f"[NAV] Wait timeout ({wait_duration:.1f}s). Aborting.")
                     self.audio.speak("Lâu quá không thấy bạn. Tôi về đây.")
                     return 'aborted'
                 
                 # Still waiting -> Stay in loop
                 pass 
             else:
                 return 'aborted'
        elif self.nav_status == GoalStatus.ABORTED:
             self.audio.speak("Không đến được.")
             return 'aborted'
             
        # Fallback for unknown status
        return 'aborted'


    def _done_cb(self, status, result):
        if not self.is_paused: # Only count if not manually cancelled
            self.done = True
            self.nav_status = status

# No more Concurrence - keep it simple!

# ============================================
# STATE 3: HANDLING_OBSTACLE
# ============================================

class HandlingObstacle(smach.State):
    """
    Handle obstacle situation (human too close)
    """
    def __init__(self, audio, perception, nav):
        smach.State.__init__(self, outcomes=['clear', 'still_blocked', 'cancelled'])
        self.audio = audio
        self.perception = perception
        self.nav = nav
    
    def execute(self, userdata):
        rospy.loginfo("[OBSTACLE] Handling obstacle...")
        
        # Cancel navigation
        self.nav.cancel_goal()
        
        # Polite request
        self.audio.speak("Xin lỗi, nhường đường cho tôi đi qua.")
        
        # Wait 3 seconds
        rospy.sleep(3)
        
        # Check again
        distance = self.perception.get_nearest_human_distance()
        
        if distance > 1.0:
            self.audio.speak("Cảm ơn. Tôi tiếp tục đi.")
            return 'clear'
        else:
            rospy.logwarn("[OBSTACLE] Still blocked")
            return 'still_blocked'

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
# STATE 5: TOUR (Nested State Machine)
# ============================================
# Note: ArrivalState is legacy/unused in Tour flow now, keeping for safety


class TourNavigate(smach.State):
    """Navigate to next plant in tour (Human-Aware)"""
    def __init__(self, nav, wp_manager, audio, perception, gesture, display):
        smach.State.__init__(self,
                             outcomes=['arrived', 'tour_interrupted', 'tour_complete'],
                             input_keys=['tour_index', 'tour_keys'],
                             output_keys=['tour_index', 'current_plant'])
        self.nav = nav
        self.wp_manager = wp_manager
        self.audio = audio
        self.perception = perception
        self.display = display
        self.gesture = gesture
        
        # Robustness State
        self.last_speech_time = rospy.Time(0)
        self.start_exec_time = rospy.Time(0)
        self.human_stop_start_time = None
        
        # Robustness Constants
        self.SPEECH_COOLDOWN = rospy.Duration(8.0)
        self.WAIT_TIMEOUT = rospy.Duration(60.0)
        self.GRACE_PERIOD = rospy.Duration(3.0)
    
    def execute(self, userdata):
        # Check if tour complete
        if userdata.tour_index >= len(userdata.tour_keys):
            return 'tour_complete'
        
        # Get next plant
        key = userdata.tour_keys[userdata.tour_index]
        plant = self.wp_manager.waypoints[key].get('plant_type', 'cây này')
        pose = self.wp_manager.get_waypoint_pose(key)
        
        rospy.loginfo(f"[TOUR] Navigating to plant {userdata.tour_index + 1}: {plant}")
        
        userdata.current_plant = plant
        
        self.display.show_text(f"Đang dẫn bạn đến: {plant}")
        self.gesture.perform_gesture("walk_pose") # Arms down ONLY (Safe walk)
        self.gesture.perform_gesture("look_around") # Head moves independently
        
        # Navigate Setup
        def done_cb(status, result):
            if not self.is_paused:
                self.nav_done = True
                self.nav_status = status
        
        self.nav_done = False
        self.nav_status = None
        self.is_paused = False
        self.nav.send_goal(pose, done_cb=done_cb)
        
        # Monitoring Loop
        rate = rospy.Rate(5)
        while not self.nav_done and not rospy.is_shutdown():
            # Check Preemption
            if self.preempt_requested():
                self.service_preempt()
                self.nav.cancel_goal()
                return 'tour_interrupted'
            
            # --- HUMAN AWARE LOGIC ---
            # --- HUMAN AWARE LOGIC ---
            human_dist = self.perception.get_nearest_human_distance()
            human_angle = self.perception.get_human_angle()
            rospy.loginfo_throttle(1.0, f"[TOUR] Human Dist: {human_dist:.2f}m, Angle: {human_angle:.2f}rad")
            
            can_speak = (rospy.Time.now() - self.last_speech_time) > self.SPEECH_COOLDOWN

            # 1. TOO CLOSE (< 0.8m)
            if human_dist < 0.8:
                # Check orientation
                # Angle 0 is front. +/- 3.14 is back.
                # If abs(angle) > 1.5 (~90deg), human is likely BEHIND or SIDE-BACK -> SAFE TO RUN
                if abs(human_angle) > 1.5:
                     # Human is behind -> Speed Up (or just don't stop)
                     rospy.loginfo_throttle(2.0, "[TOUR] Human close behind. Maintaining speed.")
                     pass
                else:
                    # Human is FRONT -> Blocking -> Back Up
                    if not self.is_paused:
                        rospy.logwarn(f"[TOUR] Blocked in front ({human_dist:.2f}m)! Backing up.")
                        self.is_paused = True
                        self.pause_start_time = rospy.Time.now()
                        self.nav.cancel_goal()
                        
                        if can_speak:
                            self.audio.speak("Bạn đứng chắn đường rồi. Để tôi lùi lại.")
                            self.last_speech_time = rospy.Time.now()
                        
                        # HRI Warning
                        self.display.show_text("VẬT CẢN! Xin nhường đường.")
                        self.gesture.perform_gesture('nod_head')

                        # Execute Backup
                        self.nav.backup(duration=1.5)
                        
                        # Resume immediately? Or wait? 
                        # Logic falls through to 'RESUME' check next loop if distance clears.
                        # But we need to reset pause to allow resume logic to trigger.
                        # Actually, RESUME logic requires is_paused=True. It will trigger when DIST > 0.8.
                        pass
            
            # 2. TOO FAR (> 2.0m)
            elif human_dist > 2.0:
                is_moving = self.perception.is_human_moving()
                
                if is_moving:
                     # Reset stop timer
                     self.human_stop_start_time = None
                     
                     if not self.is_paused:
                        rospy.logwarn(f"[TOUR] Human lagging ({human_dist:.2f}m). Waiting.")
                        self.is_paused = True # Flag FIRST
                        self.pause_start_time = rospy.Time.now()
                        self.nav.cancel_goal()
                        if can_speak:
                             self.audio.speak("Tôi đợi bạn một chút.")
                             self.last_speech_time = rospy.Time.now()
                else:
                    # Human Stopped -> Check duration (2s)
                    if self.human_stop_start_time is None:
                        self.human_stop_start_time = rospy.Time.now()
                    
                    # Calculate duration
                    stop_duration = (rospy.Time.now() - self.human_stop_start_time).to_sec()
                    
                    if stop_duration > 2.0:
                        if not self.is_paused:
                            rospy.logwarn(f"[TOUR] Human stopped > 2s. Checking.")
                            self.is_paused = True # Flag FIRST
                            self.pause_start_time = rospy.Time.now()
                            self.nav.cancel_goal()
                            if can_speak:
                                 self.audio.speak("Sao thế? Bạn có cần tôi giúp gì không?")
                                 self.last_speech_time = rospy.Time.now()
            
            # 3. RESUME
            elif self.is_paused and human_dist < 1.5 and human_dist > 0.8:
                rospy.loginfo("[TOUR] Resuming navigation.")
                self.audio.speak("Được rồi, đi tiếp nào.")
                self.nav.send_goal(pose, done_cb=done_cb)
                self.is_paused = False
            
            rate.sleep()
        
        if self.nav_status == GoalStatus.SUCCEEDED:
            return 'arrived'
        
        if self.nav_status == GoalStatus.PREEMPTED:
            if self.is_paused:
                 # Check for Timeout
                 if self.pause_start_time is None:
                     self.pause_start_time = rospy.Time.now()
                 
                 wait_duration = (rospy.Time.now() - self.pause_start_time).to_sec()
                 
                 if wait_duration > self.WAIT_TIMEOUT.to_sec():
                     rospy.logwarn(f"[TOUR] Wait timeout ({wait_duration:.1f}s). Aborting tour.")
                     self.audio.speak("Bạn đi đâu mất rồi? Tôi kết thúc tour tại đây.")
                     return 'tour_interrupted'
                     
                 # Still waiting
                 pass 
            else:
                return 'tour_interrupted'
        elif self.nav_status == GoalStatus.ABORTED:
             self.audio.speak("Không đến được.")
             return 'tour_interrupted'
            
        return 'tour_interrupted'

class TourIntro(smach.State):
    """Give detailed introduction at each plant"""
    def __init__(self, audio, knowledge, gesture, display):
        smach.State.__init__(self,
                             outcomes=['intro_done'],
                             input_keys=['current_plant', 'tour_index'],
                             output_keys=['tour_index'])
        self.audio = audio
        self.knowledge = knowledge
        self.gesture = gesture
        self.display = display
    
    def execute(self, userdata):
        plant = userdata.current_plant
        data = self.knowledge.plants_data.get(plant, {})
        
        # Visual & Gesture
        # Resolve image filename safely
        img_name = get_plant_image(plant)
        self.display.show_image(img_name) 
        self.gesture.perform_gesture('wave_hand') # Point to plant
        
        # Concise Vietnamese introduction
        intro = f"Chúng ta đã đến khu vực {plant}. "
        intro += f"Đây là giống {data.get('variety', 'chưa rõ')}. "
        # Use concise description instead of long characteristics
        intro += f"{data.get('description', '')}"
        
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
        
        # Listen for response
        max_attempts = 3
        for attempt in range(max_attempts):
            text = self.audio.listen_once(timeout=5)
            
            if not text:
                if attempt < max_attempts - 1:
                    self.audio.speak("Xin nhắc lại?")
                    continue
                else:
                    return 'continue_tour'
            
            intent, conf = self.audio.predict_intent(text)
            
            if conf < 0.65:
                continue
            
            # Universal Intent Handling
            # 1. Cancel/Stop (Check FIRST for negative keywords)
            if any(w in text for w in ['dừng', 'thôi', 'không muốn', 'không', 'kết thúc', 'stop']):
                 rospy.loginfo(f"[TOUR_WAIT_USER] User cancelled: {text}")
                 userdata.handoff_cmd = None # DO NOT pass 'end tour' to IDLE (prevents shutdown)
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
        smach.StateMachine.add(
            'TOUR_NAVIGATE',
            TourNavigate(nav, wp_manager, audio, perception, gesture, display),
            transitions={
                'arrived': 'TOUR_INTRO',
                'tour_interrupted': 'tour_cancelled',
                'tour_complete': 'tour_finished'
            }
        )
        
        smach.StateMachine.add(
            'TOUR_INTRO',
            TourIntro(audio, knowledge, gesture, display),
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
            IdleState(audio, knowledge, gesture, display),
            transitions={
                'start_tour': 'TOUR',
                'go_to_plant': 'EXECUTE',
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
        
        # Add EXECUTE - Simple navigation (no concurrency)
        smach.StateMachine.add(
            'EXECUTE',
            NavigateState(nav, wp_manager, audio, knowledge, perception),  # Pass perception
            transitions={
                'arrived': 'ARRIVAL',
                'aborted': 'IDLE'
            }
        )
        
        # Add HANDLING_OBSTACLE state
        smach.StateMachine.add(
            'HANDLING_OBSTACLE',
            HandlingObstacle(audio, perception, nav),
            transitions={
                'clear': 'EXECUTE',
                'still_blocked': 'HANDLING_OBSTACLE',
                'cancelled': 'IDLE'
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
