#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Audio Manager - Layer 2: Functional Module
Handles all audio I/O with strict Walkie-Talkie protocol
"""

import speech_recognition as sr
from gtts import gTTS
import os
import time
import pygame
import pickle
import rospy
import threading
from queue import Queue
import webrtcvad
import struct

# ALSA Error Suppression
from ctypes import *
from contextlib import contextmanager

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextmanager
def no_alsa_error():
    try:
        asound = cdll.LoadLibrary('libasound.so')
        asound.snd_lib_error_set_handler(c_error_handler)
    except:
        pass
    
    try:
        yield
    finally:
        try:
            asound.snd_lib_error_set_handler(None)
        except:
            pass

class AudioManager:
    """
    Professional Audio Manager with Thread-Safe Walkie-Talkie Protocol
    """
    def __init__(self):
        # Speech Recognition
        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = 1.5
        self.recognizer.energy_threshold = 4000  # Filter echo (1000-2000), allow speech (>4000)
        
        # WebRTC VAD for voice activity detection
        self.vad = webrtcvad.Vad(1)  # Aggressiveness: 0-3 (1 = more sensitive)
        
        # NLU Model
        self.classifier = None
        self._load_model()
        
        # Audio Output
        pygame.mixer.init()
        
        # Thread Safety - Walkie-Talkie Protocol
        self.audio_lock = threading.Lock()  # Mutex for speaker
        self.is_speaking = False  # Flag for state checking
        self.last_speak_time = 0  # Timestamp of last speech finish
        
        # Background Listening
        self.listening_active = False
        self.listen_thread = None
        self.audio_queue = Queue()  # Thread-safe queue
        
        rospy.loginfo("[AudioManager] Initialized with thread-safe locking.")
    
    def _load_model(self):
        """Load NLU intent classification model"""
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.abspath(os.path.join(current_dir, '..', 'model.pkl'))
            
            if os.path.exists(model_path):
                with open(model_path, "rb") as f:
                    self.classifier = pickle.load(f)
                rospy.loginfo("[AudioManager] NLU model loaded.")
            else:
                rospy.logwarn(f"[AudioManager] Model not found at {model_path}")
        except Exception as e:
            rospy.logerr(f"[AudioManager] Model load error: {e}")
    
    def speak(self, text, blocking=True):
        """
        Speak text with automatic microphone locking (Walkie-Talkie)
        
        Args:
            text: Vietnamese text to speak
            blocking: If True, wait until speech finishes. If False, return immediately.
        """
        rospy.loginfo(f"🤖 [AudioManager] Speaking: {text}")
        
        def _speak_worker():
            with self.audio_lock:  # ACQUIRE LOCK - Mic is now CLOSED
                self.is_speaking = True
                try:
                    tts = gTTS(text=text, lang='vi')
                    filename = "/tmp/robot_voice.mp3"
                    tts.save(filename)
                    
                    pygame.mixer.music.load(filename)
                    pygame.mixer.music.play()
                    
                    while pygame.mixer.music.get_busy():
                        pygame.time.Clock().tick(10)
                    
                    # Echo cancellation buffer (reduced for faster response)
                    time.sleep(1.5)
                    
                    pygame.mixer.music.unload()
                except Exception as e:
                    rospy.logerr(f"[AudioManager] TTS Error: {e}")
                finally:
                    self.is_speaking = False  # RELEASE FLAG
                    self.last_speak_time = time.time()  # Record finish time
            # Lock auto-released here - Mic can open
        
        if blocking:
            _speak_worker()
        else:
            thread = threading.Thread(target=_speak_worker)
            thread.daemon = True
            thread.start()
    
    def listen_once(self, timeout=5):
        """
        Single blocking listen
        
        Returns:
            str: Recognized text (lowercase) or empty string
        """
        # Wait for speaking to finish AND echo to fade
        while True:
            if not self.is_speaking:
                time_since_speak = time.time() - self.last_speak_time
                if time_since_speak > 0.5:  # Reduced from 1.0s for faster response
                    break
            time.sleep(0.1)
        
        # Now safe to open microphone
        with no_alsa_error():
            try:
                with sr.Microphone() as source:
                    # AUDIO NOTIFICATION for user
                    rospy.loginfo("👂 Đang lắng nghe...")
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.2)
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                    text = self.recognizer.recognize_google(audio, language="vi-VN")
                    rospy.loginfo(f"🗣️ Heard: {text}")
                    return text.lower()
                    
            except sr.WaitTimeoutError:
                return ""
            except sr.UnknownValueError:
                return ""
            except sr.RequestError as e:
                rospy.logerr(f"[AudioManager] API Error: {e}")
                return ""
    

    def wait_for_speech(self, timeout=30):
        """
        Wait for voice activity using WebRTC VAD (low CPU)
        
        Args:
            timeout: Maximum time to wait for speech (seconds)
            
        Returns:
            bool: True if speech detected, False if timeout
        """
        # Wait for robot to finish speaking
        while self.is_speaking:
            time.sleep(0.1)
        
        rospy.loginfo("👂 [AudioManager] Waiting for voice activity...")
        
        # VAD parameters
        sample_rate = 16000
        frame_duration_ms = 30  # 10, 20, or 30 ms
        frame_size = int(sample_rate * frame_duration_ms / 1000)
        
        start_time = time.time()
        
        with no_alsa_error():
            try:
                with sr.Microphone(sample_rate=sample_rate) as source:
                    # Quick ambient noise adjustment
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.3)
                    
                    consecutive_voiced = 0
                    required_voiced_frames = 5  # ~150ms of speech (reduced for sensitivity)
                    
                    while time.time() - start_time < timeout:
                        # Check if robot started speaking
                        if self.is_speaking:
                            rospy.loginfo("[AudioManager] VAD interrupted by robot speech")
                            return False
                        
                        # Read audio frame
                        try:
                            audio_data = source.stream.read(frame_size, exception_on_overflow=False)
                        except:
                            continue
                        
                        # VAD detection
                        is_speech = self.vad.is_speech(audio_data, sample_rate)
                        
                        if is_speech:
                            consecutive_voiced += 1
                            if consecutive_voiced >= required_voiced_frames:
                                rospy.loginfo("🎤 [AudioManager] Voice detected! Activating full recognition...")
                                return True
                        else:
                            consecutive_voiced = 0
                    
                    rospy.loginfo("[AudioManager] VAD timeout - no speech detected")
                    return False
                    
            except Exception as e:
                rospy.logerr(f"[AudioManager] VAD error: {e}")
                return False
    def wait_for_speech(self, timeout=30):
        """
        Wait for voice activity using WebRTC VAD (low CPU)
        
        Args:
            timeout: Maximum time to wait for speech (seconds)
            
        Returns:
            bool: True if speech detected, False if timeout
        """
        # Wait for robot to finish speaking
        while self.is_speaking:
            time.sleep(0.1)
        
        rospy.loginfo("👂 [AudioManager] Waiting for voice activity...")
        
        # VAD parameters
        sample_rate = 16000
        frame_duration_ms = 30  # 10, 20, or 30 ms
        frame_size = int(sample_rate * frame_duration_ms / 1000)
        
        start_time = time.time()
        
        with no_alsa_error():
            try:
                with sr.Microphone(sample_rate=sample_rate) as source:
                    # Quick ambient noise adjustment
                    self.recognizer.adjust_for_ambient_noise(source, duration=0.3)
                    
                    consecutive_voiced = 0
                    required_voiced_frames = 10  # ~300ms of speech
                    
                    while time.time() - start_time < timeout:
                        # Check if robot started speaking
                        if self.is_speaking:
                            rospy.loginfo("[AudioManager] VAD interrupted by robot speech")
                            return False
                        
                        # Read audio frame
                        try:
                            audio_data = source.stream.read(frame_size, exception_on_overflow=False)
                        except:
                            continue
                        
                        # VAD detection
                        is_speech = self.vad.is_speech(audio_data, sample_rate)
                        
                        if is_speech:
                            consecutive_voiced += 1
                            if consecutive_voiced >= required_voiced_frames:
                                rospy.loginfo("🎤 [AudioManager] Voice detected! Activating full recognition...")
                                return True
                        else:
                            consecutive_voiced = 0
                    
                    rospy.loginfo("[AudioManager] VAD timeout - no speech detected")
                    return False
                    
            except Exception as e:
                rospy.logerr(f"[AudioManager] VAD error: {e}")
                return False

    def start_listening_background(self, callback=None):
        """
        Start continuous background listening thread
        
        Args:
            callback: Optional function(text) called when speech detected
        """
        if self.listening_active:
            rospy.logwarn("[AudioManager] Background listening already active!")
            return
        
        self.listening_active = True
        
        def _listen_loop():
            while self.listening_active and not rospy.is_shutdown():
                text = self.listen_once(timeout=5)
                if text:
                    self.audio_queue.put(text)
                    if callback:
                        callback(text)
        
        self.listen_thread = threading.Thread(target=_listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()
        
        rospy.loginfo("[AudioManager] Background listening started.")
    
    def stop_listening_background(self):
        """Stop background listening thread"""
        self.listening_active = False
        if self.listen_thread:
            self.listen_thread.join(timeout=2.0)
        rospy.loginfo("[AudioManager] Background listening stopped.")
    
    def get_queued_speech(self):
        """
        Get speech from queue (non-blocking)
        
        Returns:
            str or None: Next queued speech text
        """
        if not self.audio_queue.empty():
            return self.audio_queue.get()
        return None
    
    def predict_intent(self, text):
        """
        Predict intent from text using NLU model
        
        Returns:
            tuple: (intent_str, confidence_float)
        """
        if not self.classifier:
            return None, 0.0
        
        try:
            proba = self.classifier.predict_proba([text])[0]
            max_proba = max(proba)
            intent = self.classifier.classes_[proba.argmax()]
            return intent, max_proba
        except Exception as e:
            rospy.logerr(f"[AudioManager] Intent prediction error: {e}")
            return None, 0.0

# Test
if __name__ == "__main__":
    rospy.init_node('audio_test')
    audio = AudioManager()
    
    audio.speak("Xin chào. Tôi là Audio Manager.")
    text = audio.listen_once()
    if text:
        intent, conf = audio.predict_intent(text)
        print(f"Intent: {intent} ({conf:.2f})")
