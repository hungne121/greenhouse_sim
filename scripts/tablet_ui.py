#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import String
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QListWidget, QPushButton, 
                             QFrame, QSizePolicy, QListWidgetItem)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette

# ROS Communicator Class (Since Qt runs in main thread, ROS callbacks need to signal it)
class Comm(QObject):
    dialogue_signal = pyqtSignal(str)
    status_signal = pyqtSignal(str)
    command_signal = pyqtSignal(str)

class TabletUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # ROS Init
        rospy.init_node('pepper_tablet_ui', anonymous=True)
        self.comm = Comm()
        self.comm.dialogue_signal.connect(self.update_dialogue)
        self.comm.status_signal.connect(self.update_status)
        self.comm.command_signal.connect(self.handle_command)
        
        # Subscribers
        rospy.Subscriber('/pepper/ui/dialogue', String, self.dialogue_cb)
        rospy.Subscriber('/pepper/ui/status', String, self.status_cb)
        rospy.Subscriber('/pepper/ui/options', String, self.options_cb) # New subscriber
        rospy.Subscriber('/pepper/tablet/command', String, self.command_cb)
        
        # UI Setup
        self.setWindowTitle("Pepper Tablet Interface (PyQt5)")
        self.setGeometry(100, 100, 1280, 800) # Larger window for larger fonts
        
        self.setup_ui()
        self.apply_styles()
        
    # --- ROS Callbacks ---
    def dialogue_cb(self, msg):
        self.comm.dialogue_signal.emit(msg.data)
        
    def status_cb(self, msg):
        self.comm.status_signal.emit(msg.data)
        
    def options_cb(self, msg):
        # Expect JSON string: '["Option 1", "Option 2"]'
        self.comm.command_signal.emit(f"OPTIONS:{msg.data}")
        
    def command_cb(self, msg):
        self.comm.command_signal.emit(msg.data)
        
    # --- UI Setup ---
    def setup_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # Main Layout: Vertical [Header | Content/Chat | Buttons]
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # --- HEADER (Status) ---
        self.header = QFrame()
        self.header.setObjectName("Header")
        self.header.setFixedHeight(100) # Increased height
        header_layout = QHBoxLayout(self.header)
        
        self.status_lbl = QLabel("IDLE")
        self.status_lbl.setObjectName("StatusLabel")
        self.status_lbl.setFont(QFont("Roboto", 28, QFont.Bold)) # Larger Font
        self.status_lbl.setAlignment(Qt.AlignCenter)
        
        header_layout.addWidget(self.status_lbl)
        
        # --- CHAT AREA ---
        self.chat_list = QListWidget()
        self.chat_list.setObjectName("ChatList")
        self.chat_list.setFont(QFont("Roboto", 60)) # Reduced to 60 as requested
        self.chat_list.setFocusPolicy(Qt.NoFocus)
        self.chat_list.setSpacing(20) # More spacing
        self.chat_list.setUniformItemSizes(False)
        self.chat_list.setWordWrap(True)
        
        # --- BUTTONS AREA ---
        self.btn_frame = QFrame()
        self.btn_frame.setObjectName("ButtonFrame")
        self.btn_frame.setFixedHeight(150) # Increased height
        self.btn_layout = QHBoxLayout(self.btn_frame)
        self.btn_layout.setContentsMargins(30, 20, 30, 20)
        self.btn_layout.setSpacing(40)
        
        # Default Buttons
        self.update_buttons(["Bắt đầu Tour", "Tìm cây"])
        
        main_layout.addWidget(self.header)
        main_layout.addWidget(self.chat_list)
        main_layout.addWidget(self.btn_frame)
        
    def update_buttons(self, options):
        # Clear existing
        for i in reversed(range(self.btn_layout.count())): 
            item = self.btn_layout.itemAt(i)
            if item.widget():
                item.widget().setParent(None)
            
        for opt in options:
            btn = QPushButton(opt)
            btn.setFixedHeight(80) # Taller buttons
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(lambda checked, text=opt: self.on_option_click(text))
            self.btn_layout.addWidget(btn)
            
    def on_option_click(self, text):
        rospy.loginfo(f"[UI] Option Clicked: {text}")
        # Could publish back to a topic if needed
        
    def apply_styles(self):
        # Modern Dark Theme - Adjusted for Visibility
        self.setStyleSheet("""
            QMainWindow { background-color: #121212; }
            #Header { background-color: #1f1f1f; border-bottom: 2px solid #333; }
            #StatusLabel { color: #00e5ff; font-weight: bold; }
            #ChatList { 
                background-color: #121212; 
                border: none; 
                padding: 30px;
            }
            #ButtonFrame { background-color: #1f1f1f; border-top: 2px solid #333; }
            QPushButton {
                background-color: #6200ea; /* Deeper Purple */
                color: #ffffff;
                border-radius: 20px;
                font-size: 28px; /* LARGE FONT */
                font-weight: bold;
                border: 2px solid #7c4dff;
            }
            QPushButton:hover { background-color: #7c4dff; }
            QPushButton:pressed { background-color: #3700b3; }
        """)
        
    # --- Update Slots ---
    def update_dialogue(self, text):
        # Format: "Speaker: Text"
        parts = text.split(":", 1)
        speaker = parts[0].strip()
        message = parts[1].strip() if len(parts) > 1 else ""
        
        # Create container widget for the row
        container_widget = QWidget()
        layout = QHBoxLayout(container_widget)
        layout.setContentsMargins(10, 5, 10, 5) # Outer margins
        
        # Create the Bubble Label (Native Widget for proper styling)
        lbl = QLabel(message)
        lbl.setWordWrap(True)
        lbl.setFont(self.chat_list.font()) # Use the font set on list
        
        # Dynamic Style based on speaker
        if speaker.lower() == "user":
            # Right Aligned, Cyan Bubble
            layout.addStretch() # Push to right
            layout.addWidget(lbl)
            
            lbl.setStyleSheet(f"""
                QLabel {{
                    background-color: #00bcd4;
                    color: #ffffff;
                    padding: 20px;
                    border-radius: 20px;
                    border: 2px solid #00acc1;
                }}
            """)
        else: # Robot
            # Left Aligned, Dark Bubble
            layout.addWidget(lbl)
            layout.addStretch() # Push to left
            
            lbl.setStyleSheet(f"""
                QLabel {{
                    background-color: #424242;
                    color: #ffffff;
                    padding: 20px;
                    border-radius: 20px;
                    border: 1px solid #616161;
                }}
            """)
            
        # Add to list
        item = QListWidgetItem()
        item.setSizeHint(container_widget.sizeHint()) 
        
        self.chat_list.addItem(item)
        self.chat_list.setItemWidget(item, container_widget)
        self.chat_list.scrollToBottom()
        
    def update_status(self, status):
        self.status_lbl.setText(status.upper().replace("_", " "))
        # Colors... (re-use logic)
            
    def handle_command(self, cmd):
        # e.g. SHOW_TEXT:..., SHOW_IMAGE:..., OPTIONS:...
        if cmd.startswith("OPTIONS:"):
            import json
            try:
                json_str = cmd.split(":", 1)[1]
                options = json.loads(json_str)
                self.update_buttons(options)
            except Exception as e:
                rospy.logerr(f"[UI] Error parsing options: {e}")
        
        elif cmd.startswith("SHOW_TEXT:"):
            text = cmd.split(":", 1)[1]
            # self.content_lbl.setText(text) # Disabled for Bubble view
            # Maybe show as system message in chat
            self.update_dialogue(f"System: {text}")
            
        elif cmd.startswith("SHOW_IMAGE:"):
            img_name = cmd.split(":", 1)[1]
            self.update_dialogue(f"Image: [Hiển thị ảnh {img_name}]")
            
        elif cmd == "CLEAR":
            pass # self.content_lbl.hide()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Enable High DPI
    app.setAttribute(Qt.AA_EnableHighDpiScaling)
    
    ui = TabletUI()
    ui.show()
    
    sys.exit(app.exec_())
