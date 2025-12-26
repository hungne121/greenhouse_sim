#!/usr/bin/env python3
import rospy
import tkinter as tk
from std_msgs.msg import String
try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    rospy.logwarn("[TabletUI] PIL.ImageTk not found. Images will not be displayed (Text only). Install: sudo apt-get install python3-pil.imagetk")
    HAS_PIL = False
import os
import threading

class TabletUI:
    def __init__(self):
        rospy.init_node('pepper_tablet_ui', anonymous=True)
        
        # UI Setup
        self.root = tk.Tk()
        self.root.title("Pepper Tablet Interface")
        self.root.geometry("800x600")
        self.root.configure(bg="#f0f0f0")
        
        self.header_label = tk.Label(self.root, text="PEPPER TABLET", font=("Arial", 20, "bold"), bg="#ddd", pady=10)
        self.header_label.pack(fill=tk.X)
        
        self.content_label = tk.Label(self.root, text="Waiting for content...", font=("Arial", 24), bg="#fff", wraplength=700, height=10)
        self.content_label.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)
        
        self.status_label = tk.Label(self.root, text="Status: Connected", font=("Consolas", 10), bg="#eee", anchor="w")
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X)
        
        # ROS Subscriber
        self.sub = rospy.Subscriber('/pepper/tablet/command', String, self.callback)
        
        # Handle Close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        rospy.loginfo("[TabletUI] Started. Waiting for commands...")

    def callback(self, msg):
        cmd = msg.data
        rospy.loginfo(f"[TabletUI] Received: {cmd}")
        
        if cmd.startswith("SHOW_TEXT:"):
            text = cmd.split(":", 1)[1]
            self.root.after(0, self.update_text, text)
            
        elif cmd.startswith("SHOW_IMAGE:"):
            img_name = cmd.split(":", 1)[1]
            self.root.after(0, self.update_image, img_name)
            
        elif cmd == "CLEAR":
            self.root.after(0, self.clear_screen)

    def update_text(self, text):
        self.content_label.config(text=text, image="", bg="#ffffff", fg="#333333")
        self.status_label.config(text=f"Showing text: {text}")

    def update_image(self, img_name):
        if not HAS_PIL:
            self.content_label.config(text=f"[IMAGE Placeholder]\n(Missing PIL Library)\n\n{img_name}.jpg", bg="#e6f7ff", fg="#0050b3")
            return

        # Construct path: ../images/{img_name}.jpg or .png
        # Current script is in scripts/, so images are in ../images/
        base_path = os.path.dirname(os.path.abspath(__file__))
        image_dir = os.path.join(base_path, '../images')
        
        # Try extensions
        found_path = None
        for ext in ['.jpg', '.png', '.jpeg']:
            p = os.path.join(image_dir, img_name + ext)
            if os.path.exists(p):
                found_path = p
                break
        
        if found_path:
            try:
                # Load and Resize
                load = Image.open(found_path)
                load = load.resize((400, 300), Image.ANTIALIAS)
                render = ImageTk.PhotoImage(load)
                
                # Update Label
                self.content_label.config(image=render, text="") # Remove text
                self.content_label.image = render # Keep reference!
                self.status_label.config(text=f"Showing image: {os.path.basename(found_path)}")
                rospy.loginfo(f"[TabletUI] Loaded image: {found_path}")
            except Exception as e:
                 rospy.logerr(f"[TabletUI] Failed to load image: {e}")
                 self.content_label.config(text=f"[ERROR]\nCould not load {img_name}\n{e}", image="", bg="#ffcccc")
        else:
            self.content_label.config(text=f"[IMAGE NOT FOUND]\n\nExpected at: {image_dir}/{img_name}.jpg", image="", bg="#e6f7ff", fg="#0050b3")
            self.status_label.config(text=f"Image not found: {img_name}")

    def clear_screen(self):
        self.content_label.config(text="", bg="#fff")
        self.status_label.config(text="Screen cleared")

    def on_close(self):
        rospy.signal_shutdown("UI Closed")
        self.root.destroy()

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    try:
        ui = TabletUI()
        ui.run()
    except rospy.ROSInterruptException:
        pass
