#!/usr/bin/env python

import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rosbag
from datetime import datetime

class RosbagExtractorGUI:
    def __init__(self, root):
        self.root = root
        root.title("ROS Bag Image Extractor")

        # Bag file selection
        tk.Label(root, text="Bag File:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.bag_file_entry = tk.Entry(root)
        self.bag_file_entry.grid(row=0, column=1, sticky=tk.EW, padx=5)
        tk.Button(root, text="Browse...", command=self.browse_bag_file).grid(row=0, column=2, padx=5)

        # Destination folder selection
        tk.Label(root, text="Destination Folder:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.destination_entry = tk.Entry(root)
        self.destination_entry.grid(row=1, column=1, sticky=tk.EW, padx=5)
        tk.Button(root, text="Browse...", command=self.browse_destination).grid(row=1, column=2, padx=5)

        # Topics input
        tk.Label(root, text="Topics (comma-separated):").grid(row=2, column=0, sticky=tk.NW, padx=5, pady=5)
        self.topics_text = scrolledtext.ScrolledText(root, height=4, width=40)
        self.topics_text.grid(row=2, column=1, sticky=tk.EW, padx=5, pady=5)

        # Start extraction button
        tk.Button(root, text="Start Extraction", command=self.extract_images).grid(row=3, column=0, columnspan=3, pady=10)

        # Logging area
        tk.Label(root, text="Log:").grid(row=4, column=0, sticky=tk.NW, padx=5, pady=5)
        self.log_text = scrolledtext.ScrolledText(root, height=10, width=50)
        self.log_text.grid(row=5, column=0, columnspan=3, sticky=tk.EW, padx=5, pady=5)
        self.log_text.config(state=tk.DISABLED)  # Disable editing of log area

        self.bridge = CvBridge()
        root.columnconfigure(1, weight=1)

    def log_message(self, message):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.config(state=tk.DISABLED)
        self.log_text.see(tk.END)  # Scroll to the bottom

    def browse_bag_file(self):
        filename = filedialog.askopenfilename(title="Select ROS Bag File", filetypes=(("ROS Bag files", "*.bag"), ("All files", "*.*")))
        self.bag_file_entry.delete(0, tk.END)
        self.bag_file_entry.insert(0, filename)

    def browse_destination(self):
        foldername = filedialog.askdirectory(title="Select Destination Folder")
        self.destination_entry.delete(0, tk.END)
        self.destination_entry.insert(0, foldername)

    def extract_images(self):
        bag_file = self.bag_file_entry.get()
        destination = self.destination_entry.get()
        topics = self.topics_text.get("1.0", tk.END).strip()

        if not topics:
            messagebox.showerror("Error", "No topics entered.")
            return

        self.log_message("Starting image extraction...")
        topics_list = [topic.strip() for topic in topics.split(',')]
        for topic in topics_list:
            self.process_topic(bag_file, topic, destination)

    def process_topic(self, bag_file, topic, output_base):
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        output_dir = os.path.join(output_base, f"{topic.replace('/', '_')}_{timestamp}")
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            self.log_message(f"Created directory: {output_dir}")

        with rosbag.Bag(bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[topic]):
                try:
                    if msg._type == 'sensor_msgs/CompressedImage':
                        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    else:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    
                    frame_id = msg.header.seq if msg.header.seq else t.to_nsec()
                    image_filename = os.path.join(output_dir, f"frame_{frame_id}.png")
                    cv2.imwrite(image_filename, cv_image)
                    self.log_message(f"Saved: {image_filename}")
                except CvBridgeError as e:
                    self.log_message(f"CV Bridge error: {e}")
                except Exception as e:
                    self.log_message(f"Error processing message: {e}")
        self.log_message(f"Completed extraction for topic: {topic}")

if __name__ == '__main__':
    root = tk.Tk()
    app = RosbagExtractorGUI(root)
    root.mainloop()
