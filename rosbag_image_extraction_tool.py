#!/usr/bin/env python

import os
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rosbag
from datetime import datetime

def extract_images(bag_file, topic, output_base):
    # Create output directory with current date and time
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    output_dir = os.path.join(output_base, f"{topic.replace('/', '_')}_{timestamp}")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Initialize CV bridge
    bridge = CvBridge()

    # Process bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic]):
            try:
                if msg._type == 'sensor_msgs/CompressedImage':
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
                else:
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
                frame_id = msg.header.seq if msg.header.seq else t.to_nsec()
                image_filename = os.path.join(output_dir, f"frame_{frame_id}.png")
                cv2.imwrite(image_filename, cv_image)
                print(f"Saved {image_filename}")
            except CvBridgeError as e:
                print(f"CV Bridge error: {e}")
            except Exception as e:
                print(f"Error processing message: {e}")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python extract_images.py <bag_file> <topic> <output_base>")
        sys.exit(1)
    
    bag_file = sys.argv[1]
    topic = sys.argv[2]
    output_base = sys.argv[3]
    
    extract_images(bag_file, topic, output_base)

