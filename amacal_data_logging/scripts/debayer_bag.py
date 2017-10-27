#!/usr/bin/env python
import rosbag
import rospy
import sys
import os

from sensor_msgs.msg import Image

def receive_image(received_image):
  global is_image_received; is_image_received = True
  global image; image = received_image

def get_image():
  global is_image_received;
  while(not is_image_received):
    pass
  is_image_received = False
  return image

if len(sys.argv) < 3:
  print "Usage: {} <bag_name> <image_topic>".format(os.path.basename(sys.argv[0]))
  print "roscore and a debayer nodelet have to be running"
  print ""
  print "To run a debayer nodelet, run:"
  print "rosrun nodelet nodelet manager __name:=nodelet_manager &"
  print "rosrun nodelet nodelet load image_proc/debayer nodelet_manager _debayer:=3"
  exit()

input_bag_name = sys.argv[1]
is_image_received = False

image_topic_name = sys.argv[2]

rospy.init_node('debayer_bag', anonymous=True)

pub = rospy.Publisher('image_raw', Image, queue_size=10)

rospy.Subscriber("image_mono", Image, receive_image)

try:
  input_bag = rosbag.Bag(input_bag_name)
  is_bag_loaded = True
except Exception:
  print "Could not open bag {}".format(input_bag_name)
  is_bag_loaded = False

if is_bag_loaded:
  name, ext = os.path.splitext(input_bag_name)
  output_bag_name = "{name}_debayered{ext}".format(name=name, ext=ext)

  is_output_bag_loaded = False
  if os.path.exists(output_bag_name):
    print('Error: {} already exists'.format(output_bag_name))
  else:
    try:
      output_bag = rosbag.Bag(output_bag_name, 'w')
      print "Writing to bag {}".format(output_bag_name)
      is_output_bag_loaded = True
    except Exception:
      print "Could not open bag {} for writing".format(output_bag_name)

  if is_output_bag_loaded:
    is_requested_topic_detected = False
    for topic, msg, t in input_bag.read_messages():
      if topic == image_topic_name:
        is_requested_topic_detected = True
        pub.publish(msg)
        mono_image = get_image()
        output_bag.write(topic, mono_image, t)
      else:
        output_bag.write(topic, msg, t)

    output_bag.close()

    if not is_requested_topic_detected:
      print "Warning: the specified image topic has not been detected in the bag! The result is a duplicate bag."

  input_bag.close()
