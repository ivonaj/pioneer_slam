#!/usr/bin/env python
import rosbag
import rospy
import sys
import os

from sensor_msgs.msg import LaserScan

if len(sys.argv) < 3:
  print "Fixes up a bag in case laser driver was mistakenly configured to be flipped."
  print "Usage: {} <bag_name> <laser topic>".format(os.path.basename(sys.argv[0]))
  exit()

input_bag_name = sys.argv[1]
topic_name = sys.argv[2]

try:
  input_bag = rosbag.Bag(input_bag_name)
  bag_loaded = True
except Exception:
  print "Could not open bag {}".format(input_bag_name)
  bag_loaded = False

if bag_loaded:
  name, ext = os.path.splitext(input_bag_name)
  output_bag_name = "{name}_swapped{ext}".format(name=name, ext=ext)
  output_bag = rosbag.Bag(output_bag_name, 'w')
  corrected = False

  for topic, msg, t in input_bag.read_messages():
    if topic == topic_name:
      msg.angle_max, msg.angle_min = msg.angle_min, msg.angle_max
      msg.angle_increment *= -1
      corrected = True
    output_bag.write(topic, msg, t)

  output_bag.close()
  input_bag.close()

  if not corrected: print "Warning: topic not found, bag has been duplicated"
