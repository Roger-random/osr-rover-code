#!/usr/bin/env python
'''
ROS module to control Open Source Rover LED matrix face
using direction pad (d-pad) on Xbox controller.
'''
import rospy
from sensor_msgs.msg import Joy
from osr_msgs.msg import Led
from std_msgs.msg import Int64MultiArray
import math

# Once we react to a direction, we wait for finger release before 
# processing more commands
global center_wait
center_wait = False

# Rover electrical status LEDs can be toggled on and off
global display_status
display_status = False

# Current index of face to be rendered. This is an eight-bit number and we're
# and we're using highest bit for status display toggle, so we can have up to
# 127 faces.
global face_index
face_index = 0

# Direction pad is typicall digital (-1, 0, 1) but reported as a floating
# point number. In case an analog d-pad is used, compare against threshold.
threshold = 0.75

'''
Called when there's a message on the /joy topic.
'''

def joy_callback(data):
  global center_wait
  global display_status
  global face_index

  # Reference: http://wiki.ros.org/joy
  x = data.axes[6]
  y = data.axes[7]

  if center_wait:
    if abs(x) < threshold and abs(y) < threshold:
      # Have returned to center and ready for next command
      center_wait = False
  else:
    if y > threshold and face_index < 256:
      # Next face
      face_index += 1
      center_wait = True
    elif y < -threshold and face_index > 0:
      # Previous face
      face_index -= 1
      center_wait = True
    elif x > threshold:
      # Toggle status bar
      display_status = not display_status
      center_wait = True

  command = face_index

  if display_status:
    command = command | 0x80

  led_msg = Int64MultiArray()
  led_msg.data = [command,1]

  led_pub.publish(led_msg)

'''
Set up ROS topic subsription and publishing channels
'''

if __name__ == '__main__':
  rospy.init_node('dpad_face')
  rospy.loginfo('dpad_face started')

  sub = rospy.Subscriber("/joy", Joy, joy_callback)
  led_pub = rospy.Publisher('led_cmds', Int64MultiArray, queue_size=1)

  rospy.spin()
