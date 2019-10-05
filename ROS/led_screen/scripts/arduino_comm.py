#!/usr/bin/env python
import time
import threading
import rospy
from osr_msgs.msg import Status,Led
from std_msgs.msg import Int64MultiArray
from screen import LedScreen

global battery
global error_status
global temp
global current
global connected
global face

global have_received_status

screen = LedScreen()
screen_mutex = threading.Lock()

'''
Build a message and send to Arduino. May be called from multiple locations
depending on what information was updated.
'''
def send_screen_message():
	with screen_mutex:
		screen.build_msg(	
							connected,
							battery,
							error_status,
							temp,
							current,
							face
						)
		
		screen.check_for_afffirm()

'''
Update screen to reflect most recent system status message plus latest
'connected' status.
'''
def status_callback(data):
	global battery
	global error_status
	global temp
	global current
	global connected
	global have_received_status

	have_received_status = True
	battery = data.battery
	error_status = data.error_status
	temp = data.temp
	current = data.current
	connected = int(rospy.get_param("remote_control/connected"))

	send_screen_message()

'''
When the face changes, update screen along with latest 'connected'.
'''
def led_cmds_callback(data):
	global face
	global connected

	if have_received_status and data.data[0] != face:
		face = data.data[0]
		connected = int(rospy.get_param("remote_control/connected"))
		send_screen_message()

def shutdown():
	with screen_mutex:
		screen.transistion_screen_to_idle()

if __name__ == "__main__":

	rospy.init_node("led_screen")
	rospy.loginfo("Starting the led_screen node")
	rospy.on_shutdown(shutdown)

	have_received_status = False
	face = int(rospy.get_param("face/value"))
	connected = int(rospy.get_param("remote_control/connected"))

	sub = rospy.Subscriber("/status", Status, status_callback)
	sub = rospy.Subscriber("/led_cmds", Int64MultiArray, led_cmds_callback)
	rospy.spin()
