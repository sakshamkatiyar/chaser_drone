#!/usr/bin/env python
import time
import rospy
import roslib
from std_msgs.msg import Int16

import sys, select, termios, tty

msg = """
Control Your Drone!
---------------------------
Moving around:

a : Arm drone
d : Dis-arm drone
r : stop smoothly
w : increase height
s : increase height

CTRL-C to quit
"""

flag = 0
start = time.time()

"""
Function Name: getKey
Input: None
Output: keyboard charecter pressed
Logic: Determine the keyboard key pressed
Example call: getkey()
"""
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
	if flag == 1:
		key = 's'
	else:
		key = 'd'

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    global flag
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('stabilize_drone')
    pub = rospy.Publisher('/input_key',Int16, queue_size=5) #publish the key pressed
    rate=rospy.Rate(8)
    msg_pub = 0
    keyboard_control={  #dictionary containing the key pressed and value associated with it
                      'd':0,
                      's':20,
                      'a':10,
                      'w':30}

    try:
        while not rospy.is_shutdown():
            key = getKey()
	    if time.time() - start >= 2 and flag == 0:
		key = 'a'
		flag = 1
            if key in keyboard_control.keys():
                msg_pub=keyboard_control[key]
            pub.publish(msg_pub)
            rate.sleep()
    except Exception as e:
        print e
    finally:
        print key
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
