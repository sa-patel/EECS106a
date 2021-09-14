#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

import sys
import rospy
from geometry_msgs.msg import Twist, Vector3

# Define the method which contains the node's main functionality
def talker():
	turtle_name = sys.argv[1]
	valid_keystrokes = ['w', 'a', 's', 'd']

	# Create an instance of the rospy.Publisher object which we can  use to
	# publish messages to a topic. This publisher publishes messages of type
	# geometry_msgs/Twist to the topic /turtle_name/cmd_vel
	pub = rospy.Publisher('%s/cmd_vel' % (turtle_name), Twist, queue_size=10)

	# Loop until the node is killed with Ctrl-C
	while not rospy.is_shutdown():
		keystroke = raw_input("Enter turtle movement (WASD): ").lower()
		while keystroke not in valid_keystrokes:
			keystroke = raw_input("Not a valid keystroke. Please use WASD: ")

		# Construct a string that we want to publish (in Python, the "%"
		# operator functions similarly to sprintf in C or MATLAB)
		# pub_string = "hello world %s" % (rospy.get_time())
		
		lin = [0.0, 0.0, 0.0]
		ang = [0.0, 0.0, 0.0]
		if keystroke == 'w':
			lin[0] = 2.0
		elif keystroke == 'a':
			ang[2] = 2.0
		elif keystroke == 's':
			lin[0] = -2.0
		elif keystroke == 'd':
			ang[2] = -2.0
		
		lin = Vector3(lin[0], lin[1], lin[2])
		ang = Vector3(ang[0], ang[1], ang[2])
		velocity = Twist(lin, ang)
		pub.publish(velocity)
		print(velocity)
			
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

	# Run this program as a new node in the ROS computation graph called /talker.
	rospy.init_node('talker', anonymous=True)

	# Check if the node has received a signal to shut down. If not, run the
	# talker method.
	try:
		talker()
	except rospy.ROSInterruptException: pass
