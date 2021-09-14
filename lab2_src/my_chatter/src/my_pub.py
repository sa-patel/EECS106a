#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Import the rospy package. For an import to work, it must be specified
# in both the package manifest AND the Python file in which it is used.
import rospy
from my_chatter.msg import TimestampString

# Define the method which contains the node's main functionality
def talker():

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # TimestampString to the topic /user_messages
    pub = rospy.Publisher('user_messages', TimestampString, queue_size=10)

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
		user_input = raw_input("Please enter a line of text and press <Enter>: ")
        
		# Publish our string to the 'user_messages' topic
		pub.publish(user_input, rospy.get_time())
            
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