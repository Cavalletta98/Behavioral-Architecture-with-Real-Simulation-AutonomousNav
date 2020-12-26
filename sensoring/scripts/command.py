#!/usr/bin/env python3

"""ROS node used to generate user command"""

# Import of libraries
import rospy
from std_msgs.msg import String

def command_generator():

    """
        Main function that generates user command
        and publishes it on topic "command"
    """

    pub = rospy.Publisher('command', String, queue_size=1)
    rospy.init_node('command_node', anonymous=True)

    while not rospy.is_shutdown():
        command = input('Enter the command: ')
        pub.publish(command)

if __name__ == '__main__':
    try:
        command_generator()
    except rospy.ROSInterruptException:
        pass