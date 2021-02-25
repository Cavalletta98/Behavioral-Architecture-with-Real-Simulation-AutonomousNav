#!/usr/bin/env python3

"""ROS node used to generate user command"""

# Import of libraries
import rospy
from std_msgs.msg import String
import random

def command_generator():

    """
        Main function that generates user command
        and publishes it on topic "command"
    """
    rooms = ["entrance","closet","living_room","kitchen","bathroom","bedroom"]
    pub = rospy.Publisher('command', String, queue_size=1)
    rospy.init_node('command_node', anonymous=True)
    rate = rospy.Rate(rospy.get_param("freq_command"))

    while not rospy.is_shutdown():
        pub.publish("play")
        rospy.sleep(random.uniform(rospy.get_param("min_delay_command"),rospy.get_param("max_delay_command")))
        room = rooms[random.randint(0,5)]
        pub.publish("goTo+"+room)
        rate.sleep()


if __name__ == '__main__':
    try:
        command_generator()
    except rospy.ROSInterruptException:
        pass