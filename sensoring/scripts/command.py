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
    rate = rospy.Rate(5) # 5hz

    while not rospy.is_shutdown():
        pub.publish("play")
        rospy.sleep(random.uniform(1,10))
        room = rooms[random.randint(0,5)]
        pub.publish("goTo+"+room)
        rate.sleep()


if __name__ == '__main__':
    try:
        command_generator()
    except rospy.ROSInterruptException:
        pass