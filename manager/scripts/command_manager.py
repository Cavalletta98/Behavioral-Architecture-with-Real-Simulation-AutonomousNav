#!/usr/bin/env python3

"""
    ROS node that implement the FSM of 
    robot behaviours
"""

# Import of libraries
import roslib
import rospy
import smach
import smach_ros
import time
import random
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensoring.srv import DetectImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

## Min delay for transition between NORMAL and SLEEP states
min_transition_normal_sleep = rospy.get_param("min_transition_normal_sleep")

## Max delay for transition between NORMAL and SLEEP states
max_transition_normal_sleep = rospy.get_param("max_transition_normal_sleep")

## Min delay for transition between NORMAL and SLEEP states
min_transition_play_normal = rospy.get_param("min_transition_play_normal")

## Max delay for transition between NORMAL and SLEEP states
max_transition_play_normal = rospy.get_param("max_transition_play_normal")

## 2D home position
home_pos = Point(rospy.get_param("home_pos_x"),rospy.get_param("home_pos_y"),0)

## Min delay for SLEEP state
min_sleep_delay = rospy.get_param("min_sleep_delay")

## Max delay for SLEEP state
max_sleep_delay = rospy.get_param("max_sleep_delay")

## Min delay for mantaining 45° clockwise the head
min_clock_head_delay = rospy.get_param("min_clock_head_delay")

## Max delay for mantaining 45° clockwise the head
max_clock_head_delay = rospy.get_param("max_clock_head_delay")

## Min delay for mantaining 45° counterclockwise the head
min_countclock_head_delay = rospy.get_param("min_countclock_head_delay")

## Max delay for mantaining 45° counterclockwise the head
max_countclock_head_delay = rospy.get_param("max_countclock_head_delay")

## Min delay for mantaining 0° the head
min_head_delay = rospy.get_param("min_head_delay")

## Max delay for mantaining 0° the head
max_head_delay = rospy.get_param("max_head_delay")

## x coordinate of the map
map_x = rospy.get_param("map_x")
## y coordinate of the map
map_y = rospy.get_param("map_y")
                     
# define state Sleep
class sleep(smach.State):

    """
        A class used to represent the SLEEP behaviour
        of the robot

        Methods
        -----
        target_pos_client(x, y):
            Send a goal to the action server of the robot and waits until it reaches the goal.
        execute(userdata)
            It send the robot to the home position and, after the robot reaches the position,
            sleeps for a random time of seconds. After that change the state to NORMAL
    """

    def __init__(self):

        """
            Constrcutor
        """

        # initialisation function, it should not wait
        smach.State.__init__(self,outcomes=['wakeUp'])

    def target_pos_client(self,x, y):

        """
            Send a goal to the action server of the robot and waits until it reaches the goal.

            @param x: x coordinate of the target position
            @type x: int
            @param y: y coordinate of the target position
            @type y: int

            @returns: the position reached by the robot
            @rtype: Pose

        """
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        # Sends the goal to the action server.
        client.send_goal(goal)

        client.wait_for_result()

        return client.get_result()
        
    def execute(self, userdata):

        """
            It send the robot to the home position and, after the robot reaches the position,
            sleeps for a random time of seconds. After that change the state to NORMAL

            @param userdata: used to pass data between states
            @type userdata: list

            @returns: transition value
            @rtype: String
        """

        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state SLEEP')

        result = self.target_pos_client(home_pos.x,home_pos.y)
        rospy.loginfo(result)
        time.sleep(random.uniform(min_sleep_delay,max_sleep_delay))
        
        return 'wakeUp'

# define state Track
class track(smach.State):

    def __init__(self):

        """
            Constrcutor
        """

        smach.State.__init__(self,outcomes=['reached'],output_keys=['toNormal'])
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        sub_odom = rospy.Subscriber('odom', Odometry, self.clbk_odom)

    def clbk_odom(self,msg):

        '''
            Callback function to obtain the odometry information
            from the robot

            @param msg: robot state
            @type msg: Odometry
        '''

        # position
        self.position = msg.pose.pose.position

    def object_detector_client(self):

        """
            Makes a request to detector server and wait for the response

            @returns: radius and center of the ball
            @rtyper: string

        """

        rospy.wait_for_service('detect_image')
        try:
            detect_obj_serv = rospy.ServiceProxy('detect_image', DetectImage)
            resp = detect_obj_serv()
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)

    def execute(self, userdata):

        """
            It checks if there is the ball, otherwise it will generate a random goal (x and y)
            for the robot. If there is the ball, it switches to the PLAY state.
            After some times, it switches to the SLEEP state

            @param userdata: used to pass data bejoint_state = tween states
            @type userdata: list

            @returns: transition value
            @rtype: String
        """
        rospy.loginfo('Executing state TRACK')

        resp = self.object_detector_client()
        ball = resp.object.split()
        ball_type = ball[0]
        center = float(ball[1])
        radius = float(ball[2])
        
        while(radius <= 100):

            resp = self.object_detector_client()
            ball = resp.object.split()
            ball_type = ball[0]
            center = float(ball[1])
            radius = float(ball[2])

            if((radius == -1) and (center == -1)):
                userdata.toNormal = False
                return 'reached'
            elif(radius > 10):
                vel = Twist()
                vel.angular.z = -0.003*(center-400)
                vel.linear.x = -0.007*(radius-110)
                self.vel_pub.publish(vel)
            elif(radius < 10):
                vel = Twist()
                vel.linear.x = 0.09
                self.vel_pub.publish(vel)
            
                 
        userdata.toNormal = True
        rospy.loginfo(self.position)
        return 'reached'
    

# define state Normal
class normal(smach.State):

    """
        A class used to represent the NORMAL behaviour
        of the robot

        Methods
        -----
        object_detector_client():
            Makes a request to detector server and wait for the response
        target_pos_client(x,y)
            Send a goal to the action server of the robot and waits until it reaches the goal.
            While it is waiting, if there is the ball, it stops the robot and return None
        execute(userdata)
            It checks if there is the ball, otherwise it will generate a random goal (x and y)
            for the robot. If there is the ball, it switches to the PLAY state.
            After some times, it switches to the SLEEP state
    """

    def __init__(self):

        """
            Constrcutor
        """

        smach.State.__init__(self,outcomes=['someTimes','ball'],input_keys=['fromTrack'])

    def object_detector_client(self):

        """
            Makes a request to detector server and wait for the response

            @returns: radius and center of the ball
            @rtyper: string

        """

        rospy.wait_for_service('detect_image')
        try:
            detect_obj_serv = rospy.ServiceProxy('detect_image', DetectImage)
            resp = detect_obj_serv()
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)
    
    def target_pos_client(self,x, y):

        """
            Send a goal to the action server of the robot and waits until it reaches the goal.
            While it is waiting, if there is the ball, it stops the robot and return None

            @param x: x coordinate of the target position
            @type x: int
            @param y: y coordinate of the target position
            @type y: int

            @returns: the position reached by the robot or None
            @rtype: Pose

        """
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        # Sends the goal to the action server.
        client.send_goal(goal)

        #client.wait_for_result()

        while(client.get_state() != 3):
            resp = self.object_detector_client()
            ball = resp.object.split()
            center = float(ball[1])
            radius = float(ball[2])
            if(((center == -1) and (radius == -1))):
                self.from_track = False
            elif((self.from_track == False) and ((center != -1) and (radius != -1))):
                client.cancel_all_goals()
                return None

        return client.get_result()

    def execute(self, userdata):

        """
            It checks if there is the ball, otherwise it will generate a random goal (x and y)
            for the robot. If there is the ball, it switches to the PLAY state.
            After some times, it switches to the SLEEP state

            @param userdata: used to pass data bejoint_state = tween states
            @type userdata: list

            @returns: transition value
            @rtype: String
        """
        rospy.loginfo('Executing state NORMAL')

        count_value = random.randint(min_transition_normal_sleep,max_transition_normal_sleep)
        neg_map_x = map_x*-1
        neg_map_y = map_y*-1

        self.from_track = userdata.fromTrack

        for count in range(0,count_value):
            if(self.from_track == False):
                resp = self.object_detector_client()
                ball = resp.object.split()
                center = float(ball[1])
                radius = float(ball[2])
                if ((center != -1) and (radius != -1)):
                    return 'ball'
            x = random.uniform(neg_map_x,map_x)
            y = random.uniform(neg_map_y,map_y)
            result = self.target_pos_client(x,y)
            if (result != None):
                rospy.loginfo("Robot arrived in (%lf,%lf)",x,y)
            else:
                return 'ball'

        return 'someTimes'

        
def main():

    """
        Main function that initializes the node and the FSM.
        After that it starts the node and the FSM
    """

    rospy.init_node('command_manager_state_machine')
     
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_data = False

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', normal(), 
                               transitions={'someTimes':'SLEEP',
                                            'ball':'TRACK'},
                                            remapping={'fromTrack':'sm_data'})
        smach.StateMachine.add('SLEEP', sleep(), 
                               transitions={'wakeUp':'NORMAL'})
        smach.StateMachine.add('TRACK', track(), 
                               transitions={'reached':'NORMAL'},
                               remapping={'toNormal':'sm_data'})
        
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()