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
import roslaunch


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensoring.srv import DetectImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from knowledge.srv import OracleReq
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

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

## 2D person position
person_pos = Point(rospy.get_param("person_pos_x"),rospy.get_param("person_pos_y"),0)

## Min delay for SLEEP state
min_sleep_delay = rospy.get_param("min_sleep_delay")

## Max delay for SLEEP state
max_sleep_delay = rospy.get_param("max_sleep_delay")

## x coordinate of the map
map_x = rospy.get_param("map_x")
## y coordinate of the map
map_y = rospy.get_param("map_y")

# define state Play
class play(smach.State):

    def __init__(self):
        # initialisation function, it should not wait

        """
            Constrcutor. It inizializes the attribute
        """
        smach.State.__init__(self,outcomes=['someTimes','unknow'],output_keys=['ballTypePlay'])     

    def ask_oracle(self,request):

        rospy.wait_for_service('oracle_req')
        try:
            target_pos = rospy.ServiceProxy('oracle_req', OracleReq)
            resp = target_pos(request)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)


    def getCommand(self,data):

        """
            Callback method that received the "play" command and set the attribute
            play to 1
            @param data: command message
            @type data: str
        """
        location = data.data.split('+')[1]
        resp = self.ask_oracle("isVisited "+location)
        resp = resp.location.split()
        self.ball_type = resp[1]
        if(resp[0] == "True"):
            resp = self.ask_oracle("getPos "+self.ball_type)
            position = resp.location.split()
            self.target_pos_client(float(position[0]),float(position[1]))
            rospy.loginfo("Robot arrived in "+location)
            time.sleep(random.uniform(min_sleep_delay,max_sleep_delay))
            self.target_pos_client(person_pos.x,person_pos.y)
            rospy.loginfo("Robot arrived in person position")
        else:
            self.goTo = True           
        self.count += 1
        if self.count == self.transition_value:
            self.transition = 1


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
        goal.target_pose.pose.orientation.w = -1.57

        # Sends the goal to the action server.
        client.send_goal(goal)

        client.wait_for_result()

        return client.get_result()

    def execute(self, userdata):

        """
            It publishes the person position and, after the robot reaches the position, it
            subsribes to "gesture" topic. At the end, it waits for the state transition
            @param userdata: used to pass data between states
            @type userdata: list
        """
        self.count = 0
        self.transition_value = random.randint(min_transition_play_normal,max_transition_play_normal)
        self.transition = 0
        self.goTo = False

        # function called when exiting from the node, it can be blacking      
        rospy.loginfo('Executing state PLAY')
        self.target_pos_client(person_pos.x,person_pos.y)
        rospy.loginfo("Robot arrived in person position")
        sub_command = rospy.Subscriber("command", String, self.getCommand)
        while self.transition == 0 and self.goTo == False:
            pass
        sub_command.unregister()
        if self.goTo == True:
            userdata.ballTypePlay = self.ball_type
            return 'unknow'
        else:
            return 'someTimes'

# define state Find
class find(smach.State):

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

        smach.State.__init__(self,outcomes=['someTimes','ball',],input_keys=['ballTypeFromPlay'],output_keys=['ballType'])

    def ask_oracle(self,request):

        rospy.wait_for_service('oracle_req')
        try:
            target_pos = rospy.ServiceProxy('oracle_req', OracleReq)
            resp = target_pos(request)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)

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
        rospy.loginfo('Executing state FIND')

        count_value = random.randint(min_transition_normal_sleep,max_transition_normal_sleep)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/simone/catkin_ws/src/Behavioral-Architecture-with-Real-Simulation-AutonomousNav/explore/launch/explore.launch"])
        launch.start()


        while True:
            resp = self.object_detector_client()
            ball = resp.object.split()
            ball_type = ball[0]
            center = float(ball[1])
            radius = float(ball[2])
            if ((center != -1) and (radius != -1)):
                resp = self.ask_oracle("prevDetect "+ball_type)
                if(resp.location == "False"):
                    launch.shutdown()
                    break

        userdata.ballType = userdata.ballTypeFromPlay
        return 'ball'

        #os.system("rosnode kill explore")
        #return 'someTimes'
                     
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
        goal.target_pose.pose.orientation.w = -1.57

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

        self.target_pos_client(home_pos.x,home_pos.y)
        rospy.loginfo("Robot arrived in home position")
        time.sleep(random.uniform(min_sleep_delay,max_sleep_delay))
        
        return 'wakeUp'

# define state Track
class track(smach.State):

    def __init__(self):

        """
            Constrcutor
        """

        smach.State.__init__(self,outcomes=['reached','reachedPlay','notRequest'],output_keys=['toNormal'],input_keys=['ballType'])
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.clbk_odom)    

    def clbk_odom(self,msg):

        '''
            Callback function to obtain the odometry information
            from the robot

            @param msg: robot state
            @type msg: Odometry
        '''

        # position
        self.position = msg.pose.pose.position

    def clbk_laser(self,msg):
        if(msg.ranges[0]< 0.5):
            self.vel.linear.z = -0.001
            #self.from_clbk = 1
        elif(msg.ranges[719]<0.5):
           self.vel.linear.z = 0.001
           #self.from_clbk = 1
        #else:
            #self.from_clbk = 0
            
    def ask_oracle(self,request):

        rospy.wait_for_service('oracle_req')
        try:
            target_pos = rospy.ServiceProxy('oracle_req', OracleReq)
            resp = target_pos(request)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)

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

        #self.from_clbk = 0
        self.vel = Twist()
        self.sub_scan = rospy.Subscriber('scan', LaserScan, self.clbk_laser)

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

            #if(self.from_clbk == 0):
            if((radius == -1) and (center == -1)):
                userdata.toNormal = False
                self.sub_scan.unregister()
                return 'reached'
            elif(radius > 10):
                self.vel.angular.z = -0.003*(center-400)
                self.vel.linear.x = -0.007*(radius-110)
            elif(radius < 10):
                self.vel.angular.z = 0
                self.vel.linear.x = 0.09
            
            self.vel_pub.publish(self.vel)
            
                   
        self.ask_oracle("setPos "+ball_type+" "+str(self.position.x)+" "+str(self.position.y))
        self.sub_scan.unregister()
        rospy.loginfo(userdata.ballType)
        if(userdata.ballType != None):
            if(userdata.ballType == ball_type):
                return 'reachedPlay'
            else:
                return 'notRequest'
        else:
            userdata.toNormal = True
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

        smach.State.__init__(self,outcomes=['someTimes','ball','play'],input_keys=['fromTrack'])
        rospy.Subscriber("command", String, self.getCommand)

    def ask_oracle(self,request):

        rospy.wait_for_service('oracle_req')
        try:
            target_pos = rospy.ServiceProxy('oracle_req', OracleReq)
            resp = target_pos(request)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)

    def getCommand(self,data):

        """
            Callback method that received the "play" command and set the attribute
            play to 1
            @param data: command message
            @type data: str
        """

        self.play = 1

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
            if self.play == 1:
                client.cancel_all_goals()
                return 'play'
            if(client.get_state()== 4):
                rospy.loginfo("Position unreachable")
                client.cancel_all_goals()
                return None
            resp = self.object_detector_client()
            ball = resp.object.split()
            ball_type = ball[0]
            center = float(ball[1])
            radius = float(ball[2])
            if(((center == -1) and (radius == -1))):
                self.from_track = False
            elif((self.from_track == False) and ((center != -1) and (radius != -1))):
                resp = self.ask_oracle("prevDetect "+ball_type)
                if(resp.location == "False"):
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

        self.play = 0

        self.from_track = userdata.fromTrack

        for count in range(0,count_value):
            if self.play == 1:
                return 'play'
            if(self.from_track == False):
                resp = self.object_detector_client()
                ball = resp.object.split()
                ball_type = ball[0]
                center = float(ball[1])
                radius = float(ball[2])
                if ((center != -1) and (radius != -1)):
                    resp = self.ask_oracle("prevDetect "+ball_type)
                    if(resp.location == "False"):
                        return 'ball'
            x = random.uniform(neg_map_x,map_x)
            y = random.uniform(neg_map_y,map_y)
            result = self.target_pos_client(x,y)
            if (result != None):
                if(result == "play"):
                    return 'play'
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
                                            'ball':'TRACK',
                                            'play':'PLAY'},
                                            remapping={'fromTrack':'sm_data'})
        smach.StateMachine.add('SLEEP', sleep(), 
                               transitions={'wakeUp':'NORMAL'})
        smach.StateMachine.add('PLAY', play(), 
                               transitions={'someTimes':'NORMAL',
                                            'unknow':'FIND'},
                                            remapping={'ballTypePlay':'sm_data',
                                                       'fromTrack':'sm_data'})
        smach.StateMachine.add('TRACK', track(), 
                               transitions={'reached':'NORMAL',
                                            'reachedPlay':'PLAY',
                                            'notRequest':'FIND'},
                               remapping={'toNormal':'sm_data',
                                          'ballType':'sm_data'})
        smach.StateMachine.add('FIND', find(), 
                               transitions={'someTimes':'PLAY',
                                            'ball':'TRACK'},
                               remapping={'ballType':'sm_data',
                                          'ballTypeFromPlay':'sm_data'})
        
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