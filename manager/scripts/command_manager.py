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

## Path of the explore lite launch file
path_exp = rospy.get_param("path_exp")

## Min delay for transition between NORMAL and SLEEP states
min_transition_normal_sleep = rospy.get_param("min_transition_normal_sleep")

## Max delay for transition between NORMAL and SLEEP states
max_transition_normal_sleep = rospy.get_param("max_transition_normal_sleep")

## Min delay for transition between FIND and PLAY states
min_transition_find_play = rospy.get_param("min_transition_find_play")

## Max delay for transition between FIND and PLAY states
max_transition_find_play = rospy.get_param("max_transition_find_play")

## Min delay for transition between PLAY and NORMAL states
min_transition_play_normal = rospy.get_param("min_transition_play_normal")

## Max delay for transition between PLAY and NORMAL states
max_transition_play_normal = rospy.get_param("max_transition_play_normal")

## 2D home position
home_pos = Point(rospy.get_param("home_pos_x"),rospy.get_param("home_pos_y"),0)

## 2D person position
person_pos = Point(rospy.get_param("person_pos_x"),rospy.get_param("person_pos_y"),0)

## Min delay for sleeping
min_sleep_delay = rospy.get_param("min_sleep_delay")

## Max delay for sleeping
max_sleep_delay = rospy.get_param("max_sleep_delay")

## x coordinate of the map
map_x = rospy.get_param("map_x")
## y coordinate of the map
map_y = rospy.get_param("map_y")

# define state Play
class play(smach.State):

    """
        A class used to represent the PLAY behaviour
        of the robot

        Attributes
        -----
        @param count: Variable that represents how many times we perform the PLAY behavior
        @type count: int

        @param transition_value: Variable that represents the value used to change state to Normal
        @type transition_value: int

        @param transition: Variable used to change state to Normal
        @type transition: bool

        @param goTo: Variable used to change state to FIND
        @type goTo: bool

        Methods
        -----
        ask_oracle(request):
            Method used to ask a specific request to the oracle
        getCommand(data)
            Callback method that received the "goTo" command and check if the location is already visited or not
            If it is already visited, send the robot to the requested room.
            If it is not visited, set the variable goTo to True in order to change state.
            When the count is equal to the transition_value, the variable transition is setted to True
        target_pos_client(x, y)
            Send a goal to the action server of the robot and waits until it reaches the goal.
            While it is waiting, if the plan is aborted,it returns None; otherwise it will return
            the result of the sended goal 
        execute(userdata)
            It sends the robot to the person. When the robot is there, waits for a goTo command.
            After some times it changes state to Normal
    """

    def __init__(self):
        # initialisation function, it should not wait

        """
            Constrcutor
        """
        smach.State.__init__(self,outcomes=['someTimes','unknow'],output_keys=['ballTypePlay'])
        ## Variable that represents how many times we perform the PLAY behavior
        self.count = 0
        ## Variable that represents the value used to change state to Normal
        self.transition_value = random.randint(min_transition_play_normal,max_transition_play_normal)

    def ask_oracle(self,request):

        """
            Method used to ask a specific request to the oracle

            @param request: request message
            @type request: OracleReq

            @returns: response to the request
            @rtype: OracleReqResponse
        """

        rospy.wait_for_service('oracle_req')
        try:
            target_pos = rospy.ServiceProxy('oracle_req', OracleReq)
            resp = target_pos(request)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s",e)


    def getCommand(self,data):

        """
            Callback method that received the "goTo" command and check if the location is already visited or not
            If it is already visited, send the robot to the requested room.
            If it is not visited, set the variable goTo to True in order to change state.
            When the count is equal to the transition_value, the variable transition is setted to True

            @param data: command message
            @type data: str
        """
        if (self.count == self.transition_value):
                self.transition = True
                self.count = 0
                self.transition_value = random.randint(min_transition_play_normal,max_transition_play_normal)

        if(data.data != "play"):   
            self.count += 1
            location = data.data.split('+')[1]
            rospy.loginfo("Requested location: %s",location)
            resp = self.ask_oracle("isVisited "+location)
            resp = resp.location.split()

            ## Color of the ball
            self.ball_type = resp[1] 
                      
            if(resp[0] == "True"):
                resp = self.ask_oracle("getPos "+self.ball_type)
                position = resp.location.split()
                while(self.target_pos_client(float(position[0]),float(position[1])) == None):
                    pass
                rospy.loginfo("Robot arrived in "+location)
                time.sleep(random.uniform(min_sleep_delay,max_sleep_delay))
                while(self.target_pos_client(person_pos.x,person_pos.y) == None):
                    pass
                rospy.loginfo("Robot arrived in person position")         
            else:
                self.goTo = True           
             
    def target_pos_client(self,x, y):

        """
            Send a goal to the action server of the robot and waits until it reaches the goal.
            While it is waiting, if the plan is aborted,it returns None; otherwise it will return
            the result of the sended goal 

            @param x: x coordinate of the target position
            @type x: float
            @param y: y coordinate of the target position
            @type y: float

            @returns: the result of the sended goal or "play"/"ball"/None
            @rtype: string

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

        while(client.get_state() != 3):
            if(client.get_state()== 4):
                rospy.loginfo("Plan aborted")
                client.cancel_all_goals()
                return None

        return client.get_result()

    def execute(self, userdata):

        """
            It sends the robot to the person. When the robot is there, waits for a goTo command.
            After some times it changes state to Normal

            @param userdata: used to pass data between states
            @type userdata: list

            @returns: transition value
            @rtype: String
        """       

        

        ## Variable used to change state to Normal
        self.transition = False

        ## Variable used to change state to Find
        self.goTo = False

        # function called when exiting from the node, it can be blacking      
        rospy.loginfo('Executing state PLAY')
        while(self.target_pos_client(person_pos.x,person_pos.y) == None):
            pass
        rospy.loginfo("Robot arrived in person position")
        sub_command = rospy.Subscriber("command", String, self.getCommand)
        while self.transition == False and self.goTo == False:
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
        A class used to represent the FIND behaviour
        of the robot

        Attributes
        -----
        @param from_track: Variable that represent if the transition has been done from TRACK state
        @type from_track: bool

        Methods
        -----
        object_detector_client():
            Makes a request to detector server and wait for the response
        execute(userdata)
            It launches the EXPLORE LITE package and checks for a ball.
            If there is a ball, not previously detected, it changes the state to TRACK.
            After some times ,if no ball is detected, it changes state to PLAY
    """

    def __init__(self):

        """
            Constrcutor
        """

        smach.State.__init__(self,outcomes=['someTimes','ball',],input_keys=['ballTypeFromPlay','fromTrack'],output_keys=['ballType'])

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
            It launches the EXPLORE LITE package and checks for a ball.
            If there is a ball, not previously detected, it changes the state to TRACK.
            After some times ,if no ball is detected, it changes state to PLAY

            @param userdata: used to pass data between states
            @type userdata: list

            @returns: transition value
            @rtype: String
        """
        rospy.loginfo('Executing state FIND')

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path_exp])
        launch.start()
        start = rospy.get_time()

        ## Variable that represent if the transition has been done from TRACK state
        self.from_track = userdata.fromTrack
        transition_time = random.randint(min_transition_find_play,max_transition_find_play)

        while True:
            resp = self.object_detector_client()
            ball = resp.object.split()
            center = float(ball[1])
            radius = float(ball[2])
            now = rospy.get_time()
            if((self.from_track == True) and (center == -1) and (radius == -1)):
                self.from_track = False
            elif ((self.from_track == False) and (center != -1) and (radius != -1)):
                launch.shutdown()
                userdata.ballType = userdata.ballTypeFromPlay
                return 'ball'
            if ((now-start) >= transition_time):
                launch.shutdown()
                return 'someTimes'
                   
# define state Sleep
class sleep(smach.State):

    """
        A class used to represent the SLEEP behaviour
        of the robot

        Methods
        -----
        target_pos_client(x, y):
            Send a goal to the action server of the robot and waits until it reaches the goal.
            While it is waiting, if the plan is aborted,it returns None; otherwise it will return
            the result of the sended goal 
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
            While it is waiting, if the plan is aborted,it returns None; otherwise it will return
            the result of the sended goal 

            @param x: x coordinate of the target position
            @type x: float
            @param y: y coordinate of the target position
            @type y: float

            @returns: the result of the sended goal or "play"/"ball"/None
            @rtype: string

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

        while(client.get_state() != 3):
            if(client.get_state()== 4):
                rospy.loginfo("Plan aborted")
                client.cancel_all_goals()
                return None

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

        while(self.target_pos_client(home_pos.x,home_pos.y) == None):
            pass
        rospy.loginfo("Robot arrived in home position")
        time.sleep(random.uniform(min_sleep_delay,max_sleep_delay))
        
        return 'wakeUp'

# define state Track
class track(smach.State):

    """
        A class used to represent the TRACK behaviour
        of the robot

        Attributes
        -----
        @param vel_pub: Publisher object for robot velocities
        @type vel_pub: Publisher

        @param position: Position of the robot
        @type position: Point

        @param vel: Velocity of the robot
        @type vel: Twist

        @param sub_scan: Object that represents the Subscriber to the topic scan
        @type sub_scan: Subscriber

        Methods
        -----
        clbk_odom(msg)
            Callback function to obtain the odometry information
            from the robot
        clbk_laser(msg)
            Callback function to obtain the laser information
            in order to perform a better track of the ball
        ask_oracle(request)
            Method used to ask a specific request to the oracle
        object_detector_client():
            Makes a request to detector server and wait for the response
        execute(userdata)
            It start to follow the ball. When it is near the ball, it saves the x and y position.
            If the ball is the requested one, it goes to PLAY otherwise it goes to FIND.
            If the transition has been done from NORMAL, it goes to NORMAL state
    """

    def __init__(self):

        """
            Constrcutor. Starts the node, initialize the publisher for the robot velocities and
            subscribe to the odom topic
        """

        smach.State.__init__(self,outcomes=['reached','reachedPlay','notRequest'],input_keys=['ballType'],output_keys=['toNormal','toFind'])
        ## Publisher object for robot velocities
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.clbk_odom)    

    def clbk_odom(self,msg):

        """
            Callback function to obtain the odometry information
            from the robot

            @param msg: robot state
            @type msg: Odometry
        """

        ## Position of the robot
        self.position = msg.pose.pose.position

    def clbk_laser(self,msg):

        """
            Callback function to obtain the laser information
            in order to perform a better track of the ball

            @param msg: laser readings
            @type msg: LaserScan
        """

        if(msg.ranges[0]< 0.5):
            self.vel.linear.z = -0.001
        elif(msg.ranges[719]<0.5):
           self.vel.linear.z = 0.001
        elif(msg.ranges[360]<0.5):
            self.vel.angular.z = 0
            self.vel.linear.x = 0.09

            
    def ask_oracle(self,request):

        """
            Method used to ask a specific request to the oracle

            @param request: request message
            @type request: OracleReq

            @returns: response to the request
            @rtype: OracleReqResponse
        """


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
            It start to follow the ball. When it is near the ball, it saves the x and y position.
            If the ball is the requested one, it goes to PLAY otherwise it goes to FIND.
            If the transition has been done from NORMAL, it goes to NORMAL state

            @param userdata: used to pass data bejoint_state = tween states
            @type userdata: list

            @returns: transition value
            @rtype: String
        """
        rospy.loginfo('Executing state TRACK')

        ## Velocity of the robot
        self.vel = Twist()

        ## Object that represents the Subscriber to the topic scan
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

            if((radius != -1) and (center != -1)):
                if(radius > 10):
                    self.vel.angular.z = -0.003*(center-400)
                    self.vel.linear.x = -0.007*(radius-110)
                elif(radius < 10):
                    self.vel.angular.z = 0
                    self.vel.linear.x = 0.09
            else:
                self.vel.angular.z = 0.5
            
            self.vel_pub.publish(self.vel)
                              
        self.ask_oracle("setPos "+ball_type+" "+str(self.position.x)+" "+str(self.position.y))
        self.sub_scan.unregister()      
        if(userdata.ballType != "ND"):
            rospy.loginfo("Requested ball: %s",userdata.ballType)
            if(userdata.ballType == ball_type):
                return 'reachedPlay'
            else:
                userdata.toFind = True
                return 'notRequest'
        else:
            userdata.toNormal = True
            return 'reached'
    

# define state Normal
class normal(smach.State):

    """
        A class used to represent the NORMAL behaviour
        of the robot

        Attributes
        -----
        @param play: Variable uses to change state to PLAY
        @type play: bool

        @param from_track: Variable uses to understand if the transition cames from TRACK state
        @type from_track: bool

        Methods
        -----
        getCommand(data)
            Callback method that received the "play" command and set the attribute
            play to true
        object_detector_client():
            Makes a request to detector server and wait for the response
        target_pos_client(x,y)
            Send a goal to the action server of the robot and waits until it reaches the goal.
            While it is waiting, if there is the ball, it stops the robot and return None
        execute(userdata)
            It checks if there is the ball, otherwise it will generate a random goal (x and y)
            for the robot. If there is the ball, it switches to the TRACK state. If the command
            play arrive, it switches to PLAY state.
            After some times, it switches to the SLEEP state
    """

    def __init__(self):

        """
            Constructor
        """

        smach.State.__init__(self,outcomes=['someTimes','ball','play'],input_keys=['fromTrack'])

    def getCommand(self,data):

        """
            Callback method that received the "play" command and set the attribute
            play to true

            @param data: command message
            @type data: str
        """

        self.play = True

    def object_detector_client(self):

        """
            Makes a request to detector server and wait for the response

            @returns: radius,center and color of the ball
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
            While it is waiting, if there is the ball or the play command arrive, it stops the robot

            @param x: x coordinate of the target position
            @type x: float
            @param y: y coordinate of the target position
            @type y: float

            @returns: the result of the sended goal or "play"/"ball"/None
            @rtype: string

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

        while(client.get_state() != 3):
            if self.play == True:
                client.cancel_all_goals()
                return 'play'
            if(client.get_state()== 4):
                rospy.loginfo("Position unreachable")
                client.cancel_all_goals()
                return None
            resp = self.object_detector_client()
            ball = resp.object.split()
            center = float(ball[1])
            radius = float(ball[2])
            if((center == -1) and (radius == -1)):
                self.from_track = False
            if((self.from_track == False) and (center != -1) and (radius != -1)):
                client.cancel_all_goals()
                return 'ball'

        return client.get_result()

    def execute(self, userdata):

        """
            It checks if there is the ball, otherwise it will generate a random goal (x and y)
            for the robot. If there is the ball, it switches to the TRACK state. If the command
            play arrive, it switches to PLAY state.
            After some times, it switches to the SLEEP state

            @param userdata: used to pass data between states
            @type userdata: list

            @returns: transition value
            @rtype: String
        """
        rospy.loginfo('Executing state NORMAL')

        count_value = random.randint(min_transition_normal_sleep,max_transition_normal_sleep)
        neg_map_x = map_x*-1
        neg_map_y = map_y*-1

        ## Variable uses to change state to PLAY
        self.play = False

        ## Variable uses to understand if the transition cames from TRACK state
        self.from_track = userdata.fromTrack
        sub_command = rospy.Subscriber("command", String, self.getCommand)

        for count in range(0,count_value):
            if self.play == True:
                sub_command.unregister()
                return 'play'
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
                if(result == "play"):
                    sub_command.unregister()
                    return 'play'
                elif(result == "ball"):
                    return 'ball'
                rospy.loginfo("Robot arrived in (%lf,%lf)",x,y)

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
    sm.userdata.sm_ball = "ND"
    sm.userdata.sm_tf = False

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
                                            remapping={'ballTypePlay':'sm_data'})
        smach.StateMachine.add('TRACK', track(), 
                               transitions={'reached':'NORMAL',
                                            'reachedPlay':'PLAY',
                                            'notRequest':'FIND'},
                               remapping={'ballType':'sm_ball',
                                          'toNormal':'sm_data',
                                          'toFind':'sm_tf'})
        smach.StateMachine.add('FIND', find(), 
                               transitions={'someTimes':'PLAY',
                                            'ball':'TRACK'},
                               remapping={'ballType':'sm_ball',
                                          'ballTypeFromPlay':'sm_data',
                                          'fromTrack':'sm_tf'})
        
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