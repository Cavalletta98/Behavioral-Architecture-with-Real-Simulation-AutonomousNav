# Behavioral-Architecture-with-Real-Simulation-AutonomousNav
ROS architecture for a robot moving into a simulated house. The robot has 5 behaviors:
- Play
- Sleep
- Normal
- Find
- Track

The user can interact with the robot by sendi a play command follow by a goTo+room
# Software architecture and states diagrams
## Software architecture 
The architercture is composed by 5 components: 

- Ball Info: detect a colored ball and provide a service to retrieve the information of the detected ball

- Command: simulate a user

- Oracle: knows all the things about the context of the application

- Command manager: implement robot behaviors through a FSM

- Simulation,Move Base and Explore lite: simulate the robot and the enviroment. They allow the robot to navigate into the house

<p align="center">
  <img src="./images/Final_Behavioral_Architecture_UML.jpg">
</p>

## State diagram
The finite state machine is composed by 5 state (robot behaviors):

- PLAY: the robot goes to the person position. When it is there, it waits the goTo+room command. If the rooms is already known, it goes there; otherwise it will goes to FIND state

- NORMAL (initial state): robot moves in a random ways

- SLEEP: robot goes to home position, it stays there for a certain time and then goes to NORMAL state

- FIND: robot tries to find the requested colored ball by moving into the house

- TRACK: robot tracks the ball in order to store its position (x and y) for future purpose

<p align="center">
  <img src="./images/Final_Behavioral_Architecture_FSM.jpg">
</p>

## ROS messages and parameters
The messages are:

- `PoseStamped`: robot target position
- `String`: radius,center and color of the ball/User command/Request and response of the oracle
- `CompressedImage`: images received from the camera
- `Twist`: robot velocities
- `Odometry`: robot odometry
- `GoalStatusArray`: status of the move base action server
- `PolygonStamped`: robot footprint
- `Movement commands;Sensor topics and map;Grid messages`

The parameters are:

- `path_exp`: path of the explore lite launch file (string, MUST BE DEFINE BEFORE THE EXECUTION)
- `home_pos_x,home_pos_y`: define the home position in the map (integer)
- `map_x,map_y`: define the dimensions of the map (integer)
- `min_transition_play_normal,max_transition_play_normal`: define the min and max delay to trasit between PLAY and NORMAL (integer)
- `min_transition_normal_sleep,max_transition_normal_sleep`: define the min and max delay to trasit between NORMAL and SLEEP (integer)
- `min_sleep_delay,max_sleep_delay`: define the min and max delay for the SLEEP state (double)
- `min_transition_find_play,max_transition_find_play`: define the min and max delay to trasit between FIND and PLAY (integer,it is expressed as seconds)
- `min_sleep_delay,max_sleep_delay`: define the min and max delay for sleeping (double)

# Packages and files
There are 5 packages:

- `Sensoring`: contains the [command.py](sensoring/scripts/command.py) and [ball_detector.py](sensoring/scripts/ball_detector.py) files used to send user command and detect a colored ball
- `Knowledge`: contains the [oracle.py](knowledge/src/oracle.py) file used to store all the knowledge of the application
- `Manager`: contains the [command_manager.py](manager/src/command_manager) file that implements the FSM of robot behaviors.
- `Simulation`: contains all the files necessary for running the simulation
- `Explore`: contains all the files of the explore lite package of ROS

# Installation and running
In order to run this software, the following prerequisities are needed:
- [ROS Noetic](http://wiki.ros.org/noetic)
- [smach](http://wiki.ros.org/smach)
- [gazebo](http://gazebosim.org/)
- [Move Base](http://wiki.ros.org/move_base)
- [rviz](http://wiki.ros.org/rviz)

Before running the software, you must have all files as executable otherwise you can make them executable with the following command
```
cd <your_workspace>/src/Behavioral-Architecture-with-Real-Simulation-AutonomousNav
chmod +x sensoring/scripts/*
chmod +x knowledge/scripts/*
chmod +x manager/scripts/*
```
After that, you have to setup the parameter of the path of explore lite.
To run the software you have to use two terminals due to a problem of ROS Noetic (it will print a lot of warning and you cannot see nothing about the application).
In the first terminal
```
cd <your_workspace>
catkin_make
source devel/setup.bash
cd src/Behavioral-Architecture-with-Real-Simulation-AutonomousNav
roslaunch simulation simulation.launch
```
In the other terminal
```
cd <your_workspace>
catkin_make
source devel/setup.bash
cd src/Behavioral-Architecture-with-Real-Simulation-AutonomousNav
roslaunch launch_file.launch
```

# Working hypothesis and environment
The robot is a pet that interact with a human who moves the ball into an a simulated arena. The arena is a 7x7 grid. Target position of the robot and the ball belong to the arena. The robot has 3 behaviours: PLAY,NORMAL,SLEEP. The PLAY behaviour will start only if the robot sees the ball. The ball is green and is very big with respect to the robot in order to detect it. The home position can be initialize before starting the simulation and cannot be changed during the execution. The word "someTimes" is inteded as number of cycles for which it is executed a piece of code. The ball has no collisions.The ball is considered stationary when the robot is close to it and is stopped.

# System's features
- Specify different dimensions of the arena
- It is possibile to define different delays for the simulation
- It is possibile to define different position of the "home" inside the arena before start the simulation
- It is possible to visualize the states transition in the shell
- The robot will notify if it will reach the target position and it is possibile to visualize it in the shell (the position that the robot has reached)
- The robot can perceives the ball even if it is moving in the NORMAL state
- It is possibile to see the camera of the robot in a separated window with the possibility to see a circle around the detected ball
- It is possible to make the ball disappear simply by moving it under the ground
- All the delays are random values between the defined ranges
- The robot ignores the ball when it is in the SLEEP state
- The robot can follow the ball well when it starts to exit the robot's field of view quickly enough thanks to the choice of a good gain for angular velocity
- The robot starts to rotate, for a certain period of time, to look for the ball when it loses it before passing to the NORMAL state

# System's limitations
- Slight wheelies of the robot when it passes from NORMAL to PLAY state
- Shaking of the camera due to the joint of the head
- Very slight wheelies of the robot when it starts to move torward a position during NORMAL behavior
- When the ball moves close to the robot but it is stationary, the robot moves its head
- Robot shaking while following the ball
- Since the PLAY-NORMAL transition is implemented as a number of cycles, if the range is small (e.g. 1-10), the transition will be fast (use suggested range or different values)

# Possible technical improvements
- Fine tune the PID values for the head joint
- Adopt an incremental gain solution in order to avoid robot wheelies and allow very fast ball following
- Keeping this robot geometry, adopt a control solution that allows you to follow the ball without shaking
- Add collision properties to the ball
- Adopt a different mechanism to understand if the ball is stationary or not
- Implement the transition mechanism between PLAY and NORMAL with a timer

# Author and contact
[Simone Voto](https://github.com/Cavalletta98) - simone.voto98@gmail.com