ProjectGuide
# Marmolino Giorgio - RobEng24/25 - Assignment no.1: Control of the turtle and distance check

# General overview and node description
This project consist in create a ROS package "assignment1_rt" containing the two nodes developed in order to control the two turtles and monitoring the distance between them; the assigned tasks are splitted among the two nodes. Both nodes have been developed in *C++*


## User Interface: Node 1 - Control Node
UI_node1_Control_Node: the first node implements an interface that allows the user to choose among the two turtles (the interface ask to the user to choose a number among 1 and 2 in order to select the corresponding turtle) and set velocity parameters related to linear velocity on X and Y axes (related to the turtle reference system - so consider the X axis aligned with the head of the turtle in the simulation enviornment) and the angular speed of the turtle (so the user has to input 3 integer numbers). Velocity can assume values in the range of [-5 +5]; if the input is an higher value the value will be set to the max value of the interval (so -5 or +5, depending on the direction). In this way the turtle will move with this set of velocity parameters for 1 second; after that the user will be able again to choose a turtle and set velocity parameters.
The node is activated after the actiovation of the turtlesim_node (the one that summon the turtlesim enviornment), and it will spawn the turtle_2 in a random position with coordinates:
- X and Y in range [1.5 8.5];
- Theta angle in range [0 359] degrees;

When the turtle spawn successfully there will be a ROS_INFO message that confirm the spawn of turtle 2 in given coordinates.

## Check Distance: Node 2 - Relative Distance Check
CK_Node2_Rel_Distance_Check: the second node is meant to:
1) Check the distance between the turtles: in order to avoid turtles collision, when the distance is less than the defined threshold, the node will stop the moving turtle (the turtle selected by the user with specific velocity) and move it in reverse for 0.1 seconds; a ROS_WARN message will be displayed with a message that specify that the moving turtle has been stopped cause it was too close too the other turtle, specifying the coordinates of the two turtles; the nodes provides a publisher for the distance on topic /turtles_distance ;
2) Check if the turtles are too close to the simulation environment boundaries: in that case the node will stop the turtle approaching the world_bound and move it in reverse for 0.1 seconds;



# Package installation and use

Assuming that the user has already installed ROS and finished the workspace configuration, move in the workspace directory in the terminal environment and use the following command in order to compile only this package:
''bash

cd my_ros/src/
git clone https://github.com/GiorgioMarmolino/assignment1_rt
cd ../../
catkin_make --only-pkg-with-deps assignment1_rt

''
After this the package is ready to run; to do this we need to run the master node "roscore", and then the other nodes, so we are running:
1) "roscore", the master node;
2) "turtlesim turtlesim_node", simulation environment;
3) "assignment1_rt UI_node1_control_node";
4) "assignment1_rt CK_node2_rel_distance_check";

So we split the terminal in 4 windows, one for each node; so in the following order, in each window we will use the following commands:
(1)
''bash
roscore
''

(2)
''bash
rosrun turtlesim turtlesim_node
''

(3)
''bash
rosrun assignment1_rt UI_node1_control_node
''

(4)
''bash
rosrun assignment1_rt CK_node2_rel_distance_check
''

Following this order the simulation environment should spawn with the two turtles inside; now it's possible to interact with the turtles using the terminal window related to the first node; info or warn messages will appear on the window related to the second node depending on verified events. 
