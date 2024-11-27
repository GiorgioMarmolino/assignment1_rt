#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include <unistd.h>
#include <array>
#include <cstdlib>
#include <ctime>
#include <string>


#define MAX_SPEED 5.0 //max speed value for turtles

std::array<double, 3> generateRandomNumbers(); //generate two random numbers


/*

? Spawn a new turtle in the environment: turtle2

? Implement a simple textual interface to retrieve the user command. 
The user should be able to select the robot they want to control (turtle1 or turtle2), and the 
velocity of the robot. 

? The command should be sent for 1 second, and then the robot should stop, and the user should 
be able again to insert the command. 


*/

int main(int argc, char **argv){


	ros::init(argc, argv, "UI_node1_control_node");
	ros::NodeHandle n;
    
    //publishers for turtles velocity
	ros::Publisher turtle_pbl1 = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher turtle_pbl2 = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    //service to spawn the turtle in random position
	ros::ServiceClient turtle_client = n.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn my_spawn;
    std::array<double, 3> spawn_pos;
    spawn_pos = generateRandomNumbers();
	my_spawn.request.x = spawn_pos[0];
	my_spawn.request.y = spawn_pos[1];
	my_spawn.request.theta = spawn_pos[2];
	my_spawn.request.name = "turtle2";

    //Check if the turtle has been spawned
    if (turtle_client.call(my_spawn)) ROS_INFO("Turtle 2 spawned [%f %f]", my_spawn.request.x, my_spawn.request.y);
    else  ROS_ERROR("Failed to spawn turtle2");
       
	ros::Rate loop_rate(10);
	geometry_msgs::Twist my_vel;
	
    int turtle_choice;
    bool err_flag = false; //flag is true when errors 
	while(ros::ok()){

        //choose turtle, velocity parameters;
        do{
        std::cout<<"Select a turtle to control [1 | 2]"<<std::endl; std::cin>>turtle_choice;
        if(turtle_choice != 1 && turtle_choice != 2){ err_flag = true; std::cout<<"Selected turtle doesn't exist"<<std::endl;}
        else err_flag = false;
        }while(err_flag);
        
        //if input speed is higher than MAX_SPEED, the speed is set to MAX_SPEED
        std::cout<<"Set linear and angular velocity (max speed 5) in the following order [X Y Z_angular]"<<std::endl;
        std::cin>> my_vel.linear.x >> my_vel.linear.z >> my_vel.angular.z;
        if (abs(my_vel.linear.x)>5) {my_vel.linear.x = std::min(my_vel.linear.x, MAX_SPEED); my_vel.linear.x = std::max(my_vel.linear.x, -MAX_SPEED); }
        if (abs(my_vel.linear.y)>5) {my_vel.linear.y = std::min(my_vel.linear.y, MAX_SPEED); my_vel.linear.y = std::max(my_vel.linear.y, -MAX_SPEED); }
        if (abs(my_vel.angular.z)>5) {my_vel.angular.z = std::min(my_vel.angular.z, MAX_SPEED); my_vel.angular.z = std::max(my_vel.angular.z, -MAX_SPEED); }

        switch (turtle_choice) //publish speed of selected turtle
        {
        case 1:
            turtle_pbl1.publish(my_vel); break;
            
        case 2:
            turtle_pbl2.publish(my_vel); break;
        }
        
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

//FUNCTIONS
std::array<double, 3> generateRandomNumbers() {
    
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    std::array<double, 3> randomNumbers = {std::rand() % 8 + 1.5, std::rand() % 8 + 1.5, std::rand()%360};
    return randomNumbers;
}
