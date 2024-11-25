#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include <array>
#include <cstdlib>
#include <ctime>


std::array<int, 3> generateRandomNumbers(); //generate two random numbers

//include "turtlesim/Pose.h" //name with first capital letter

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
    
	ros::Publisher turtle_pbl1 = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher turtle_pbl2 = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

	ros::ServiceClient turtle_client = n.serviceClient<turtlesim::Spawn>("/spawn");
	
	turtlesim::Spawn my_spawn;
    std::array<int, 3> spawn_pos;
    spawn_pos = generateRandomNumbers();
	my_spawn.request.x = spawn_pos[0];
	my_spawn.request.y = spawn_pos[1];
	my_spawn.request.theta = spawn_pos[2];
	my_spawn.request.name = "turtle2";
	turtle_client.call(my_spawn);


    if (turtle_client.call(my_spawn)) {
        ROS_INFO("Turtle 2 spawned [%f %f]", my_spawn.request.x, my_spawn.request.y);
    } else {
        ROS_ERROR("Failed to spawn turtle2");
    }
	
	ros::Rate loop_rate(10);
	geometry_msgs::Twist my_vel;
	
    int turtle_choice;
    bool err_flag = false;
	while(ros::ok()){

        //choose turtle, velocity parameters;
        std::cout<<"Select a turtle to control [1 | 2]"<<std::endl; std::cin>>turtle_choice;
        
        std::cout<<"Set linear and angular velocity in the following order [X Y Z_angular]"<<std::endl;
        std::cin>> my_vel.linear.x >> my_vel.linear.z >> my_vel.angular.z;

        do
            switch (turtle_choice)
            {
            case 1:
                err_flag = false;
                turtle_pbl1.publish(my_vel);
                break;

            case 2:
                err_flag = false;
                turtle_pbl2.publish(my_vel);
                break;

            default:
                std::cout<<"Selected turtle doesn't exist"<<std::endl;
                std::cout<<"Select a turtle to control [1 | 2]"<<std::endl;
                std::cin>>turtle_choice;
                err_flag=true;
                break;
            }
        while(err_flag);



		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	
	
	return 0;
}


//FUNCTIONS
std::array<int, 3> generateRandomNumbers() {
    
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    std::array<int, 3> randomNumbers = {std::rand() % 7 + 2, std::rand() % 7 + 2, std::rand()%360};
    return randomNumbers;
}

