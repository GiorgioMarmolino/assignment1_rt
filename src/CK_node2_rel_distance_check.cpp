#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <math.h>

#define min_lim 1.0 //minimal value in space
#define max_lim 10.0 //max value in space
#define min_distance 2.0 //minimal distance between turtles

turtlesim::Pose pos_t1;
geometry_msgs::Twist vel_t1;
turtlesim::Pose pos_t2;
geometry_msgs::Twist vel_t2;
bool t1_moving = false;

/*
A node that checks the relative distance between turtle1 and turtle2 and:

    - publish on a topic the distance (you can use a std_msgs/Float32 for that)

    - stops the moving turtle if the two turtles are “too close” (you may set a threshold to monitor that)

    - stops the moving turtle if the position is too close to the boundaries (.e.g, x or y > 10.0, x or y < 1.0

*/

void stop(ros::Publisher& trt_pbl) {
	geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0;
    trt_pbl.publish(vel);
}
bool world_limit(turtlesim::Pose p){return (p.x <= min_lim || p.x>= max_lim || p.y <= min_lim || p.y >= max_lim);}
bool turtles_too_close(double distance){return (distance <= min_distance);}
double mod(float a, float b){ return sqrt(pow(a,2)+pow(b,2));}
double compute_distance(turtlesim::Pose a, turtlesim::Pose b){
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	
	return mod(dx,dy);
}

void trt1_pose(const turtlesim::Pose::ConstPtr& msg) {

    pos_t1.x = msg->x;
    pos_t1.y = msg->y;
    pos_t1.theta = msg->theta;
}
void trt2_pose(const turtlesim::Pose::ConstPtr& msg) {
    pos_t2.x = msg->x;
    pos_t2.y = msg->y;
    pos_t2.theta = msg->theta;
}
void trt1_vel(const geometry_msgs::Twist::ConstPtr& msg) {
    vel_t1.linear.x = msg->linear.x;
    vel_t1.linear.y = msg->linear.y;
    vel_t1.angular.z = msg->angular.z;
	if(vel_t1.linear.x == 0 && vel_t1.linear.y == 0 && vel_t1.angular.z == 0) t1_moving = false;
	else t1_moving = true;
	
}
void trt2_vel(const geometry_msgs::Twist::ConstPtr& msg) {
    vel_t2.linear.x = msg->linear.x;
    vel_t2.linear.y = msg->linear.y;
    vel_t2.angular.z = msg->angular.z;
}

int moving_turtle(){ int moving_turtle_id = t1_moving ? 1 : 2; return moving_turtle_id;}






int main(int argc, char **argv){

    ros::init(argc,argv,"CK_node2_rel_distance_check");
	ros::NodeHandle n;
    ros::Publisher dist_pbl = n.advertise<std_msgs::Float32>("turtles_distance", 100);

    ros::Subscriber trt1_pos_sub = n.subscribe<turtlesim::Pose>("turtle1/pose", 10, trt1_pose);
    ros::Subscriber trt1_vel_sub = n.subscribe<geometry_msgs::Twist>("turtle1/cmd_vel",10,trt1_vel);
    ros::Publisher turtle_pbl1 = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

	ros::Subscriber trt2_pos_sub = n.subscribe<turtlesim::Pose>("turtle2/pose", 10, trt2_pose);
    ros::Subscriber trt2_vel_sub = n.subscribe<geometry_msgs::Twist>("turtle2/cmd_vel",10,trt2_vel);
    ros::Publisher turtle_pbl2 = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

	std_msgs::Float32 dist;
	ros::Rate loop_rate(1);
	while(ros::ok()){

		//compute distance and publish on topic
		dist.data = compute_distance(pos_t1, pos_t2);
		dist_pbl.publish(dist);

		if(turtles_too_close(dist.data))
			switch(moving_turtle())
			{
			case 1:
				stop(turtle_pbl1);
				ros::spinOnce();
            	ros::Duration(0.1).sleep();
				break;
			case 2:
				stop(turtle_pbl2);
				ros::spinOnce();
        	    ros::Duration(0.1).sleep();
				break;
			}
			
		if(world_limit(pos_t1)){
			stop(turtle_pbl1);
			ros::spinOnce();
            ros::Duration(0.1).sleep();
			break;
		}
		if(world_limit(pos_t2)){
			stop(turtle_pbl2);
			ros::spinOnce();
        	ros::Duration(0.1).sleep();
		}






		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	
	
	return 0;
}

//ROS_INFO, ROS_ERROR, ROS_WARNING

