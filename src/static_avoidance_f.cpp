#include <ros/ros.h>
#include <iostream>
#include <string>

#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
// #include <ackermann_msgs/AckermannDriveStamped.h>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <std_msgs/String.h>
// #include <std_msgs/Empty.h>

#define DETECT_DISTANCE 0.2
#define CONSTANT_STEER 1500
#define CONSTANT_VEL 1550

using namespace std;

ros::Publisher pub;

/*bool turn_left_flag = false;
bool turn_right_flag = false;
bool return_left_flag = false;
bool return_right_flag = false;*/
bool forward_flag = false;
int sequence = 0;

vector<int> steer_buffer;

void calculator(const obstacle_detector::Obstacles data) {
  geometry_msgs::Point c, mycar;
  bool flag = true;
  int speed = CONSTANT_VEL;
  int steer = 1500;


  // Select nearest point and assign it to 'c'
  for(int i = 0; i < data.circles.size(); i++) {
    if(sqrt(data.circles[i].center.x * data.circles[i].center.x + data.circles[i].center.y * data.circles[i].center.y)  <= DETECT_DISTANCE) {
      //if(sqrt(c.x*c.x + c.y*c.y) > sqrt(data.circles[i].center.x * data.circles[i].center.x + data.circles[i].center.y * data.circles[i].center.y)) {
        flag = true;
        c = data.circles[i].center;
      //}
	if(c.x < 0){
	  sequence = 1;
	}
      	else{
          sequence = 2;
      	}

    }
  }
  

  double distance = sqrt(c.x * c.x + c.y * c.y);

     if(flag == 1){
      //avoidance right Obstacles
      if(sequence == 1){
          //turn_left_flag = true;
	  steer = 1400;
	  ros::Duration(1).sleep();
	  //turn_left_flag = false;

          //return_right_flag = true;
	  steer = 1650;
	  ros::Duration(1).sleep();
	  //return_right_flag = false;
	  forward_flag = true;
      }
      else if(sequence == 2){
          //turn_right_flag = true;
	  steer = 1600;
	  ros::Duration(1).sleep();
	  //turn_right_flag = false;
          //return_left_flag = true;
	  steer = 1350;
	  ros::Duration(1).sleep();
	  //return_left_flag = false;  
	  forward_flag = true;    
      }
    }
    else{
      steer = 1500;
    }
    
    /*
    if(turn_left_flag){
	steer = 1400;        
	//steer = CONSTANT_STEER - ((1 - distance) * 200);
        //turn_left_flag = false;
    }
    if(return_right_flag){
	steer = 1650;
        //steer = CONSTANT_STEER + (distance * 200);
        //return_right_flag = false;
    }
    if(turn_right_flag){
	steer = 1600;  
	//steer = CONSTANT_STEER + ((1 - distance) * 200);
        //turn_right_flag = false;
    }
    if(return_left_flag){
	steer = 1350;        
	//steer = CONSTANT_STEER - (distance * 200);
        //return_left_flag = false;
    }
*/
    if(forward_flag){
        steer = CONSTANT_STEER;
        if(distance < 0.1){
	    forward_flag = false;	
	}
    }

 
 // ackermann_msgs::AckermannDriveStamped msg;
  std_msgs::String msg;
  ROS_INFO("distance: %f", distance);
  ROS_INFO("sequence : %d", sequence);
  ROS_INFO("c.x : %f", c.x);
  ROS_INFO("c.y : %f", c.y);
  ROS_INFO("forward : %d", forward_flag);
  /*ROS_INFO("left turn flag : %d", turn_left_flag);
  ROS_INFO("right turn flag : %d", turn_right_flag);
  ROS_INFO("return left flag : %d", return_left_flag);
  ROS_INFO("return right flag : %d", return_right_flag);*/
  ROS_INFO("Steer : %d", steer);
  ROS_INFO("Speed : %d", speed);
  ROS_INFO("-----------------------------------------");
    
  
  //std::string steer_speed = std::to_string(steer) + std::to_string(speed);
  msg.data = std::to_string(steer) + "," + std::to_string(speed) + "," ;
  // msg.drive.steering_angle = steer;
  // msg.drive.speed = speed;
  
   pub.publish(msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Static_avoidance_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("raw_obstacles", 1, calculator);
  pub = nh.advertise<std_msgs::String>("write", 1000);
  ros::spin();
}
