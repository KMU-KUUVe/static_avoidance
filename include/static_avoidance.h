#include <ros/ros.h>
#include <iostream>
#include <string>

#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <std_msgs/String.h>
// #include <std_msgs/Empty.h>

#define DETECT_DISTANCE 5
#define CONSTANT_STEER 0
#define CONSTANT_VEL 6
#define OBSTACLE_RADIUS 0.3
#define TURN_FACTOR 2
#define TURN_WEIGHT 12
#define RETURN_WEIGHT 14

using namespace std;

namespace static_avoidance{

class StaticAvoidance{
public:
	StaticAvoidance();
	StaticAvoidance(ros::NodeHandle nh);
	void initSetup();
    	void obstacle_cb(const obstacle_detector::Obstacles& data);
	void run();

private:
	ros::NodeHandle nh_;
	ros::Publisher pub;
	ros::Subscriber sub;
	
	int steer;
	int speed;
	bool turn_left_flag;
	bool turn_right_flag;
	bool return_left_flag;
	bool return_right_flag;
	bool end_flag;
	int sequence;
	bool flag;

	vector<int> steer_buffer;

	geometry_msgs::Point c;
	ackermann_msgs::AckermannDriveStamped msg;
};

} //end namespace
