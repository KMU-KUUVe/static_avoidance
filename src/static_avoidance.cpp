#include "static_avoidance.h"

namespace static_avoidance{

	StaticAvoidance::StaticAvoidance():nh_("~"){
		initSetup();
	}

	StaticAvoidance::StaticAvoidance(ros::NodeHandle nh):nh_(nh){
		initSetup();
	}

	void StaticAvoidance::initSetup(){
		pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped> ("/ackermann", 100);
		sub = nh_.subscribe("/raw_obstacles", 100, &StaticAvoidance::obstacle_cb, this);
		
		turn_left_flag = false;
		turn_right_flag = false;
		return_left_flag = false;
		return_right_flag = false;
		end_flag = false;
		sequence = 0;
		//flag = false;
		c.x = 100;
		end_count = 0;

	}

	void StaticAvoidance::obstacle_cb(const obstacle_detector::Obstacles& data) {
		
		nh_.getParam("CONST_VEL", CONST_VEL);
		nh_.getParam("OBSTACLE_RADIUS", OBSTACLE_RADIUS);
		nh_.getParam("DETECT_DISTANCE", DETECT_DISTANCE);
		
		flag = false;
		speed = CONST_VEL;
		//steer = CONST_STEER;
		for(int i = 0; i < data.circles.size(); i++) {
			if( (data.circles[i].radius >= OBSTACLE_RADIUS) &&(sqrt(data.circles[i].center.x * data.circles[i].center.x + data.circles[i].center.y * data.circles[i].center.y)  <= DETECT_DISTANCE)) {
				flag = true;
				c = data.circles[i].center;
				//ROS_INFO("CallBack c.x, c.y : %f, %f", c.x, c.y);
				//c.y is lateral axis. so if c.y > 0 means the obstacles are on the left.
				if(c.y < 0){
					sequence = 1;
				}
				else{
					sequence = 2;
				}
			}	
		}
	}
	void StaticAvoidance::run(){
		ros::Rate r(100);
		nh_.getParam("CONST_VEL", CONST_VEL);
		nh_.getParam("DETECT_DISTANCE", DETECT_DISTANCE);
		nh_.getParam("CONST_STEER", CONST_STEER);
		nh_.getParam("OBSTACLE_RADIUS", OBSTACLE_RADIUS);
		nh_.getParam("TURN_FACTOR", TURN_FACTOR);
		nh_.getParam("TURN_WEIGHT", TURN_WEIGHT);
		nh_.getParam("RETURN_WEIGHT", RETURN_WEIGHT);
		
		while(c.x >= DETECT_DISTANCE && ros::ok()){
			ros::spinOnce();			
			steer = CONST_STEER;
			speed = CONST_VEL;
			msg.drive.steering_angle = steer;
			msg.drive.speed = speed;
			pub.publish(msg);
			ROS_INFO("approaching the obstacle");
		}

		while(ros::ok()){
#ifdef DEBUG
			ROS_INFO("While entered");
#endif
			ros::spinOnce();

		double distance = sqrt(c.x * c.x + c.y * c.y);
		// c.x is longitudinal axis. so if c.x >0 means the obstacles are in the  rear.		
		//ROS_INFO("While c.x, c.y : %f, %f", c.x, c.y);
		if(flag == 1){
			//speed = CONST_VEL;
			//avoidance right Obstacles
			if(sequence == 1){
				if(c.x > 0.5){
					turn_left_flag = true;
					return_right_flag = false;
					turn_right_flag = false;
					return_left_flag = false;
					end_flag = false;
				}
				else{
					return_right_flag = true;
					turn_left_flag = false;
					turn_right_flag = false;
					return_left_flag = false;
					end_flag = false;
				}
			}
			else if(sequence == 2){
				if(c.x > 0.5){
					turn_right_flag = true;
					return_left_flag = false;
					turn_left_flag = false;
					return_right_flag = false;
					end_flag = false;
				}
				else{
					return_left_flag = true;
					turn_right_flag = false;
					turn_left_flag = false;
					return_right_flag = false;
					end_flag = false;
				}
			}
		end_count = 0;
		}
		else if(flag == 0){
			end_count ++;
			ROS_INFO("count : %d", end_count);
			if(end_count >= 350){
				end_flag = true;
				steer = 0;
				speed = 0;
				turn_left_flag = false;
				turn_right_flag = false;
				return_left_flag = false;
				return_right_flag = false;
			}
			/*
			turn_left_flag = false;
			turn_right_flag = false;
			return_left_flag = false;
			return_right_flag = false;*/
		}



		if(turn_left_flag){
			steer = int(CONST_STEER - ((TURN_FACTOR - distance) * TURN_WEIGHT));
		}
		if(return_right_flag){
			steer = int(CONST_STEER + (distance * RETURN_WEIGHT));
		}
		if(turn_right_flag){
			steer = int(CONST_STEER + ((TURN_FACTOR - distance) * TURN_WEIGHT));
		}
		if(return_left_flag){
			steer = int(CONST_STEER - (distance * TURN_WEIGHT));
		}

		if(steer > 26){
			steer = 26;
		}
		if(steer < -26){
			steer = -26;
		}

		// ackermann_msgs::AckermannDriveStamped msg;
		//std_msgs::String msg;
		//ROS_INFO("distance: %f", distance);
		ROS_INFO("sequence : %d", sequence);
		//ROS_INFO("c.x : %f", c.x);
		ROS_INFO("c.x, c.y : %f, %f", c.x, c.y);
		//ROS_INFO("c.y : %f", c.y);
		ROS_INFO("flag : %d",flag);
		ROS_INFO("left turn flag : %d", turn_left_flag);
		ROS_INFO("return right flag : %d", return_right_flag);
		ROS_INFO("right turn flag : %d", turn_right_flag);
		ROS_INFO("return left flag : %d", return_left_flag);
		ROS_INFO("end count : %d",end_count);
		ROS_INFO("end flag : %d",end_flag);
		ROS_INFO("Steer : %d Speed : %d", steer, speed);
		ROS_INFO("-----------------------------------------");


		//msg.data = std::to_string(steer) + "," + std::to_string(speed) + "," ;
		msg.drive.steering_angle = steer;
		msg.drive.speed = speed;
		pub.publish(msg);
	
		r.sleep();
		}
	}
} //end namespace
