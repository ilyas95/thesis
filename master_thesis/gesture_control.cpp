/**
Send commands to drone using hand gestures. Images coming from laptop positioned in front of person doing gestures
 */
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
using namespace std;
using namespace ros;

class Node{
private:
	NodeHandle nh;
	Publisher pub;
	Publisher pub_land;
	Publisher pub_takeoff;
	string state;
	geometry_msgs::Twist twist;
	std_msgs::Empty empty_msg;
public:
	Node();
	void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
	void set_velocity(float x, float y, float z, float angular);
	void send_command(float& R_dx, float& R_sx, string state);
	void set_state(string input);
	string get_state();
};

Node::Node() //Constructor
{
	pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel/", 1);
	pub_land = nh.advertise<std_msgs::Empty>("/bebop/land/", 1);
	pub_takeoff = nh.advertise<std_msgs::Empty>("/bebop/takeoff/", 1);
	state = "landed";
}
 
void Node::set_state(string input){
	/**
	Sets drone state. Possible states: "landed" and "flying".
	Default state is "landed"
	*/
	state = input;
}

string Node::get_state(){
	/**
	Gets current drone state and then it returns it
	*/
	return state;
}

void Node::set_velocity(float x, float y, float z, float angular){
	/**
	It sets the linear and angular velocity components based on input received
	Then it publishes the velocity to the drone
	*/	
	twist.linear.x = x, twist.linear.y = y, twist.linear.z = z;
	twist.angular.x = 0, twist.angular.y = 0, twist.angular.z = angular;
	pub.publish(twist);
}

void Node::send_command(float& R_dx, float& R_sx, string state){
	/**
	It uses the drone state and the ratios to publish via ROS
	the velocities to the drone based on the ratios.
	*/
	//Takeoff-Hovering
	if(R_dx >= 0.9 && R_dx < 1.1 && R_sx >= 0.9 && R_sx < 1.1 && state == "landed"){
		set_state("flying");
		pub_takeoff.publish(empty_msg);
		cout << "Take off/Hovering" << endl;
	}
	//Fly Down
	else if(R_dx >= 0.6 && R_dx < 0.85 && R_sx >= 0.6 && R_sx < 0.85 && state == "flying"){
		set_velocity(0,0,-0.3,0);
		cout << "Fly Down" << endl;
	}
	//Fly Up
	else if(R_dx >= 1.6 && R_dx < 2.4 && R_sx >= 1.6 && R_sx < 2.4 && state == "flying"){
		set_velocity(0,0,0.3,0);
		cout << "Fly Up" << endl;
	}
	//Fly Forward
	else if(R_dx > 1.6 && R_sx >= 0.8 && R_sx < 1.2 && state == "flying"){
		set_velocity(3,0,0,0);
		cout << "Fly Forward" << endl;
	}
	//Fly Backward
	else if(R_dx >= 0.8 && R_dx < 1.2 && R_sx > 1.6 && state == "flying"){
		set_velocity(-3,0,0,0);
		cout << "Fly Backward" << endl;
	}
	//Fly Left
	else if(R_dx >= 0.8 && R_dx < 1.2 && R_sx < 0.7 && state == "flying"){
		set_velocity(0,0.3,0,0);
		cout << "Fly Left" << endl;
	}
	//Fly Right
	else if(R_dx < 0.7 && R_sx >= 0.8 && R_sx < 1.2 && state == "flying"){
		set_velocity(0,-0.3,0,0);
		cout << "Fly Right" << endl;
	}
	//Clockwise around Z axis "right"
	else if(R_dx > 1.6 && R_sx < 0.7 && state == "flying"){
		set_velocity(0,0,0,-0.3);
		cout << "Clockwise rotation Z axis" << endl;
	}
	//Anti-clockwise around Z axis "left"
	else if(R_dx < 0.7 && R_sx > 1.6 && state == "flying"){
		set_velocity(0,0,0,0.3);
		cout << "Anti Clockwise rotation Z axis" << endl;
	}
	//Landing
	else if(R_dx > 0 && R_dx < 0.6 && R_sx > 0 && R_sx < 0.6 && state == "flying"){
		set_state("landed");
		pub_land.publish(empty_msg);
		cout << "Landing" << endl;
	}
}

void Node::callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	/**
	It iterates through the bounding boxes received and distinguished
	between face, left hand and right hand. Then calculates left ratio
	and right ratio based on vertical distance between the face and the hands.
	*/
	int temp_x;
	int temp_y;
	int Cy_face = 0, Cx_face = 0, Cy_hand_dx = 0, Cy_hand_sx = 0;
	float R_dx = 0, R_sx = 0;
	for(int i = 0; i < msg->bounding_boxes.size();i++){

		temp_x = (msg->bounding_boxes[i].xmin + (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)/2);
		temp_y = (msg->bounding_boxes[i].ymin + (msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin)/2);
		if(msg->bounding_boxes[i].Class == "face"){
			Cy_face = temp_y;
			Cx_face = temp_x;
		}
		if(msg->bounding_boxes[i].Class == "hand" and Cy_face != 0){
			if(temp_x < Cx_face){
				Cy_hand_dx = temp_y;
			}
			else if(temp_x > Cx_face){
				Cy_hand_sx = temp_y;
			}
		}
		if(Cy_hand_dx != 0 && Cy_hand_sx != 0){
			R_dx = Cy_face/float(Cy_hand_dx);
			R_sx = Cy_face/float(Cy_hand_sx);
		}
		send_command(R_dx,R_sx,get_state());
	}
}

int main(int argc, char **argv)
{
	init(argc, argv, "node");
	Node node;
	NodeHandle nh;
	Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes/", 1000, &Node::callback, &node);
	Rate loop_rate(1);
	spin();

	return 0;
}
