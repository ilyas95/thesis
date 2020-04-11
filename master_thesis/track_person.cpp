/*
Detects and tracks a person by sending commands to the drone using a PID based
on the error between ideal position and bounding box of tracked person
 */
#include <iostream>
#include <string>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
using namespace std;
using namespace ros;

//CONSTANTS
// Gains
const float fb[3] = {-,-,-}; //forward-backward axis
const float lr[3] = {-,-,-}; //left-right axis
const float ud[3] = {-,-,-}; //up-down axis
//Camera image resolution
const float Cy_center = 320; // center position of the image 640x480
const float Cz_center = 240; // center position of the image
const float Area_id = 640*480*0.4; // 40% of pixel area (for x)

class Node{
private:
	NodeHandle nh;
	Publisher pub;
        geometry_msgs::Twist twist;
	string first_person;
	int last_person_y, integral;
	int previous[3] = {0,0,0};
public:
	Node();
	void callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
	void send_command(float x, float y, float z);
	float calculate_pid(float err_vel, const float gain[], int axis);
	void set_first_person(string p);
	string get_first_person();
	void set_last_person_y(int Cy);
	int get_last_person_y();
};

Node::Node() //Constructor
{
	pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel/", 1);
	first_person = "false";
	last_person_y = 0;
	integral = 0;
}

void Node::set_first_person(string p){
	/**
	Sets first person state. Possible states: "true" and "false".
	Default state is "false" - no person detected -> no person to track
	*/
	first_person = p;
}

string Node::get_first_person(){
	/**
	Gets first person detected and then it returns it
	*/
	return first_person;
}

void Node::set_last_person_y(int Cy){
	/**
	Sets last person Cy coordinate
	*/
	last_person_y = Cy;
}

int Node::get_last_person_y(){
	/**
	Gets last person detected and then it returns it
	*/
	return last_person_y;
}

float Node::calculate_pid(float err_vel, const float gain[], int axis){
	/**
	PID controller that calculates velocity to send to the drone
	*/
	integral+= err_vel;
	int derivative = err_vel - previous[axis];
	float output = gain[0] * err_vel + gain[1] * derivative + gain[2] * integral;
	return output;
}

void Node::send_command(float x, float y, float z){
	/**
	Send velocity calculated by PID controller to the drone via ROS
	*/
	twist.linear.x = x, twist.linear.y = y, twist.linear.z = z;
	twist.angular.x = 0, twist.angular.y = 0, twist.angular.z = y;
	pub.publish(twist);
	cout << twist.linear.x << " " << twist.linear.y << " " << twist.linear.z << " "
			<< twist.angular.z << endl;
}

void Node::callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	/**
	It iterates through the bounding boxes received and tracks most centered
	person, then it calculates the error between ideal position and bounding
	box position. Then error is used as input for the PID in order to calculate
	velocity to send to the drone.
	*/
	int box_num = 0; // box counter
	int i = 0; // person counter
	int diff_y = Cy_center;
	int i_x, Cy, Cz, Area_ref;
	float err_x, err_y, err_z, output_x, output_y, output_z;
	first_person = get_first_person();

	if(first_person == "true"){
		/**Check who is the same person as the previous frame based on how close
		it is from its previous position
		*/
		for(int i = 0; i < msg->bounding_boxes.size();i++){
			if(msg->bounding_boxes[i].Class == "person"){
				i_x = msg->bounding_boxes[i].xmin + (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)/2;
				i++;
				if((abs(get_last_person_y() - i_x)) < diff_y){
					diff_y = abs(get_last_person_y() - i_x);
					box_num = i;	
				}
			}
		}
	}
	else{
		/**
		no person to track so get the most centered person in the image as
		the person to track
		 */
		for(int i = 0; i < msg->bounding_boxes.size();i++){
			if(msg->bounding_boxes[i].Class == "person"){
				set_first_person("true");
				first_person = "true";
				i_x = msg->bounding_boxes[i].xmin + (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)/2;
				i++;
				if((abs(Cy_center - i_x)) < diff_y){
					diff_y = abs(Cy_center - i_x);
					box_num = i;
				}
			}
		}
	}
	if(box_num > 0){ //Only if we have a person to track
		for(int i = 0; i < msg->bounding_boxes.size();i++){
			if(msg->bounding_boxes[i].Class == "person"){
				box_num--;
				if(box_num == 0){
					Area_ref = ((msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin)*(msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin));
					Cy = msg->bounding_boxes[i].xmin + (msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin)/2;
					Cz = msg->bounding_boxes[i].ymin + (msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin)/2;
					set_last_person_y(Cy);
				}
			}
		}
	}
	if(Cy != 0 && Cz != 0){
		//CONVERT FROM PIXEL ERROR TO VELOCITY ERROR
		err_x = (Area_id - Area_ref) / Area_id;
		err_y = (Cy_center - Cy + 35) / Cy_center;
		err_z = (Cz_center - Cz) / Cz_center;
		//PID X AXES
		output_x = calculate_pid(err_x,fb,0);
		previous[0] = err_x;
		//PID Y AXES
		output_y = calculate_pid(err_y,lr,1);
		previous[1] = err_y;
		//PID Z AXES
		output_z = calculate_pid(err_z,ud,2);
		previous[2] = err_z;
		//SEND COMMANDS
		send_command(output_x,output_y,output_z);
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
