#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace ros;
using namespace std;

class LineFollower{
private:
    NodeHandle nh;
    Publisher pub;
    Subscriber sub;
    geometry_msgs::Twist twist;
    cv_bridge::CvImagePtr cv_ptr;
    Mat original, hsv, mask, res;
    int lin_error_x, ang_error_z, width;
public:
    LineFollower();
    void callback(const sensor_msgs::Image::ConstPtr& msg);
    ~LineFollower();
};

LineFollower::LineFollower(){
    pub = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel/", 10);
    sub = nh.subscribe("/robot1/camera1/image_raw", 1000, &LineFollower::callback, this);
}

LineFollower::~LineFollower(){
    destroyWindow("Original");
    destroyWindow("HSV");
    destroyWindow("Mask");
    destroyWindow("Bitwise AND");
}
void LineFollower::callback(const sensor_msgs::Image::ConstPtr& msg){
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    width = cv_ptr->image.cols;
    //Crop image to focus on the lines in the folower part of the image (240, 320)
    Rect bounds(0,0,cv_ptr->image.cols,cv_ptr->image.rows);
    Rect r(0,20,320,240); //to dimish height of image as we're only interested in using lines that are near the robot
    original = cv_ptr->image(r & bounds);
    //Convert from BGR to HSV
    cvtColor(original, hsv, COLOR_BGR2HSV);
    //Define lower and upper bounds of color range and threshold to get only yellow colors
    inRange(hsv, Scalar(20,100,100), Scalar(50,255,255), mask); //yellow
    //Calculate centroid of the blob of binary image using ImageMoments
    Moments m = moments(mask,false);
    Point p(m.m10/m.m00, m.m01/m.m00);
    // show the image with a point mark at the centroid
    circle(original, p, 10, Scalar(0,0,255), -1);
    //Bitwise-AND mask and original image
    bitwise_and(original,original,res, mask);
    // Update GUI Window
    imshow("Original", original);
    imshow("HSV", hsv);
    imshow("Mask",mask);
    imshow("Bitwise AND",res);
    waitKey(1);
    //Calculation to correct robot trajectory by rotating it to follow the line
    lin_error_x = p.x - (width / 2);
    ang_error_z = - (lin_error_x / 100);
    //Send velocity
    twist.linear.x = 0.2;
    twist.angular.z = ang_error_z;
    pub.publish(twist);
}

int main(int argc, char** argv) {
    init(argc, argv, "line_follower_node");
	LineFollower line_follower;
	spin();
	return 0;
}
