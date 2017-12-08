#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
ros::Publisher pub_cmd;
//ros::Publisher pub_laser;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB16);
    Mat frame, frame_filtered;
    frame = bridge_ptr->image;
    if(frame.empty()) {
        ROS_INFO("Didn't get the cv_bridge image");
    }

    // Detecting Yellow
    inRange(frame, Scalar(240, 240, 15), Scalar(255, 255, 0), frame_filtered);
    imshow("filtered origin", frame);
//    imshow("filtered filtered", frame_filtered);


    waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_follower");
    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe("/flipped_image", 100, img_callback);
    pub_cmd = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 10);

    ros::spin();
    /*
    pub_laser = n.advertise<std_msgs::Bool>("follower_laser_switch", 100);

    ros::Rate loop_rate(0.4);
    while(ros::ok())
    {
        std_msgs::Bool laser_switch_bool;
        laser_switch_b}ool.data = false;
        pub_laser.publish(laser_switch_bool);
        ros::spinOnce();
        loop_rate.sleep();
    }
     */
}
