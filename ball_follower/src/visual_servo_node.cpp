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
double Kp_angle, Kd_angle, Ki_angle, Kp_vel, Kd_vel, Ki_vel;
Scalar lower_bound, upper_bound;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
    Mat frame, frame_filtered;
    frame = bridge_ptr->image;
    if(frame.empty()) {
        ROS_INFO("Didn't get the cv_bridge image");
    }

    // Detecting Yellow
    inRange(frame, lower_bound, upper_bound, frame_filtered);
//    imshow("filtered origin", frame);
    imshow("filtered filtered", frame_filtered);


    waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_follower");
    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe("/flipped_image", 100, img_callback);
    pub_cmd = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 10);

    int r_lower, g_lower, b_lower, r_upper, g_upper, b_upper;
    n.param("Kp_angle", Kp_angle, 5.0);
    n.param("Kd_angle", Kd_angle, 0.0);
    n.param("Ki_angle", Ki_angle, 0.0);
    n.param("Kp_vel", Kp_vel, 5.0);
    n.param("Kd_vel", Kd_vel, 0.0);
    n.param("Ki_vel", Ki_vel, 0.0);
    n.param("lower_bound_color_b", b_lower, 0);
    n.param("lower_bound_color_g", g_lower, 180);
    n.param("lower_bound_color_r", r_lower, 180);
    n.param("upper_bound_color_b", b_upper, 80);
    n.param("upper_bound_color_g", g_upper, 255);
    n.param("upper_bound_color_r", r_upper, 255);
    
    lower_bound << b_lower, g_lower, r_lower;
    upper_bound << b_upper, g_upper, r_upper;
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
