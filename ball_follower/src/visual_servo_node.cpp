#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "../include/pid.h"

using namespace cv;
using namespace std;
ros::Publisher pub_cmd;
//ros::Publisher pub_laser;
double vel_offset, dis_offset;
double Kp_angle, Kd_angle, Ki_angle, Kp_vel, Kd_vel, Ki_vel;
Scalar bgr_lower_bound, bgr_upper_bound;
arm_pid_instance_f32 angle_pid, vel_pid;
float angle_gap, distance_gap;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    Mat frame, frame_filtered;
    Mat dst;

    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);

    frame = bridge_ptr->image;
    if(frame.empty()) {
        cout << "Didn't get the cv_bridge image" << endl;
    }
    dst = Mat::zeros(frame.rows, frame.cols, CV_8UC3);

    // binary frame
    inRange(frame, bgr_lower_bound, bgr_upper_bound, frame_filtered);

    // Detecting Yellow contour
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(frame_filtered, contours, hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


    if (contours.empty()) {
        // if the ball is lost
        angle_gap = 0;
    }
    else {
        unsigned int idx = 0;
        unsigned int largest_idx = 0;
        double largest_area = 0;
        for ( ; idx < contours.size(); ++idx) {
            double area = contourArea(contours[idx], false);
            if (area > largest_area) {
                largest_area = area;
                largest_idx  = idx;
            }
        }

        Rect bound = boundingRect(contours[largest_idx]);
        rectangle(frame_filtered, bound, Scalar(100, 255, 100), 5, 8, 0);

        //imshow("filtered", frame_filtered);
        angle_gap = bound.x + bound.width / 2 - frame_filtered.cols / 2;
        distance_gap = (float)(sqrt(bound.area()) - dis_offset);
        cout << "x: " << angle_gap << endl;
        cout << "dis" << distance_gap << endl;
    }
    waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_follower");
    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe("/flipped_image", 100, img_callback);
    pub_cmd = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 100);

    int r_lower, g_lower, b_lower, r_upper, g_upper, b_upper;
    n.param("Kp_angle", Kp_angle, 0.01);
    n.param("Kd_angle", Kd_angle, 0.0);
    n.param("Ki_angle", Ki_angle, 0.0);
    n.param("vel_offset", vel_offset, 1.0);
    n.param("dis_offset", dis_offset, 60.0);
    n.param("Kp_vel", Kp_vel, 0.0);
    n.param("Kd_vel", Kd_vel, 0.0);
    n.param("Ki_vel", Ki_vel, 0.0);
    n.param("lower_bound_color_b", b_lower, 0);
    n.param("lower_bound_color_g", g_lower, 180);
    n.param("lower_bound_color_r", r_lower, 180);
    n.param("upper_bound_color_b", b_upper, 80);
    n.param("upper_bound_color_g", g_upper, 255);
    n.param("upper_bound_color_r", r_upper, 255);
    
    bgr_lower_bound << b_lower, g_lower, r_lower;
    bgr_upper_bound << b_upper, g_upper, r_upper;

    angle_pid.Kp = (float)Kp_angle;
    angle_pid.Ki = (float)Ki_angle;
    angle_pid.Kd = (float)Kd_angle;
    arm_pid_init_f32(&angle_pid, 0);

    vel_pid.Kp = (float)Kp_vel;
    vel_pid.Ki = (float)Ki_vel;
    vel_pid.Kd = (float)Kd_vel;
    arm_pid_init_f32(&vel_pid, 0);
    distance_gap = 0;

    /**
     * 100 Hz control loop
     */
    ros::Rate loop_rate(100);
    geometry_msgs::Twist feedback_output;
    while(ros::ok())
    {
        feedback_output.linear.x = vel_offset + arm_pid_f32(&vel_pid, distance_gap);
        feedback_output.angular.z = arm_pid_f32(&angle_pid, angle_gap);
        pub_cmd.publish(feedback_output);

        loop_rate.sleep();

        ros::spinOnce();
    }

}

// iterate through all the top-level contours,
// draw each connected component with its own random color
//        for ( ; idx >= 0; idx = hierarchy[idx][0]) {
//            Scalar color( rand()&255, rand()&255, rand()&255 );
//            drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy);
//        }
//        drawContours(dst, contours, largest_idx, Scalar(255, 255, 0), CV_FILLED, 8);
//        namedWindow( "Components", 1 );
//        imshow("Components", dst );