#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
ros::Publisher pub_flipped_image;

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
//    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);

    Mat frame, frame_flipped;
    frame = bridge_ptr->image;
    flip(frame, frame_flipped, 1);
    imshow("flipped", frame);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_flipped).toImageMsg();
    pub_flipped_image.publish(msg);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flip_image");
    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe("/vrep/image", 100, img_callback);
    pub_flipped_image = n.advertise<sensor_msgs::Image>("/flipped_image", 100);

    ros::Rate loop_rate(10);
    while (ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
//    ros::spin();
}
