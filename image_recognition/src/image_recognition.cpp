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

Mat img; Mat templ; Mat result;

int match_method = CV_TM_SQDIFF;
int max_Trackbar = 5;

void MatchingMethod( int, void* )
{
  /// Source image to display
  Mat img_display;
  img.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

  /// Do the Matching and Normalize
  matchTemplate( img, templ, result, match_method );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }
   
  /// Show me what you got
  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  imshow( image_window, img_display );
  imshow( result_window, result );

  return;
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
	cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB16)
	img = bridge_ptr->image;
	if(frame.empty()) {
        ROS_INFO("Didn't get the cv_bridge image");
    }


    // to be modified
    templ = imread ("/home/shengrui/catkin_ws/src/final_picture/pic001.jpg");
    

     namedWindow( image_window, CV_WINDOW_AUTOSIZE );
     namedWindow( result_window, CV_WINDOW_AUTOSIZE );



     char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
     createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );


    MatchingMethod(0,0);



}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_recognition");
    ros::NodeHandle n("~");

    ros::Subscriber sub = n.subscribe("/flipped_image", 100, img_callback);
    // pub_cmd = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 10);

    ros::spin();
    
    
}