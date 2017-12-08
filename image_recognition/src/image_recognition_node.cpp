
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
#include <math.h>
#include <cmath>
// #include "template_matching.h"
using namespace cv;
ros::Publisher pub_cmd;



char* image_window = "Source Image";
char* result_window = "Result window";

int match_method = CV_TM_SQDIFF ;
int max_Trackbar = 5;
double th1 = 1.5e08;

 Mat templ_2nd;
 Mat templ_3rd;

void MatchingMethod(Mat img, Mat templ1, Mat templ2, int type)
{
  /// Source image to display



  Mat img_display;
  img.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  img.cols - templ1.cols + 1;
  int result_rows = img.rows - templ1.rows + 1;
  

  Mat result1;
  result1.create( result_rows, result_cols, CV_32FC1 );

  matchTemplate( img, templ1, result1, match_method );
 
  double minVal1=10; double maxVal1=-10; Point minLoc1; Point maxLoc1;


  minMaxLoc( result1, &minVal1, &maxVal1, &minLoc1, &maxLoc1, Mat() );

 
  Mat result2;
  result1.create( result_rows, result_cols, CV_32FC1 );

  matchTemplate( img, templ2, result2, match_method );
 
  double minVal2=10; double maxVal2=-10; Point minLoc2; Point maxLoc2;


  minMaxLoc( result2, &minVal2, &maxVal2, &minLoc2, &maxLoc2, Mat() );   
  

 std::cout<<"Min1"<<minVal1<<std::endl;
 std::cout<<"Min2"<<minVal2<<std::endl;


Point matchLoc;
Mat templ;
Mat result;
double minVal;

if (minVal2<minVal1){
   minVal = minVal2;
   matchLoc = minLoc2;
   templ = templ2;
   result = result2;
}
else{
  minVal = minVal1;
  matchLoc = minLoc1;
  templ = templ1;
  result = result1;
}


  double th;
  switch(type){
    case 1: th = th1; break;
  }

  if(minVal< th){
  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
 
}


  imshow( image_window, img_display );
  imshow( result_window, result );

  waitKey(10);

  return;
}



void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
	cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	Mat img = bridge_ptr->image;
	if(img.empty()) {
        ROS_INFO("Didn't get the cv_bridge image");
        std::cout<<"didn't received cv_bridge image"<<std::endl;
    }

 //    waitKey(10);

    // to be modified
    // Mat original_templ = imread ("/home/shengrui/catkin_ws/src/picture_sample/pic001.jpg");
    
    // Mat templ_2nd;
    // //pyrDown( original_templ, templ_2nd, Size( original_templ.cols/2, original_templ.rows/2) );
    // resize(original_templ, templ_2nd, Size(120,120), 0, 0);
    // namedWindow( "templ_2nd", CV_WINDOW_AUTOSIZE );
    // imshow("templ_2nd",templ_2nd);
    
    // Mat templ_3rd;
    // resize(original_templ, templ_3rd, Size(150,150), 0, 0);
    // namedWindow( "templ_3rd", CV_WINDOW_AUTOSIZE );
    // imshow("templ_3rd",templ_3rd);
    


    
      

    //  namedWindow( image_window, CV_WINDOW_AUTOSIZE );
    //  namedWindow( result_window, CV_WINDOW_AUTOSIZE );

     

     


 

     MatchingMethod(img,templ_2nd,templ_3rd,1);

     cv::waitKey(10);



}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_recognition");
    ros::NodeHandle n("~");

  

    // to be modified
    Mat original_templ = imread ("/home/shengrui/catkin_ws/src/picture_sample/pic001.jpg");
    
    
          
    
   
    // put templ_2nd as global variable, becuase we cannot pass it to call_back function
    resize(original_templ, templ_2nd, Size(120,120), 0, 0);
    namedWindow( "templ_2nd", CV_WINDOW_AUTOSIZE );
    imshow("templ_2nd",templ_2nd);
    
   
    resize(original_templ, templ_3rd, Size(150,150), 0, 0);
    namedWindow( "templ_3rd", CV_WINDOW_AUTOSIZE );
    imshow("templ_3rd",templ_3rd);
    


    
      

     namedWindow( image_window, CV_WINDOW_AUTOSIZE );
     namedWindow( result_window, CV_WINDOW_AUTOSIZE );

     namedWindow( image_window, CV_WINDOW_AUTOSIZE );
     namedWindow( result_window, CV_WINDOW_AUTOSIZE );

     
     



    ros::Subscriber sub = n.subscribe("/flipped_image", 100, img_callback);
    // pub_cmd = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 10);

    ros::spin();
    
    
}