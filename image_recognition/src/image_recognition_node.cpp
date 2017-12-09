
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
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
double th2 = 2.1e08;
double th3 = 1e08;
double th4 = 1.2e08;
double th5 = 2e08;

Mat templ1_1st;
 Mat templ1_2nd;
 Mat templ1_3rd;

Mat templ2_1st;
  Mat templ2_2nd;
 Mat templ2_3rd;

Mat templ3_1st;
  Mat templ3_2nd;
 Mat templ3_3rd;

Mat templ4_1st;
  Mat templ4_2nd;
 Mat templ4_3rd;

Mat templ5_1st;
  Mat templ5_2nd;
 Mat templ5_3rd;

 bool dis = false;
 Point matchLoc;
 Mat templ;

Mat result;


void MatchingMethod(Mat img, Mat templ1, Mat templ2, Mat templ3, int type)
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

   Mat result3;
  result3.create( result_rows, result_cols, CV_32FC1 );

  matchTemplate( img, templ3, result3, match_method );
 
  double minVal3=10; double maxVal3=-10; Point minLoc3; Point maxLoc3;


  minMaxLoc( result3, &minVal3, &maxVal3, &minLoc3, &maxLoc3, Mat() );   
  









double minVal;

if (minVal2<minVal1&&minVal2<minVal3){
   minVal = minVal2;
   matchLoc = minLoc2;
   templ = templ2;
   result = result2;
}
else if (minVal3<minVal1&&minVal3<minVal2){
  minVal = minVal3;
  matchLoc = minLoc3;
  templ = templ3;
  result = result3;
}
else{
  minVal = minVal1;
  matchLoc = minLoc1;
  templ = templ1;
  result = result1;
}

 std::cout<<"type"<<type<<"Min  "<<minVal<<std::endl;



  dis = false;
  switch(type){
    case 1: dis=minVal < th1; break;
    case 2: dis=minVal < th2; break;
    case 3: dis=minVal < th3; break;
    case 4: dis=minVal < th4; break;
    case 5: dis=minVal < th5; break;
  }

  if(dis){
  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
 // rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
   geometry_msgs::Pose message;
   message.position.x = matchLoc.x;
   message.position.y = matchLoc.y;
   message.position.z = 0;
   message.orientation.x  = matchLoc.x + templ.cols;
   message.orientation.y = matchLoc.y + templ.rows;
   message.orientation.z = 0;
   message.orientation.w =0;

   pub_cmd.publish(message);

}


  imshow( image_window, img_display );
 // imshow( result_window, result );

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

      
     MatchingMethod(img,templ1_1st,templ1_2nd,templ1_3rd,1);
     cv::waitKey(10);

     MatchingMethod(img,templ2_1st,templ2_2nd,templ2_3rd,2);
     cv::waitKey(10);

     MatchingMethod(img,templ3_1st,templ3_2nd,templ3_3rd,3);
     cv::waitKey(10);

     MatchingMethod(img,templ4_1st,templ4_2nd,templ4_3rd,4);
     cv::waitKey(10);

     MatchingMethod(img,templ5_1st,templ5_2nd,templ5_3rd,5);
     cv::waitKey(10);
     


}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_recognition");
    ros::NodeHandle n("~");
    pub_cmd = n.advertise<geometry_msgs::Pose>("image_reconition_result",100);

  

    // to be modified
    Mat original_templ1 = imread ("/home/shengrui/catkin_ws/src/picture_sample/pic001.jpg");
    Mat original_templ2 = imread ("/home/shengrui/catkin_ws/src/picture_sample/pic002.jpg");
    
    Mat original_templ3 = imread ("/home/shengrui/catkin_ws/src/picture_sample/pic003.jpg");
   flip(original_templ3, original_templ3, 1); 
   

    Mat original_templ4 = imread ("/home/shengrui/catkin_ws/src/picture_sample/pic004.jpg");
    Mat original_templ5 = imread ("/home/shengrui/catkin_ws/src/picture_sample/pic005.jpg");  
   flip(original_templ5, original_templ5, 1); 
   


    // put templ_2nd as global variable, becuase we cannot pass it to call_back function
    resize(original_templ1,templ1_1st,Size(250,250),0,0);
    resize(original_templ1, templ1_2nd, Size(120,120), 0, 0);
    namedWindow( "templ1_2nd", CV_WINDOW_AUTOSIZE );
    imshow("templ1_2nd",templ1_2nd);
    resize(original_templ1, templ1_3rd, Size(150,150), 0, 0);
    namedWindow( "templ1_3rd", CV_WINDOW_AUTOSIZE );
    imshow("templ1_3rd",templ1_3rd);

    
    resize(original_templ2,templ2_1st,Size(250,250),0,0);
    resize(original_templ2, templ2_2nd, Size(120,120), 0, 0);
    namedWindow( "templ2_2nd", CV_WINDOW_AUTOSIZE );
    imshow("templ2_2nd",templ2_2nd);
    resize(original_templ2, templ2_3rd, Size(150,150), 0, 0);
    namedWindow( "templ2_3rd", CV_WINDOW_AUTOSIZE );
    imshow("templ2_3rd",templ2_3rd);


    resize(original_templ3,templ3_1st,Size(250,250),0,0);
    resize(original_templ3, templ3_2nd, Size(120,120), 0, 0);
    namedWindow( "templ3_2nd", CV_WINDOW_AUTOSIZE );
    imshow("templ3_2nd",templ3_2nd);
    resize(original_templ3, templ3_3rd, Size(150,150), 0, 0);
    namedWindow( "templ3_3rd", CV_WINDOW_AUTOSIZE );
    imshow("templ3_3rd",templ3_3rd);

    resize(original_templ4,templ4_1st,Size(250,250),0,0);
    resize(original_templ4, templ4_2nd, Size(120,120), 0, 0);
    namedWindow( "templ4_2nd", CV_WINDOW_AUTOSIZE );
    imshow("templ4_2nd",templ4_2nd);
    resize(original_templ4, templ4_3rd, Size(150,150), 0, 0);
    namedWindow( "templ4_3rd", CV_WINDOW_AUTOSIZE );
    imshow("templ4_3rd",templ4_3rd);




    resize(original_templ5,templ5_1st,Size(250,250),0,0);
    resize(original_templ5, templ5_2nd, Size(120,120), 0, 0);
    namedWindow( "templ5_2nd", CV_WINDOW_AUTOSIZE );
    imshow("templ5_2nd",templ5_2nd);
    resize(original_templ5, templ5_3rd, Size(150,150), 0, 0);
    namedWindow( "templ5_3rd", CV_WINDOW_AUTOSIZE );
    imshow("templ5_3rd",templ5_3rd);
      

     namedWindow( image_window, CV_WINDOW_AUTOSIZE );
     namedWindow( result_window, CV_WINDOW_AUTOSIZE );

     namedWindow( image_window, CV_WINDOW_AUTOSIZE );
     namedWindow( result_window, CV_WINDOW_AUTOSIZE );

     
     



    ros::Subscriber sub = n.subscribe("/flipped_image", 100, img_callback);

  

    ros::spin();
    
    
}