#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <geometry_msgs/Pose2D.h>

using namespace std;
using namespace cv;

//White ROI
int roi_x = 2200;
int roi_y = 350;
int roi_width = 1200;
int roi_height = 280;

//White Color Setting
int white_b_low = 95;
int white_b_high = 150;
int white_g_low = 95;
int white_g_high = 150;
int white_r_low = 95;
int white_r_high =150;

int main(int argc, char** argv)
{
  // Node Name : white_detect
  ros::init(argc, argv, "white_detect");

  ros::NodeHandle nh;

  /*  fitLine_msg
    Type : geometry_msgs/Pose2D

    msg.x = b.x   (x value of point on line)
    msg.y = b.y   (y value of point on line)
    msg.theta = m (Radian degree of line)
  */
  geometry_msgs::Pose2D fitLine_msg;
  ros::Publisher pub_fitLine = nh.advertise<geometry_msgs::Pose2D>("/white_detect/white_line_pos", 1000);

  // Set Publishers & Sublscribers
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/white_detect/white_detect_img", 1);
  image_transport::Publisher pub2 = it.advertise("/white_detect/white_img", 1);
  image_transport::Subscriber sub = it.subscribe("/mindvision1/image", 1,
  [&](const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge Error ! : %s", e.what());
      return;
    }

    // Recognize Slope angle tolerance
    int slope_tor = 60;
    // Recognize Slope angle treshold (-45 deg ~ 45deg)
    double slope_treshold = (90 - slope_tor) * CV_PI / 180.0;

    Mat img_hsv, white_mask, img_white, img_edge, test;
    Mat frame = cv_ptr->image;
    Mat grayImg, blurImg, edgeImg, copyImg;

    int x_left_lim = frame.cols, x_right_lim = 0, x_lim;

    Point pt1, pt2;
    vector<Vec4i> lines, selected_lines, right_lines, left_lines;
    vector<double> slopes;
    vector<Point> left_pts, right_pts;
    Vec4d left_fit_line, right_fit_line, fit_line;

    Rect bounds(0, 0, frame.cols, frame.rows);
    Rect roi(roi_x, roi_y, roi_width, roi_height);
    frame = frame(bounds & roi);

    // Color Filtering

    inRange(frame, Scalar(white_b_low, white_g_low, white_r_low) , Scalar(white_b_high, white_g_high, white_r_high), white_mask);
    bitwise_and(frame, frame, img_white, white_mask);
    img_white.copyTo(copyImg);

    // Canny Edge Detection
    cvtColor(img_white, img_white, COLOR_BGR2GRAY);
    Canny(img_white, img_edge, 50, 150);

    // Line Dtection
    HoughLinesP(img_edge, lines, 1, CV_PI / 180 , 50 ,20, 10);

    //cout << "slope treshol : " << slope_treshold << endl;

    for(size_t i = 0; i < lines.size(); i++)
    {
      Vec4i line = lines[i];
      pt1 = Point(line[0] , line[1]);
      pt2 = Point(line[2], line[3]);

      double slope = (static_cast<double>(pt1.y) - static_cast<double>(pt2.y)) / (static_cast<double>(pt1.x) - static_cast<double>(pt2.x) );
      //cout << slope << endl;

      cv::line(frame, Point(pt1.x, pt1.y) , Point(pt2.x , pt2.y) , Scalar(0,255,0) , 2 , 8);
      if(abs(slope) >= slope_treshold)
      {
        selected_lines.push_back(line);
        if(pt1.x < x_left_lim )
          x_left_lim = pt1.x;

        else if(pt1.x > x_right_lim)
          x_right_lim = pt1.x;

        if(pt2.x < x_left_lim)
          x_left_lim = pt2.x;

        else if(pt2.x > x_right_lim)
          x_right_lim = pt2.x;
      }
    }

    if(selected_lines.size() > 0)
    {
      x_lim = (x_left_lim + x_right_lim) / 2;
      for(size_t i = 0; i < selected_lines.size(); i++)
      {
        pt1 = Point(selected_lines[i][0] , selected_lines[i][1]);
        pt2 = Point(selected_lines[i][2], selected_lines[i][3]);

        if(pt1.x <= x_lim && pt2.x <= x_lim)
        {
          left_lines.push_back(selected_lines[i]);
          left_pts.push_back(pt1);
          left_pts.push_back(pt2);
        }


        else if(pt1.x > x_lim && pt2.x > x_lim)
        {
          right_lines.push_back(selected_lines[i]);
          right_pts.push_back(pt1);
          right_pts.push_back(pt2);
        }

      }

      if(left_pts.size() > 0 && right_pts.size() > 0)
      {
        fitLine(right_pts, right_fit_line, DIST_L2, 0,0.1, 0.01);
        fitLine(left_pts, left_fit_line, DIST_L2, 0, 0.01, 0.01);

        double r_m = right_fit_line[1] / right_fit_line[0];
        Point r_b = Point(right_fit_line[2], right_fit_line[3]);

        double l_m = left_fit_line[1] / left_fit_line[0];
        Point l_b = Point(left_fit_line[2], left_fit_line[3]);

        double vm = (r_m + l_m)/2;
        double vx = static_cast<double>(((r_m * r_b.x) - (l_m * l_b.x) - r_b.y + l_b.y) / (r_m - l_m));
        double vy = (vm * vx) + l_m;


        ROS_INFO("vx :%f" , vx);

        int pt1_y = frame.rows;
        int pt2_y = 0;

        double pt1_x = ((pt1_y - l_b.y) / l_m) + l_b.x;
        double pt2_x = ((pt2_y - l_b.y) / l_m) + l_b.x;


        //cout << "slope : " << (static_cast<double>(pt1_y) - static_cast<double>(pt2_y)) / (static_cast<double>(pt1_x) - static_cast<double>(pt2_x)) << endl;

        line(frame, Point(pt1_x, pt1_y) , Point(pt2_x , pt2_y) , Scalar(0,0,255) , 2 , 8);

        fitLine_msg.x = l_b.x;
        fitLine_msg.y = l_b.y;
        fitLine_msg.theta = l_m;



      for(size_t i = 0; i < selected_lines.size(); i++)
        {
          //cout << "i : " << i << endl;
          Vec4i I = selected_lines[i];
          line(frame, Point(I[0], I[1]), Point(I[2], I[3]) , Scalar(255,0,0) , 2 , 8);
        }
      }

    }



    //ROS_INFO("cols : %d , rows : %d" , frame.cols, frame.rows);
    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    sensor_msgs::ImagePtr pub_msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", copyImg).toImageMsg();
    pub.publish(pub_msg);
    pub2.publish(pub_msg2);
    pub_fitLine.publish(fitLine_msg);

  });



  ros::spin();

  return 0;

}
