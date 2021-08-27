#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <string.h>
#include <fstream>
#include <iomanip>
#include <vector>
#include <stdexcept>

class Listener {
public:
  ros::NodeHandle nh;
  ros::Subscriber sub_odom, sub_scan;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub_image;

  geometry_msgs::Pose2D pos;
  sensor_msgs::LaserScan scan;
  cv::Mat image;

  int stamp;
  int data_num;
  std::vector<bool> flag;

  std::ofstream outputfile;
  std::string filename;

  Listener(std::string filename_) : stamp(0), data_num(0), it(nh) {
    sub_odom = nh.subscribe("/ypspur_ros/odom", 1, &Listener::callback_odom, this);
    sub_scan = nh.subscribe("/scan", 1, &Listener::callback_scan, this);
    sub_image = it.subscribe("/image_raw", 1, &Listener::imageCb, this);

    filename = filename_;
    outputfile.open(filename, std::ios::out);
  }
  ~Listener() {
    outputfile.close();
  }

  void loop(int loop_rate_hz);
  void callback_odom(const nav_msgs::Odometry &odom_msg);
  void callback_scan(const sensor_msgs::LaserScan &scan_msg);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void write2file();

};
