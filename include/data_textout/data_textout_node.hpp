#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <string>
#include <fstream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <stdexcept>
#include <mutex>

#define DEG2RAD(x) ((x)*M_PI/180)  // 度からラジアン
#define RAD2DEG(x) ((x)*180/M_PI)  // ラジアンから度

std::mutex m;

//クオータニオンからRPYに変換
void geometry_quat_to_rpy(
  double& roll,
  double& pitch,
  double& yaw,
  geometry_msgs::Quaternion geometry_quat)
{
  tf::Quaternion quat;
  quaternionMsgToTF(geometry_quat, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

struct Pose2D {
  double tx;                           // 並進x
  double ty;                           // 並進y
  double th;                           // 回転角(度)
  double Rmat[2][2];                   // 姿勢の回転行列

  Pose2D() : tx(0), ty(0), th(0) {
    for(int i=0;i<2;i++) {
      for(int j=0;j<2;j++) {
        Rmat[i][j] = (i==j)? 1.0:0.0;
      }
    }
  }

  Pose2D(double tx, double ty, double th) {
    this->tx = tx;
    this->ty = ty;
    this->th = th;
    calRmat();
  }

  void calRmat(){
    double a = DEG2RAD(th);
    Rmat[0][0] = Rmat[1][1] = std::cos(a);
    Rmat[1][0] = std::sin(a);
    Rmat[0][1] = -Rmat[1][0];
  }
  
  void setPose(double x, double y, double a) {
    tx = x;
    ty = y;
    th = a;
    calRmat();
  }
};

class Listener {
public:
  ros::NodeHandle nh;
  ros::Subscriber sub_odom, sub_scan_front, sub_scan_left, sub_scan_right;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub_image;

  geometry_msgs::Pose2D pos;
  sensor_msgs::LaserScan scan_front, scan_left, scan_right;
  cv::Mat image;

  int stamp;
  int scan_front_num, scan_left_num, scan_right_num;
  std::vector<bool> flag_front, flag_left, flag_right;

  std::ofstream outputfile;
  std::string filename;

  std::string scan_front_name, scan_left_name, scan_right_name, odom_name, image_name;

  tf::TransformListener TFlistener;
  Pose2D pose_front, pose_left, pose_right;

  Listener(std::string filename_) : stamp(0), nh("~"), it(nh) {
    nh.getParam("scan_front_name", scan_front_name);
    nh.getParam("scan_left_name", scan_left_name);
    nh.getParam("scan_right_name", scan_right_name);
    nh.getParam("odom_name", odom_name);
    nh.getParam("image_name", image_name);

    sub_scan_front = nh.subscribe(scan_front_name, 1, &Listener::callback_scan_front, this);
    sub_scan_left = nh.subscribe(scan_left_name, 1, &Listener::callback_scan_left, this);
    sub_scan_right = nh.subscribe(scan_right_name, 1, &Listener::callback_scan_right, this);
    sub_odom = nh.subscribe(odom_name, 1, &Listener::callback_odom, this);
    sub_image = it.subscribe(image_name, 1, &Listener::callback_image, this);

    filename = filename_;
    outputfile.open(filename, std::ios::out);
  }
  ~Listener() {
    outputfile.close();
  }

  void loop(int loop_rate_hz);
  void callback_odom(const nav_msgs::Odometry &odom_msg);
  void callback_scan_front(const sensor_msgs::LaserScan &scan_msg);
  void callback_scan_left(const sensor_msgs::LaserScan &scan_msg);
  void callback_scan_right(const sensor_msgs::LaserScan &scan_msg);
  void callback_image(const sensor_msgs::ImageConstPtr& msg);
  void write2file();

  //////

  void writeFormat() {
    outputfile << "Timestamp Odom_x Odom_y Odom_th Image_name" << std::endl;
    outputfile << "LidarFront_num LidarFront1_x LidarFront1_y LidarFront2_x LidarFront2_y ..." << std::endl;
    outputfile << "LidarLeft_num LidarLeft1_x LidarLeft1_y LidarLeft2_x LidarLeft2_y ..." << std::endl;
    outputfile << "LidarRight_num LidarRight1_x LidarRight1_y LidarRight2_x LidarRight2_y ..." << std::endl;
    return;
  }

  void getTransform() {
    tf::StampedTransform transform_front, transform_left, transform_right;
    try{
      TFlistener.waitForTransform("base_link", "laser_front", ros::Time(0), ros::Duration(3.0));
      TFlistener.lookupTransform("base_link", "laser_front", ros::Time(0), transform_front);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    try{
      TFlistener.waitForTransform("base_link", "laser_left", ros::Time(0), ros::Duration(3.0));
      TFlistener.lookupTransform("base_link", "laser_left", ros::Time(0), transform_left);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    try{
      TFlistener.waitForTransform("base_link", "laser_right", ros::Time(0), ros::Duration(3.0));
      TFlistener.lookupTransform("base_link", "laser_right", ros::Time(0), transform_right);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
 
    tf::Quaternion quat;
    double front_roll, front_pitch, front_yaw;
    quat = transform_front.getRotation();
    tf::Matrix3x3(quat).getRPY(front_roll, front_pitch, front_yaw);
    double left_roll, left_pitch, left_yaw;
    quat = transform_left.getRotation();
    tf::Matrix3x3(quat).getRPY(left_roll, left_pitch, left_yaw);
    double right_roll, right_pitch, right_yaw;
    quat = transform_right.getRotation();
    tf::Matrix3x3(quat).getRPY(right_roll, right_pitch, right_yaw);

    pose_front.setPose(transform_front.getOrigin().x(), transform_front.getOrigin().y(), RAD2DEG(front_yaw));
    pose_left.setPose(transform_left.getOrigin().x(), transform_left.getOrigin().y(), RAD2DEG(left_yaw));
    pose_right.setPose(transform_right.getOrigin().x(), transform_right.getOrigin().y(), RAD2DEG(right_yaw));

    std::cout << "pose_front = " << pose_front.tx << "," << pose_front.ty << "," << pose_front.th << std::endl;
    std::cout << "pose_left = " << pose_left.tx << "," << pose_left.ty << "," << pose_left.th << std::endl;
    std::cout << "pose_right = " << pose_right.tx << "," << pose_right.ty << "," << pose_right.th << std::endl;
  }
};
