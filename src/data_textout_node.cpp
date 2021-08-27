#include "data_textout/data_textout_node.hpp"

#define DEG2RAD(x) ((x)*M_PI/180)  // 度からラジアン
#define RAD2DEG(x) ((x)*180/M_PI)  // ラジアンから度

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

////////////////////////////////////////////////////////////////////////////////

// odomのコールバック
void Listener::callback_odom(const nav_msgs::Odometry &odom_msg) {
  double roll, pitch, yaw;

  pos.x = odom_msg.pose.pose.position.x;
  pos.y = odom_msg.pose.pose.position.y;
  geometry_quat_to_rpy(roll, pitch, yaw, odom_msg.pose.pose.orientation);
  pos.theta = RAD2DEG(yaw); // zではなく角度を保存

  return;
}

////////////////////////////////////////////////////////////////////////////////

// scanのコールバック
void Listener::callback_scan(const sensor_msgs::LaserScan &scan_msg) {
  int count = 0;
  double angle = -135.0;

  flag.clear();

  for(int i = 0; i < scan_msg.ranges.size(); i++) {
    // 無限大は無視 30m以内のセンサ値のみ使用
    //if(std::isinf(scan_msg.ranges[i]) == 0 && scan_msg.ranges[i] < 30)
    if(angle > -90 && angle < 90 && std::isinf(scan_msg.ranges[i]) == 0 && scan_msg.ranges[i] < 30)
    {
      count++;
      flag.push_back(true);
    } else {
      flag.push_back(false);
    }
    angle += 0.25;
  }
  data_num = count;
  scan = scan_msg;

  return;
}

////////////////////////////////////////////////////////////////////////////////

void Listener::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cvtColor(cv_ptr->image, image, cv::COLOR_BGR2GRAY);
}

////////////////////////////////////////////////////////////////////////////////

void Listener::write2file() {
  if(stamp == 0) {
    stamp++;
    return;
  }

  // タイムスタンプ
  outputfile << "LASERSCAN " << stamp << " " << data_num << " " << std::flush;
  // スキャンデータ
  double angle = -135.0;
  for(int i=0; i<scan.ranges.size(); i++) {
    if(flag[i] == true) {
      outputfile << angle << " " << scan.ranges[i] << " " << std::flush;
    }
    angle += 0.25;
  }
  // オドメトリデータ
  outputfile << pos.x << " " << pos.y << " " << pos.theta << std::endl;

  std::ostringstream oss;
  oss << stamp;
  cv::imwrite("image" + oss.str() + ".jpg", image);

  stamp++;

  ROS_INFO("Output sensor data to file.");

  return;
}

////////////////////////////////////////////////////////////////////////////////

void Listener::loop(int loop_rate_hz) {
  ros::Rate loop_rate(loop_rate_hz);

  while(ros::ok()){
    ros::spinOnce();
    write2file();
    loop_rate.sleep();
  }
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "data_textout");

  if(argc != 3){
    std::cerr << "Usage: roslaunch data_textout data_textout.launch [filename] [frequency(Hz)]" << std::endl;
    return -1;
  }

  std::string filename = argv[1];
  int loop_rate_hz = atoi(argv[2]);

  Listener lis(filename);
  lis.loop(loop_rate_hz);

  return 0;
}
