#include "data_textout/data_textout_node.hpp"

// odomのコールバック
void Listener::callback_odom(const nav_msgs::Odometry &odom_msg) {
  std::lock_guard<std::mutex> lock(m);

  double roll, pitch, yaw;

  pos.x = odom_msg.pose.pose.position.x;
  pos.y = odom_msg.pose.pose.position.y;
  geometry_quat_to_rpy(roll, pitch, yaw, odom_msg.pose.pose.orientation);
  pos.theta = RAD2DEG(yaw); // zではなく角度を保存

  return;
}

// scan_frontのコールバック
void Listener::callback_scan_front(const sensor_msgs::LaserScan &scan_msg) {
  std::lock_guard<std::mutex> lock(m);

  int count = 0;
  double angle = -135.0;

  flag_front.clear();

  for(int i = 0; i < scan_msg.ranges.size(); i++) {
    // 無限大は無視 0.1m〜30mのセンサ値のみ使用
    if(angle > -90 && angle < 90 && std::isinf(scan_msg.ranges[i]) == 0 && scan_msg.ranges[i] < 30 && scan_msg.ranges[i] > 0.1)
    {
      count++;
      flag_front.push_back(true);
    } else {
      flag_front.push_back(false);
    }
    angle += 0.25;
  }

  scan_front_num = count;
  scan_front = scan_msg;

  return;
}

// scan_leftのコールバック
void Listener::callback_scan_left(const sensor_msgs::LaserScan &scan_msg) {
  std::lock_guard<std::mutex> lock(m);
  
  int count = 0;
  double angle = 180.0;

  flag_left.clear();

  for(int i = 0; i < scan_msg.ranges.size(); i++) {
    if((angle > 135 || angle < -90) && std::isinf(scan_msg.ranges[i]) == 0 && scan_msg.ranges[i] < 30 && scan_msg.ranges[i] > 0.1)
    {
      count++;
      flag_left.push_back(true);
    } else {
      flag_left.push_back(false);
    }
    angle -= 0.2803738493;
  }

  scan_left_num = count;
  scan_left = scan_msg;

  return;
}

// scan_rightのコールバック
void Listener::callback_scan_right(const sensor_msgs::LaserScan &scan_msg) {
  std::lock_guard<std::mutex> lock(m);

  int count = 0;
  double angle = 180.0;

  flag_right.clear();
  
  for(int i = 0; i < scan_msg.ranges.size(); i++) {
    if((angle > 135 || angle < -90) && std::isinf(scan_msg.ranges[i]) == 0 && scan_msg.ranges[i] < 30 && scan_msg.ranges[i] > 0.1)
    {
      count++;
      flag_right.push_back(true);
    } else {
      flag_right.push_back(false);
    }
    angle -= 0.2803738493;
  }

  scan_right_num = count;
  scan_right = scan_msg;
  return;
}

// imageのコールバック
void Listener::callback_image(const sensor_msgs::ImageConstPtr& msg) {
  std::lock_guard<std::mutex> lock(m);

  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cvtColor(cv_ptr->image, image, cv::COLOR_BGR2GRAY);
  return;
}

////////////////////////////////////////////////////////////////////////////////

void Listener::write2file() {
  // 画像出力
  std::ostringstream oss;
  oss << stamp;
  cv::imwrite("image" + oss.str() + ".jpg", image);

  outputfile << stamp << " " << pos.x << " " << pos.y << " " << pos.theta << " image" + oss.str() + ".jpg" << std::endl;
  // front
  outputfile << scan_front_num << " " << std::flush;
  double angle = -135.0;
  for(int i=0; i<scan_front.ranges.size(); i++) {
    if(flag_front[i] == true) {
      outputfile << scan_front.ranges[i]*std::cos(DEG2RAD(angle)) + pose_front.tx << " " << scan_front.ranges[i]*std::sin(DEG2RAD(angle)) + pose_front.ty << " " << std::flush;
    }
    angle += 0.25;
  }
  outputfile << std::endl;

  // left
  outputfile << scan_left_num << " " << std::flush;
  angle = 180;
  for(int i=0; i<scan_left.ranges.size(); i++) {
    if(flag_left[i] == true) {
      double x = pose_left.Rmat[0][0]*scan_left.ranges[i]*std::cos(DEG2RAD(angle)) + pose_left.Rmat[0][1]*scan_left.ranges[i]*std::sin(DEG2RAD(angle)) + pose_left.tx;
      double y = pose_left.Rmat[1][0]*scan_left.ranges[i]*std::cos(DEG2RAD(angle)) + pose_left.Rmat[1][1]*scan_left.ranges[i]*std::sin(DEG2RAD(angle)) + pose_left.ty;
      outputfile << x << " " << y << " " << std::flush;
    }
    angle -= 0.2803738493;
  }
  outputfile << std::endl;

  // right
  outputfile << scan_right_num << " " << std::flush;
  angle = 180;
  for(int i=0; i<scan_left.ranges.size(); i++) {
    if(flag_right[i] == true) {
      double x = pose_right.Rmat[0][0]*scan_right.ranges[i]*std::cos(DEG2RAD(angle)) + pose_right.Rmat[0][1]*scan_right.ranges[i]*std::sin(DEG2RAD(angle)) + pose_right.tx;
      double y = pose_right.Rmat[1][0]*scan_right.ranges[i]*std::cos(DEG2RAD(angle)) + pose_right.Rmat[1][1]*scan_right.ranges[i]*std::sin(DEG2RAD(angle)) + pose_right.ty;
      outputfile << x << " " << y << " " << std::flush;
    }
    angle -= 0.2803738493;
  }
  outputfile << std::endl;

  ROS_INFO("Output sensor data to file. timestamp = %d", stamp);
  stamp++;

  return;
}

////////////////////////////////////////////////////////////////////////////////

void Listener::loop(int loop_rate_hz) {
  ros::Rate loop_rate(loop_rate_hz);
  getTransform();
  writeFormat();

  ros::AsyncSpinner spinner(5);
  spinner.start();
  ROS_INFO("sleep()");
  loop_rate.sleep();

  while(ros::ok()){
    {
      std::lock_guard<std::mutex> lock(m);
      write2file();
    }
    ROS_INFO("sleep()");
    loop_rate.sleep();
  }

  spinner.stop();
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
