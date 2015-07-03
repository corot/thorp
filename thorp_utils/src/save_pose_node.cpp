#include <fstream>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class SaveAmclPose
{
  ros::Subscriber pose_subscriber_;
  geometry_msgs::PoseWithCovarianceStamped last_pose_;
  bool has_last_pose_;
  std::ofstream file_to_write_;
  double check_frequency_;
  ros::NodeHandle nh_;
  boost::mutex the_mutex_;
  boost::thread timer_;

public:
  SaveAmclPose(ros::NodeHandle nh, std::string file_name) :
      has_last_pose_(false), file_to_write_(file_name.c_str()), check_frequency_(10.0), nh_(nh)
  {
    pose_subscriber_ = nh_.subscribe("/amcl_pose", 100, &SaveAmclPose::AMCLPoseCB, this);
    nh_.param<double>("check_frequency", check_frequency_, 10.0);
    timer_ = boost::thread(boost::bind(&SaveAmclPose::TimerCB, this));
  }

  void AMCLPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    boost::mutex::scoped_lock the_lock(the_mutex_);
    has_last_pose_ = true;
    last_pose_ = *msg;
  }

  void TimerCB()
  {
    ros::Rate rate(check_frequency_);

    while (!ros::isShuttingDown())
      rate.sleep();

    boost::mutex::scoped_lock the_lock(the_mutex_);
    if (!has_last_pose_)
      std::cerr << "can't save position to file no position given" << std::endl;

    file_to_write_ << last_pose_.header.frame_id << " ";
    file_to_write_ << last_pose_.pose.pose.position.x << " ";
    file_to_write_ << last_pose_.pose.pose.position.y << " ";
    file_to_write_ << last_pose_.pose.pose.position.z << " ";
    file_to_write_ << last_pose_.pose.pose.orientation.x << " ";
    file_to_write_ << last_pose_.pose.pose.orientation.y << " ";
    file_to_write_ << last_pose_.pose.pose.orientation.z << " ";
    file_to_write_ << last_pose_.pose.pose.orientation.w << std::endl;

    file_to_write_.flush();
    file_to_write_.close();
  }
};

bool initAmcl(ros::NodeHandle nh, std::string file_name, geometry_msgs::PoseWithCovarianceStamped& last_pose)
{
  // read last amcl pose from file, if any
  std::ifstream file_stream(file_name.c_str());
  file_stream >> last_pose.header.frame_id;
  if ((last_pose.header.frame_id == "0") || (last_pose.header.frame_id == ""))
  {
    ROS_INFO("No valid last amcl pose found; amcl will keep the default initial pose");
    return false;
  }
  file_stream >> last_pose.pose.pose.position.x;
  file_stream >> last_pose.pose.pose.position.y;
  file_stream >> last_pose.pose.pose.position.z;
  file_stream >> last_pose.pose.pose.orientation.x;
  file_stream >> last_pose.pose.pose.orientation.y;
  file_stream >> last_pose.pose.pose.orientation.z;
  file_stream >> last_pose.pose.pose.orientation.w;

  if ((last_pose.pose.pose.orientation.x == 0.) && (last_pose.pose.pose.orientation.y == 0.) &&
      (last_pose.pose.pose.orientation.z == 0.) && (last_pose.pose.pose.orientation.w == 0.))
    last_pose.pose.pose.orientation.w = 1.0;

  // set some arbitrary covariance values (TODO: save it too!)
  last_pose.pose.covariance.elems[0] = 0.25;
  last_pose.pose.covariance.elems[7] = 0.25;
  last_pose.pose.covariance.elems[35] = 0.069;

  file_stream.close();

  ROS_INFO_STREAM("Amcl reinitialized with the last saved pose:\n" << last_pose);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_amcl_pose");
  ros::NodeHandle nh;

  std::string file_name;
  nh.param<std::string>("last_pose_file", file_name, "last_pose.txt");

  ros::Publisher init_amcl = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

  geometry_msgs::PoseWithCovarianceStamped msg;
  if (initAmcl(nh, file_name, msg))
    init_amcl.publish(msg);

  SaveAmclPose save_amcl(nh, file_name);

  ros::spin();
}
