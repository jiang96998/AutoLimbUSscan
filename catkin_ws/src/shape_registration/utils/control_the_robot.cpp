/**
  STD
 **/

#include <iostream>
#include <string>
#include <vector>


/**
  ROS RELATED
 **/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <iiwa_msgs/JointPosition.h>
#include <sensor_msgs/PointCloud2.h>


/**
  PCL RELATED
 **/
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

/**
  EIGEN RELATED
 **/

#include <Eigen/Core>


/***
 * After the trajectory is calculated and transformed into the robot base coordinate system
 * then in this file the trajectory will be read from the file is is written in, and then
 * a pose will get created and then be published to the topic /iiwa/command/CartesianPose
 * this was you can move the real robot or the simulation.
 * */



#define SIM

const std::string BASE_LINK = "iiwa_link_0";
const std::string EE_LINK = "iiwa_link_ee";


class RobotControl {

  ros::NodeHandle nh_;
  ros::Publisher pub_desired_pose_;
  ros::Publisher pub_arm_;
  ros::Subscriber sub_pose_;
  geometry_msgs::Pose curr_pose_;
  sensor_msgs::PointCloud2 arm_cloud_msg;
  geometry_msgs::Pose prev_pose_;
  std::vector<geometry_msgs::Pose> poses_;
  void init_position(const geometry_msgs::PoseStampedConstPtr &position_msg);
  void read_positions();

public:
  RobotControl(ros::NodeHandle nh);
  ~RobotControl(){}
};

RobotControl::RobotControl(ros::NodeHandle nh): nh_(nh) {

  read_positions();
  sub_pose_ = nh.subscribe("/iiwa/command/CartesianPose", 30, &RobotControl::init_position, this);
  pub_desired_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose",10);
  pub_arm_ = nh_.advertise<sensor_msgs::PointCloud2>("/arm_cloud",10);

  auto artery_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto arm_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto transformed_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/nehil/catkin_ws_registration/src/arm_downsampled_robot_base.pcd", *arm_cloud) == -1) //* load the arm
  {
    PCL_ERROR ("Couldn't read file\n");
  }

  // publish the initial position
  geometry_msgs::PoseStamped command;
  command.header.frame_id = BASE_LINK;

  std::vector<double> init_pose{0.0, -0.6, 0.4, 1, 0, 0, 0};

  command.pose.position.x = init_pose[0];
  command.pose.position.y = init_pose[1];
  command.pose.position.z = init_pose[2];
  command.pose.orientation.x = init_pose[3];
  command.pose.orientation.y = init_pose[4];
  command.pose.orientation.z = init_pose[5];
  command.pose.orientation.w = init_pose[6];
  prev_pose_ = command.pose;


  pcl::toROSMsg(*arm_cloud, arm_cloud_msg);
  arm_cloud_msg.header.frame_id = BASE_LINK;

  std::cout << "prev pose : " << prev_pose_ << std::endl;

  pub_desired_pose_.publish(command);
}

void RobotControl::init_position(const geometry_msgs::PoseStampedConstPtr &position_msg){
  static size_t pose_ind = 0;
  ros::Rate loop_rate(1);
  loop_rate.sleep();
  auto init_pose = position_msg->pose;

  if (std::abs(prev_pose_.position.x-init_pose.position.x)+std::abs(prev_pose_.position.y-init_pose.position.y)+std::abs(prev_pose_.position.z-init_pose.position.z) < 0.0001) {
    if(pose_ind < poses_.size()) {
      pub_arm_.publish(arm_cloud_msg);
      geometry_msgs::PoseStamped command;
      command.header.frame_id = BASE_LINK;
      command.pose = poses_[pose_ind];
      prev_pose_ = command.pose;
      pub_desired_pose_.publish(command);
      pose_ind += 1;
      if(pose_ind == poses_.size()) {
        pose_ind = 0;
      }
    }
  }
}

void RobotControl::read_positions() {
    std::string s;
    std::ifstream myfile ("/home/nehil/catkin_ws_registration/src/artery_in_robot_base.txt");
    if (myfile.is_open())
    {
      while ( getline (myfile,s) )
      {
        pcl::PointXYZ point;

        Eigen::VectorXd pointPose(7);
        int ind = 0;
        // parse the string
        std::string delimiter = " ";
        size_t pos = 0;
        std::string token;
        geometry_msgs::Pose pose;
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            if(ind == 0) {
              pose.position.x = std::stod(token);
            }
            else if(ind == 1) {
              pose.position.y = std::stod(token);
            }
            else if(ind == 2) {
              pose.position.z = std::stod(token);
            }
            else if(ind == 3) {
              pose.orientation.x = std::stod(token);
            }
            else if(ind == 4) {
              pose.orientation.y = std::stod(token);
            }
            else if(ind == 5) {
              pose.orientation.z = std::stod(token);
            }
            s.erase(0, pos + delimiter.length());
            ind += 1;
        }
        pose.orientation.w = std::stod(s);
        poses_.push_back(pose);
      }
      myfile.close();
    }
    std::cout << "read the positions \n";

}

int main(int argc, char** argv){
  ros::init(argc, argv, "pub_controller");
  ros::NodeHandle nh;
  RobotControl rob_cont(nh);
  while(ros::ok()) {
    ros::spinOnce();
  }
}
