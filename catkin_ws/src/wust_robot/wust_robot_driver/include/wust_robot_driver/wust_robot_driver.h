#ifndef MR_DRIVER_H
#define MR_DRIVER_H
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/action_server.h"
#include "actionlib/server/server_goal_handle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "mr_msgs/GripperCommand.h"
#include "mr_msgs/GripperState.h"
#include "mr_msgs/JointCommand.h"
#include <controller_manager/controller_manager.h>

/// TF
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// API
#include <mrapi.h>

namespace ros_control_mr {
class mrDriver {
protected:
  std::condition_variable rt_msg_cond_;
  std::condition_variable msg_cond_;
  ros::NodeHandle nh_;
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_;
  bool has_goal_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;
  ros::Subscriber position_sub_;
  ros::Subscriber gripper_sub_;
  std::thread* rt_publish_thread_;
  std::vector<double> joint_offsets_;
  std::string base_frame_;
  std::string tool_frame_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> gripper_names_;
  std::size_t num_joints_;
  std::size_t num_grippers_;
  double max_velocity_;
  std::vector<JOINT_HANDLE> hJoint_;
  std::vector<GRIPPER_HANDLE> hGripper_;
  std::vector<int> jointID_;
  std::vector<int> gripperID_;
  std::size_t num_devices_;
  double servoj_time_;
  bool executing_traj_;
  int32_t robot_mode_;

public:
  mrDriver(ros::NodeHandle nh);
  ~mrDriver();
  void halt();
private:
  void trajThread(std::vector<double> timestamps,
      std::vector<std::vector<double> > positions,
      std::vector<std::vector<double> > velocities);
  void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
  void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh);
  bool validateJointNames();
  void reorder_traj_joints(trajectory_msgs::JointTrajectory& traj);
  bool has_velocities();
  bool has_positions();
  bool start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps);
  bool has_limited_velocities();
  bool traj_is_finite();
  void jointInterface(const mr_msgs::JointCommand::ConstPtr& msg);
  void gripperInterface(const mr_msgs::GripperCommand::ConstPtr& msg);
  void publishRTMsg();
  bool openServo();
  void closeServo(std::vector<double> positions);
  void interp_cubic(double t, double T,
  std::vector<double> p0_pos, std::vector<double> p1_pos,
  std::vector<double> p0_vel, std::vector<double> p1_vel,
  std::vector<double> & positions, std::vector<double> & velocities);
  void servoj(std::vector<double> positions, std::vector<double> velocities);
  bool doTraj(std::vector<double> inp_timestamps,
  std::vector<std::vector<double> > inp_positions,
  std::vector<std::vector<double> > inp_velocities);
  void stopTraj();
};
}

#endif // MR_DRIVER_H
