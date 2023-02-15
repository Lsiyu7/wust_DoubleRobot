#ifndef DH_H
#define DH_H

#include <iostream> 
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <time.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/transforms/transforms.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

class dual_arm_coordinate{
public:
//    dual_arm_coordinate();
//    ~dual_arm_coordinate();
//
    Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);
    Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w);
    Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw);
    Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R);

    Eigen::Matrix<double,4,4> TransMatrixDH(double a,double alpha,double d,double theta);
    Eigen::Matrix<double,4,4> ForwardKinematics(double theta[7]);
    Eigen::VectorXd inverse_kinematics(const std::string &group_name, Eigen::Vector3d pos, Eigen::Vector3d rot);
    void arm_joint_control(const std::string &group_name, double theta[7]);
    void dual_arm_pose_control(Eigen::Vector3d pos_left, Eigen::Vector3d rot_left,
                                Eigen::Vector3d pos_right, Eigen::Vector3d rot_right);
    bool master_slaver(Eigen::Vector3d pos_left, Eigen::Vector3d rot_left,
                        Eigen::Vector3d pos_r_from_l, Eigen::Vector3d rot_r_from_l);
    void arm_angle_inverse_kinematics(const std::string &group_name, Eigen::Vector3d pos,
                                                           Eigen::Vector3d rot, double q);
};

#endif
