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
using namespace std;
//// 欧拉角转四元数(xyz)
//Eigen::Quaterniond dual_arm_coordinate::euler2Quaternion(const double roll, const double pitch, const double yaw)
//{
//    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
//    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
//    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
//    return q;
//}
//
//// 四元数转欧拉角(xyz)
//Eigen::Vector3d dual_arm_coordinate::Quaterniond2Euler(const double x,const double y,const double z,const double w)
//{
//    Eigen::Quaterniond q;
//    q.x() = x;
//    q.y() = y;
//    q.z() = z;
//    q.w() = w;
//    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2,1,0);
//    Eigen::VectorXd a(7);
//    a(0) = 1;
//    a(1) = 2;
//    a(2) = 3;
//    a(3) = 1;
//    a(4) = 2;
//    a(5) = 3;
//    a(6) = 3;
//    cout<<"a = "<<a<<endl;
//    return euler;
//}
//
//// 欧拉角转旋转矩阵(xyz)
//Eigen::Matrix3d dual_arm_coordinate::euler2RotationMatrix(const double roll, const double pitch, const double yaw)
//{
//    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
//    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
//    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
//    Eigen::Matrix3d R = q.matrix();
//    return R;
//}
//
//// 旋转矩阵转欧拉角(xyz)
//Eigen::Vector3d dual_arm_coordinate::RotationMatrix2euler(Eigen::Matrix3d R)
//{
//    Eigen::Matrix3d m;
//    m = R;
//    Eigen::Vector3d euler = m.eulerAngles(2,1,0);
//    return euler;
//}
//
int main(int argc, char **argv)
{
//    dual_arm_coordinate arm;
//    cout<<arm.euler2Quaternion(-2.406, 1.566, -1.359).coeffs()[0]<<endl;
//    cout<<arm.Quaterniond2Euler(-0.355, 0.613, 0.352, 0.612)<<endl;
//    //cout<<arm.euler2RotationMatrix(-2.406, 1.566, -1.359)<<endl;
//    Eigen::Matrix3d R;
//    R(0,0) = 0.00100826;
//    R(0,1) = -0.865925;
//    R(0,2) = 0.500173;
//    R(1,0) = -0.00468913;
//    R(1,1) = 0.500164;
//    R(1,2) = 0.865918;
//    R(2,0) = -0.999988;
//    R(2,1) = -0.00321845;
//    R(2,2) = -0.00355614;
//    cout<<R<<endl;
//    cout<<arm.RotationMatrix2euler(R)<<endl;
//
}

//struct Test
//{
//    double positions[];
//};
//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "moveit_revise_trajectory_demo");
//    ros::NodeHandle node_handle;
//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//
////    Test arm;
////    arm.positions=;
////    cout<<arm.positions<<endl;
//    moveit_msgs::RobotTrajectory trajectory;
//    string joint_names_right[7] = {"R_Joint1", "R_Joint2", "R_Joint3", "R_Joint4", "R_Joint5", "R_Joint6", "R_Joint7"};
//    for(int i = 0; i < 7;i++)
//    {
//        trajectory.joint_trajectory.joint_names.push_back(joint_names_right[i]);
//    }
//    trajectory.joint_trajectory.points.push_back(joint_names_right[0]);
//    cout<<trajectory.joint_trajectory<<endl;
//}