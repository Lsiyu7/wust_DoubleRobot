//Mrp2a DH参数
#ifndef DH_H
#define DH_H

#include <iostream> 
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h> 
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <time.h>

static double a1=0,a2=0,a3=0,a4=0,a5=0,a6=0,a7=0;
static double alpha1=0, alpha2=-M_PI/2, alpha3=M_PI/2, alpha4=-M_PI/2, alpha5=M_PI/2,alpha6=-M_PI/2, alpha7=M_PI/2;
static double d1=0.164, d2=0.0, d3=0.292, d4=0, d5=0.242, d6=0, d7=0.224;

Eigen::Matrix<double,4,4> TransMatrixDH(double a,double alpha,double d,double theta);

int DisplayPoint(double x,double y,double z);

Eigen::Matrix<double,4,4> ForwardKinematics(double theta1,double theta2,double
theta3,double theta4,double theta5,double theta6,double theta7);

int moveJ(double j1,double j2,double j3,double j4,double j5,double j6,double j7);

#endif
