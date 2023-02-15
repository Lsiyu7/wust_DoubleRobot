#include <dual_arm_coordination.h>

using namespace std;

static double a1=0,a2=0,a3=0,a4=0,a5=0,a6=0,a7=0;
static double alpha1=0, alpha2=-M_PI/2, alpha3=M_PI/2, alpha4=-M_PI/2, alpha5=M_PI/2,alpha6=-M_PI/2, alpha7=M_PI/2;
static double d1=0.164, d2=0.0, d3=0.292, d4=0, d5=0.242, d6=0, d7=0.224;

// 欧拉角转四元数(xyz)
Eigen::Quaterniond dual_arm_coordinate::euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

// 四元数转欧拉角(xyz)
Eigen::Vector3d dual_arm_coordinate::Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2,1,0);
    return euler;
}

// 欧拉角转旋转矩阵(xyz)
Eigen::Matrix3d dual_arm_coordinate::euler2RotationMatrix(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d R = q.matrix();
    std::cout<<R<<std::endl;
    return R;
}

// 旋转矩阵转欧拉角(xyz)
Eigen::Vector3d dual_arm_coordinate::RotationMatrix2euler(Eigen::Matrix3d R)
{
    Eigen::Matrix3d m;
    m = R;
    Eigen::Vector3d euler = m.eulerAngles(2,1,0);
    return euler;
}

// 机械臂末端变换矩阵
Eigen::Matrix<double,4,4> dual_arm_coordinate::TransMatrixDH(double a,double alpha,double d,double theta)
{
    Eigen::Matrix<double,4,4> T;
    T.row(0)<<cos(theta),-sin(theta), 0, a;
    T.row(1)<<sin(theta)*cos(alpha),cos(theta)*cos(alpha),-sin(alpha),-sin(alpha)*d;
    T.row(2)<<sin(theta)*sin(alpha),cos(theta)*sin(alpha),cos(alpha),cos(alpha)*d;
    T.row(3)<<0,0,0,1;

    return T;
}


//void dual_arm_coordinate(const std::string &group_name, Eigen::Vector3d pos, Eigen::Vector3d rot)
//{
//    dual_arm_coordinate t;
//
//    moveit::planning_interface::MoveGroupInterface group(group_name);
//    //获取终端link的名称
//    std::string end_effector_link = group.getEndEffectorLink();
//
//    //设置目标位置所使用的参考坐标系
//    std::string reference_frame = "base_link";
//    group.setPoseReferenceFrame(reference_frame);
//    group.allowReplanning(true);
//    group.setPlannerId("RRTConnectkConfigDefault");
//    group.setGoalPositionTolerance(0.001);
//    group.setGoalOrientationTolerance(0.01);
//    //机械臂末端位姿
//    geometry_msgs::Pose target_pose;
//
//    target_pose.position.x = pos(0);
//    target_pose.position.y = pos(1);
//    target_pose.position.z = pos(2);
//
//    Eigen::Vector4d quaternion;
//    for(int i = 0; i < 4; i++)
//    {
//        quaternion(i) = t.euler2Quaternion(rot(0), rot(1), rot(2)).coeffs()[i];
//    }
//
//    target_pose.orientation.x = quaternion(0);
//    target_pose.orientation.y = quaternion(1);
//    target_pose.orientation.z = quaternion(2);
//    target_pose.orientation.w = quaternion(3);
//
////    std::cout<<pos(0)<<std::endl;
////    std::cout<<pos(1)<<std::endl;
////    std::cout<<pos(2)<<std::endl;
////    std::cout<<quaternion(0)<<std::endl;
////    std::cout<<quaternion(1)<<std::endl;
////    std::cout<<quaternion(2)<<std::endl;
////    std::cout<<quaternion(3)<<std::endl;
//
//    // 设置机器臂当前的状态作为运动初始状态
//    group.setStartStateToCurrentState();
//    group.setPoseTarget(target_pose);
//
//    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
//    moveit::planning_interface::MoveGroupInterface::Plan plan;
//    moveit::planning_interface::MoveItErrorCode success = group.plan(plan);
//
//    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");
//    if(success)
//        group.execute(plan);
//    sleep(1);
//}



//机械臂正运动学方程
Eigen::Matrix<double,4,4> dual_arm_coordinate::ForwardKinematics(double theta[7])
{

    Eigen::Matrix<double,4,4> T1, T2, T3, T4, T5, T6, T7;
    Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
    T1=TransMatrixDH(a1,alpha1,d1,theta[0]);
    T2=TransMatrixDH(a2,alpha2,d2,theta[1]);
    //std::cout<<"======STOP====== "<<T2<<std::endl;
    T3=TransMatrixDH(a3,alpha3,d3,theta[2]);
    T4=TransMatrixDH(a4,alpha4,d4,theta[3]);
    T5=TransMatrixDH(a5,alpha5,d5,theta[4]);
    T6=TransMatrixDH(a6,alpha6,d6,theta[5]);
    T7=TransMatrixDH(a7,alpha7,d7,theta[6]);
    T=T7;
    //std::cout<<"1.T = "<<T<<std::endl;
    T=T6*T;
    //std::cout<<"2.T = "<<T<<std::endl;
    T=T5*T;
    //std::cout<<"3.T = "<<T<<std::endl;
    T=T4*T;
    //std::cout<<"4.T = "<<T<<std::endl;
    T=T3*T;
    //std::cout<<"5.T = "<<T<<std::endl;
    T=T2*T;
    //std::cout<<"6.T = "<<T<<std::endl;
    T=T1*T;
    //std::cout<<"7.T = "<<T<<std::endl;
    return T;
}

//机械臂逆运动学
Eigen::VectorXd dual_arm_coordinate::inverse_kinematics(const std::string &group_name,
                                    Eigen::Vector3d pos, Eigen::Vector3d rot)
{

    dual_arm_coordinate t;

    moveit::planning_interface::MoveGroupInterface group(group_name);
    //获取终端link的名称
    std::string end_effector_link = group.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    group.setPoseReferenceFrame(reference_frame);
    group.allowReplanning(true);
    group.setPlannerId("RRTConnect");
    group.setGoalPositionTolerance(0.001);
    group.setGoalOrientationTolerance(0.01);

    //机械臂末端位姿
    geometry_msgs::Pose target_pose;

    target_pose.position.x = pos(0);
    target_pose.position.y = pos(1);
    target_pose.position.z = pos(2);

    Eigen::Vector4d quaternion;
    for(int i = 0; i < 4; i++)
    {
        quaternion(i) = t.euler2Quaternion(rot(0), rot(1), rot(2)).coeffs()[i];
    }

    target_pose.orientation.x = quaternion(0);
    target_pose.orientation.y = quaternion(1);
    target_pose.orientation.z = quaternion(2);
    target_pose.orientation.w = quaternion(3);

    //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    //robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    //robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));


//    std::cout<<pos(0)<<std::endl;
//    std::cout<<pos(1)<<std::endl;
//    std::cout<<pos(2)<<std::endl;
//    std::cout<<quaternion(0)<<std::endl;
//    std::cout<<quaternion(1)<<std::endl;
//    std::cout<<quaternion(2)<<std::endl;
//    std::cout<<quaternion(3)<<std::endl;

    // 设置机器臂当前的状态作为运动初始状态
    group.setStartStateToCurrentState();
    group.setPoseTarget(target_pose);

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");

    int n_points = plan.trajectory_.joint_trajectory.points.size();
    //std::cout<<plan.trajectory_.joint_trajectory.joint_names[0]<<std::endl;
    //std::cout<<plan.trajectory_.joint_trajectory.points[0].positions[0]<<std::endl;
    //return plan.trajectory_.joint_trajectory.points[n_points-1];
    Eigen::VectorXd joint(7);
//    cout<<plan.trajectory_.joint_trajectory.points[n_points-1].positions[0]<<endl;
//    cout<<plan.trajectory_.joint_trajectory.points[n_points-1].positions[1]<<endl;
//    cout<<plan.trajectory_.joint_trajectory.points[n_points-1].positions[2]<<endl;
//    cout<<plan.trajectory_.joint_trajectory.points[n_points-1].positions[3]<<endl;
//    cout<<plan.trajectory_.joint_trajectory.points[n_points-1].positions[4]<<endl;
//    cout<<plan.trajectory_.joint_trajectory.points[n_points-1].positions[5]<<endl;
//    cout<<plan.trajectory_.joint_trajectory.points[n_points-1].positions[6]<<endl;

    for (int i = 0; i < 7;i++)
    {
        joint(i) = plan.trajectory_.joint_trajectory.points[n_points-1].positions[i];
    }
    return joint;
}

// 轴动
void dual_arm_coordinate::arm_joint_control(const std::string &group_name, double theta[7])
{
    moveit::planning_interface::MoveGroupInterface group(group_name);
    std::string reference_frame = "base_link";
    std::vector<double> joint_group_positions(7);
    joint_group_positions[0] = theta[0];
    joint_group_positions[1] = theta[1];
    joint_group_positions[2] = theta[2];
    joint_group_positions[3] = theta[3];
    joint_group_positions[4] = theta[4];
    joint_group_positions[5] = theta[5];
    joint_group_positions[6] = theta[6];

    group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");
    if (!group.execute(plan))
    {
        ROS_ERROR("Failed to execute plan");
    }

}

//双臂规划
void dual_arm_coordinate::dual_arm_pose_control(Eigen::Vector3d pos_left, Eigen::Vector3d rot_left,
                                                Eigen::Vector3d pos_right, Eigen::Vector3d rot_right)
{
    dual_arm_coordinate t;

    moveit::planning_interface::MoveGroupInterface group("dual_arm");
    //获取终端link的名称
    std::string end_effector_link = group.getEndEffectorLink();

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    group.setPoseReferenceFrame(reference_frame);
    group.allowReplanning(true);
    group.setPlannerId("RRTConnect");

    //机械臂末端位姿
    geometry_msgs::PoseStamped target_pose1;

    target_pose1.header.frame_id = "base_link";
    target_pose1.pose.position.x = pos_left(0);
    target_pose1.pose.position.y = pos_left(1);
    target_pose1.pose.position.z = pos_left(2);

    Eigen::Vector4d quaternion_left;
    for(int i = 0; i < 4; i++)
    {
        quaternion_left(i) = t.euler2Quaternion(rot_left(0), rot_left(1), rot_left(2)).coeffs()[i];
    }

    target_pose1.pose.orientation.x = quaternion_left(0);
    target_pose1.pose.orientation.y = quaternion_left(1);
    target_pose1.pose.orientation.z = quaternion_left(2);
    target_pose1.pose.orientation.w = quaternion_left(3);

    geometry_msgs::PoseStamped target_pose2;

    target_pose2.header.frame_id = "base_link";
    target_pose2.pose.position.x = pos_right(0);
    target_pose2.pose.position.y = pos_right(1);
    target_pose2.pose.position.z = pos_right(2);

    Eigen::Vector4d quaternion_right;
    for(int i = 0; i < 4; i++)
    {
        quaternion_right(i) = t.euler2Quaternion(rot_right(0), rot_right(1), rot_right(2)).coeffs()[i];
    }

    target_pose2.pose.orientation.x = quaternion_right(0);
    target_pose2.pose.orientation.y = quaternion_right(1);
    target_pose2.pose.orientation.z = quaternion_right(2);
    target_pose2.pose.orientation.w = quaternion_right(3);

    // 设置机器臂当前的状态作为运动初始状态
    group.setStartStateToCurrentState();
    group.setPoseTarget(target_pose1, "L_ee");
    group.setPoseTarget(target_pose2, "R_ee");

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");
    if (!group.execute(plan))
    {
        ROS_ERROR("Failed to execute plan");
    }
    cout<<plan.trajectory_.joint_trajectory<<endl;
}

//双臂协调
bool dual_arm_coordinate::master_slaver(Eigen::Vector3d pos_left, Eigen::Vector3d rot_left,
                                        Eigen::Vector3d pos_r_from_l, Eigen::Vector3d rot_r_from_l)
{
    dual_arm_coordinate t;
	// 左臂基坐标系相对于base_link的变换矩阵
    Eigen::Matrix<double,4,4> T_L0_from_base;
    T_L0_from_base.row(0)<<0, -0.865, 0.5017, -0.278;
    T_L0_from_base.row(1)<<0, 0.5017, 0.865, 0.076;
    T_L0_from_base.row(2)<<-1, 0, 0, 1.155;
    T_L0_from_base.row(3)<<0, 0, 0, 1;

    // 右臂基坐标系相对于base_link的变换矩阵
    Eigen::Matrix<double,4,4> T_R0_from_base;
    T_R0_from_base.row(0)<<0, -0.866, 0.4988, -0.278;
    T_R0_from_base.row(1)<<0, -0.4988, -0.8668, -0.076;
    T_R0_from_base.row(2)<<1, 0, 0, 1.155;
    T_R0_from_base.row(3)<<0, 0, 0, 1;

 	//从臂相对于主臂的变换矩阵
    Eigen::Matrix<double,4,4> T_R_from_L;
    Eigen::Matrix3d R = t.euler2RotationMatrix(rot_r_from_l(0), rot_r_from_l(1), rot_r_from_l(2));
    T_R_from_L.row(0)<<R(0,0), R(0,1), R(0,2), pos_r_from_l(0);
    T_R_from_L.row(1)<<R(1,0), R(1,1), R(1,2), pos_r_from_l(1);
    T_R_from_L.row(2)<<R(2,0), R(2,1), R(2,2), pos_r_from_l(2);
    T_R_from_L.row(3)<<0, 0, 0, 1;

	moveit::planning_interface::MoveGroupInterface group("dual_arm");
	moveit::planning_interface::MoveGroupInterface group_left("left_arm");
	moveit::planning_interface::MoveGroupInterface group_right("right_arm");
    std::string reference_frame = "base_link";
    group_left.setPoseReferenceFrame(reference_frame);
    group_left.allowReplanning(true);
    group_left.setPlannerId("RRTConnectkConfigDefault");

    group_left.setGoalPositionTolerance(0.01);
    group_left.setGoalOrientationTolerance(0.05);

	//规划左臂的轨迹
    geometry_msgs::Pose target_pose;

    target_pose.position.x = pos_left(0);
    target_pose.position.y = pos_left(1);
    target_pose.position.z = pos_left(2);

    Eigen::Vector4d quaternion;
    for(int i = 0; i < 4; i++)
    {
        quaternion(i) = t.euler2Quaternion(rot_left(0), rot_left(1), rot_left(2)).coeffs()[i];
    }

    target_pose.orientation.x = quaternion(0);
    target_pose.orientation.y = quaternion(1);
    target_pose.orientation.z = quaternion(2);
    target_pose.orientation.w = quaternion(3);

    group_left.setStartStateToCurrentState();
    group_left.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan_left;
    moveit::planning_interface::MoveItErrorCode success = group_left.plan(plan_left);


	int n_joints = plan_left.trajectory_.joint_trajectory.joint_names.size();
	int n_points = plan_left.trajectory_.joint_trajectory.points.size();

    string joint_names_right[7] = {"R_joint1", "R_joint2", "R_joint3", "R_joint4", "R_joint5", "R_joint6", "R_joint7"};
    //重新定义一条轨迹
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = plan_left.trajectory_.joint_trajectory.joint_names;
    trajectory.joint_trajectory.points = plan_left.trajectory_.joint_trajectory.points;
    //cout<<trajectory.joint_trajectory<<endl;
    //cout<<trajectory.joint_trajectory.points[0]<<endl;
    for(int i = 0; i < 7;i++)
    {
        trajectory.joint_trajectory.joint_names.push_back(joint_names_right[i]);
    }

	//左臂路点数据
	double joint_left[7];    //每个轨迹点的关节角数据
    vector<double*> joints_left;  //存储所有的轨迹点

    //左臂末端轨迹路点列表(相对左臂基坐标系)
    Eigen::Matrix<double,4,4> T_Left_from_L0;
    //左臂末端轨迹路点列表
    Eigen::Matrix<double,4,4> T_Left;
    //右臂末端轨迹路点列表
    Eigen::Matrix<double,4,4> T_Right;

    //右臂路点数据
    double joint_right[7];
    vector<double*> joints_right;

    //计算出右臂轨迹的关节角数据
    for(int n = 0; n < n_points; n++)
    {
        for (int i = 0; i < 7;i++)
        {
            joint_left[i] = plan_left.trajectory_.joint_trajectory.points[n].positions[i];
        }
        joints_left.push_back(joint_left);     //joints_left向量里有n个元素,每个元素是1个含7个元素的一维数组

        T_Left_from_L0 = t.ForwardKinematics(joint_left);
        T_Left = T_L0_from_base*T_Left_from_L0;
        T_Right = T_Left*T_R_from_L;

        // 右臂末端旋转矩阵
        Eigen::Matrix<double,3,3> R_Right;
        R_Right.row(0)<<T_Right(0,0), T_Right(0,1), T_Right(0,2);
        R_Right.row(1)<<T_Right(1,0), T_Right(1,1), T_Right(1,2);
        R_Right.row(2)<<T_Right(2,0), T_Right(2,1), T_Right(2,2);

        Eigen::Vector3d pos_right;
        pos_right<<T_Right(0,3), T_Right(1,3), T_Right(2,3);
        Eigen::Vector3d rot_right= t.RotationMatrix2euler(R_Right);

//        cout<<"T_Right"<<T_Right<<endl;
//        cout<<"R_Right"<<R_Right<<endl;
//        cout<<"pos_right"<<pos_right<<endl;
//        cout<<"rot_right"<<rot_right<<endl;

        Eigen::VectorXd q(7);
        q = t.inverse_kinematics("right_arm", pos_right, rot_right);
        // 第一个点为右臂初始位置
        if(n==0)
        {
            for(int j = 0; j<7; j++) {
                trajectory.joint_trajectory.points[0].positions.push_back(group_right.getCurrentJointValues()[j]);
            }
        }
        else
        {
            for(int m = 0;m < 7; m++)
            {
                trajectory.joint_trajectory.points[n].positions.push_back(q(m));
            }
        }
        for(int k = 0; k < 7; k++)
        {
            joint_right[k] = q(k);
        }
        joints_right.push_back(joint_right);
    }

    //重新规划速度和加速度
    double accScale = 0.5;
    double velScale = 0.5;
    moveit::planning_interface::MoveGroupInterface::Plan dual_arm_plan;

    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "dual_arm");
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, velScale, accScale);

    rt.getRobotTrajectoryMsg(trajectory);
    dual_arm_plan.trajectory_ = trajectory;

//    for(int i=0;i<7;i++) {
//        cout << dual_arm_plan.trajectory_.joint_trajectory.points[0].velocities[i+7] << endl;
//    }
//    for(int i=0;i<7;i++) {
//        cout << dual_arm_plan.trajectory_.joint_trajectory.points[1].velocities[i+7] << endl;
//    }
//    cout<<"joint"<<endl;
//    for(int i=0;i<7;i++) {
//        cout << dual_arm_plan.trajectory_.joint_trajectory.points[0].positions[i+7] << endl;
//    }
    cout<<"traj"<<endl;;
    cout<<dual_arm_plan.trajectory_.joint_trajectory<<endl;
    if (!group.execute(dual_arm_plan))
    {
        ROS_ERROR("Failed to execute plan");
        return false;
    }
	return true;
}

//臂形角方法
void dual_arm_coordinate::arm_angle_inverse_kinematics(const std::string &group_name, Eigen::Vector3d pos,
                                                    Eigen::Vector3d rot, double q)
{
    Eigen::Vector3d bt;    //机械臂末端到基坐标系的向量
    Eigen::Vector3d bs;    //机械臂s坐标系到基坐标系的向量
    if(group_name == "left_arm")
    {
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_coordination");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    dual_arm_coordinate t;
    
    //t.euler2RotationMatrix(0.919, 0.472, 2.237);

    //双臂协调
    Eigen::Vector3d pos_left(0.30, 0.18, 1.12);
    Eigen::Vector3d rot_left(-2.713, 1.503, 2.244);
    Eigen::Vector3d pos(-0.013, -0.059, 0.232);
    Eigen::Vector3d rot(2.620, -0.031, 3.140);
    t.master_slaver(pos_left, rot_left, pos, rot);
   // double theta[7] = {0.0489146,0.702708,1.42906,-0.718713,0.617084,-1.56289,-1.13594};
//    Eigen::Vector3d pos_left(0.85, 0.26, 1.08);
//    Eigen::Vector3d rot_left(-2.713, 1.503, 2.244);
//    Eigen::Vector3d pos_right(0.85, -0.26, 1.08);
//    Eigen::Vector3d rot_right(-0.837, -1.473, -1.011);
//    t.dual_arm_pose_control(pos_left, rot_left, pos_right, rot_right);

    //movej
    //double theta[7] = {0,0,0,0.5,0,0,0};
//    double theta[7] = {1.0,1.0,0,0,0,0,0};
//    t.arm_joint_control("left_arm", theta);
    return 0;
}


