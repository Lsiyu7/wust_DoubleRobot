#include "wust_robot_driver/wust_robot_moveit_control_full.h"


void arm_joint_control(moveit::planning_interface::MoveGroupInterface::group_name,joint)
{
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_control_full");
    ros::NodeHandle node_handle;
    //ros::NodeHandle nh;

    //

    ros::AsyncSpinner spinner(1);
    spinner.start();

    return 0;//
}
