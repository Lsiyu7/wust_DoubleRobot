#include "wust_robot_driver/wust_robot_driver.h"

namespace ros_control_mr {
  mrDriver::mrDriver(ros::NodeHandle nh) :
          nh_(nh),max_velocity_(3.14),robot_mode_(4),
          as_(nh_, "left_arm_controller/follow_joint_trajectory",
          boost::bind(&mrDriver::goalCB, this, _1),
          boost::bind(&mrDriver::cancelCB, this, _1), false), joint_offsets_(
          16, 0.0)
  {
    std::string busname;
    int buf_int;
    char device_key[256];
    ROS_DEBUG("The action server for this driver has been started");

    if (ros::param::get("/left_arm_hardware_interface/peak_can_device/num", buf_int)) {
      ROS_INFO("Number of Peak CAN devices: %d", buf_int);
    } else {
      ROS_ERROR("Failed to get param '/left_arm_hardware_interface/peak_can_device/num'");
    }
    num_devices_ = buf_int;
    // start masters(Peak CAN devices)
    for (std::size_t i = 0; i < num_devices_; ++i) {
      std::sprintf(device_key, "/left_arm_hardware_interface/peak_can_device/device%d", (int)i);
      ros::param::get(device_key, busname);
      if (busname.c_str() == NULL) {
        ROS_ERROR("Peak CAN device %d name not specified", (int)i);
        exit(-2);
      }
      if (MR_ERROR_OK == startMaster(busname.c_str(), MASTER(i))) {
        ROS_INFO("device%d: %s has been initiallized.", (int)i, busname.c_str());
      } else {
        exit(-2);
      }
    }

    ros::param::get("/left_arm_hardware_interface/joints", joint_names_);
    if (joint_names_.size() == 0) {
      ROS_FATAL_STREAM_NAMED("mr_driver",
          "No joints found on parameter server for controller, did you load the proper yaml file?" << " Namespace: " << nh_.getNamespace());
      exit(-1);
    }
    nh_.getParam("/left_arm_hardware_interface/jointID", jointID_);
    num_joints_ = joint_names_.size();
    if (num_joints_ != jointID_.size()) {
      ROS_ERROR("joint names doesn't match joint IDs");
      exit(-2);
    }

    hJoint_.resize(num_joints_);

    // grippers
    ros::param::get("/left_arm_hardware_interface/grippers", gripper_names_);
    if (gripper_names_.size() == 0) {
      ROS_FATAL_STREAM_NAMED("mr_driver",
          "No grippers found on parameter server for controller, did you load the proper yaml file?" << " Namespace: " << nh_.getNamespace());
      exit(-1);
    }
    nh_.getParam("/left_arm_hardware_interface/gripperID", gripperID_);
    num_grippers_ = gripper_names_.size();
    if (num_grippers_ != gripperID_.size()) {
      ROS_ERROR("gripper names doesn't match joint IDs");
      exit(-2);
    }

    hGripper_.resize(num_grippers_);

    double loop_hz = 0.010;
    if (ros::param::get("/left_arm_hardware_control_loop/loop_hz", loop_hz)) {
        ROS_DEBUG("loop_hz set to: %f [hz]", loop_hz);
    }
    servoj_time_ = 1./loop_hz;

    // Initialize joints
    for (std::size_t i = 0; i < num_joints_; ++i) {
      int masterId;
      std::vector<std::string> map;
      ros::param::get("/left_arm_hardware_interface/jointDevice", map);
      std::sscanf(map.at(i).c_str(), "device%d", &masterId);
      hJoint_[i] = jointUp(jointID_[i], MASTER(masterId));
      ROS_INFO_STREAM_NAMED("mr_driver",
          "Loading " << joint_names_[i] << " id "<< jointID_[i] << " on device" << masterId << ((hJoint_[i]!=NULL)?" SUCCEED" : " FAILED"));
      if(hJoint_[i]) {
        jointSetMode(hJoint_[i], joint_cyclesync, 1000, NULL);
      }
    }

    // Initialize grippers
    for (std::size_t i = 0; i < num_grippers_; ++i) {
      int masterId;
      std::vector<std::string> map;
      ros::param::get("/left_arm_hardware_interface/gripperDevice", map);
      std::sscanf(map.at(i).c_str(), "device%d", &masterId);
      hGripper_[i] = gripperUp(gripperID_[i], MASTER(masterId));
      ROS_INFO_STREAM_NAMED("mr_driver",
          "Loading " << gripper_names_[i] << " id "<< gripperID_[i] << " on device" << masterId << ((hJoint_[i]!=NULL)?" SUCCEED" : " FAILED"));
      if(hGripper_[i]) {
        if(gripperSetMode(hGripper_[i], gripper_servo, 1000, NULL)) {
        }
      }
    }

    //Base and tool frames
//    base_frame_ = joint_prefix + "base_Link";
//    tool_frame_ = joint_prefix + "tool0_controller";
//    if (ros::param::get("~base_frame", base_frame_)) {
//        base_frame_ = base_frame_;
//        ROS_DEBUG("Base frame set to: %s", base_frame_.c_str());
//    }

    //start actionserver
    has_goal_ = false;
    as_.start();

    //subscribe to the data topic of interest
    rt_publish_thread_ = new std::thread(boost::bind(&mrDriver::publishRTMsg, this));

    position_sub_ = nh_.subscribe("left_arm/joint_cmd", 1, &mrDriver::jointInterface, this);
    gripper_sub_ = nh_.subscribe("left_gripper/gripper_cmd", 1, &mrDriver::gripperInterface, this);
  }

  mrDriver::~mrDriver()
  {
    for (std::size_t i = 0; i < num_devices_; ++i) {
      stopMaster(MASTER(i));
      joinMaster(MASTER(i));
    }
  }

  void mrDriver::halt() {
//    robot_.halt();
    rt_publish_thread_->join();

  }

  bool mrDriver::openServo() {
    return true;
  }

  void mrDriver::closeServo(std::vector<double> positions) {
    std::vector<double> velocities;
    velocities.resize(positions.size());
    servoj(positions, velocities);
  }

  void mrDriver::interp_cubic(double t, double T,
      std::vector<double> p0_pos, std::vector<double> p1_pos,
      std::vector<double> p0_vel, std::vector<double> p1_vel,
      std::vector<double> & positions, std::vector<double> & velocities)
  {
    /*Returns positions of the joints at time 't' */
    for (unsigned int i = 0; i < p0_pos.size(); i++) {
      double a = p0_pos[i];
      double b = p0_vel[i];
      double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i]
          - T * p1_vel[i]) / pow(T, 2);
      double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i]
          + T * p1_vel[i]) / pow(T, 3);
      positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
      velocities.push_back(b + 2 * c * t + 3 * d * pow(t, 2));
    }
  }

  void mrDriver::servoj(std::vector<double> positions, std::vector<double> velocities) {
    for(std::size_t i = 0; i < positions.size(); i++) {
      jointPush(hJoint_[i], positions[i]/M_PI*180., velocities[i]/M_PI*180.);
    }
  }

  bool mrDriver::doTraj(std::vector<double> inp_timestamps,
      std::vector<std::vector<double> > inp_positions,
      std::vector<std::vector<double> > inp_velocities)
  {
    std::chrono::high_resolution_clock::time_point t0, t;
    std::vector<double> positions, velocities;
    unsigned int j;

    if (!mrDriver::openServo()) {
        return false;
    }

    executing_traj_ = true;
    t0 = std::chrono::high_resolution_clock::now();
    t = t0;
    j = 0;
    while ((inp_timestamps[inp_timestamps.size() - 1]
        >= std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count())
        && executing_traj_) {
      while (inp_timestamps[j]
          <= std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count()
          && j < inp_timestamps.size() - 1) {
        j += 1;
      }
      positions.resize(0);
      velocities.resize(0);
      mrDriver::interp_cubic(
          std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count() - inp_timestamps[j - 1],
          inp_timestamps[j] - inp_timestamps[j - 1], inp_positions[j - 1],
          inp_positions[j], inp_velocities[j - 1], inp_velocities[j],
          positions, velocities);

      mrDriver::servoj(positions, velocities);

      // oversample with 1 * sample_time
      std::this_thread::sleep_for(std::chrono::milliseconds((int) ((servoj_time_ * 1000) / 1.)));
      t = std::chrono::high_resolution_clock::now();
    }
    executing_traj_ = false;

    //Signal robot to stop driverProg()
    mrDriver::closeServo(positions);
    return true;
  }

  void mrDriver::stopTraj() {
    executing_traj_ = false;
  }

  void mrDriver::trajThread(std::vector<double> timestamps,
      std::vector<std::vector<double> > positions,
      std::vector<std::vector<double> > velocities) {
    ROS_INFO("do trajectory");
    doTraj(timestamps, positions, velocities);
    if (has_goal_) {
      result_.error_code = result_.SUCCESSFUL;
      goal_handle_.setSucceeded(result_);
      has_goal_ = false;
    }
  }
  void mrDriver::goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) {
    ROS_INFO("on_goal");

    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
        *gh.getGoal(); //make a copy that we can modify
    if (has_goal_) {
      ROS_WARN("Received new goal while still executing previous trajectory. Canceling previous trajectory");
      has_goal_ = false;
      stopTraj();
      result_.error_code = -100; //nothing is defined for this...?
      result_.error_string = "Received another trajectory";
      goal_handle_.setAborted(result_, result_.error_string);
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    goal_handle_ = gh;
    if (!validateJointNames()) {
      std::string outp_joint_names = "";
      for (unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
        outp_joint_names += goal.trajectory.joint_names[i] + " ";
      }
      result_.error_code = result_.INVALID_JOINTS;
      result_.error_string =
          "Received a goal with incorrect joint names: "
              + outp_joint_names;
      gh.setRejected(result_, result_.error_string);
      ROS_ERROR("%s", result_.error_string.c_str());
      return;
    }
    if (!has_positions()) {
      result_.error_code = result_.INVALID_GOAL;
      result_.error_string = "Received a goal without positions";
      gh.setRejected(result_, result_.error_string);
      ROS_ERROR("%s", result_.error_string.c_str());
      return;
    }

    if (!has_velocities()) {
      result_.error_code = result_.INVALID_GOAL;
      result_.error_string = "Received a goal without velocities";
      gh.setRejected(result_, result_.error_string);
      ROS_ERROR("%s", result_.error_string.c_str());
      return;
    }

    if (!traj_is_finite()) {
      result_.error_string = "Received a goal with infinities or NaNs";
      result_.error_code = result_.INVALID_GOAL;
      gh.setRejected(result_, result_.error_string);
      ROS_ERROR("%s", result_.error_string.c_str());
      return;
    }

    if (!has_limited_velocities()) {
      result_.error_code = result_.INVALID_GOAL;
      result_.error_string =
          "Received a goal with velocities that are higher than "
              + std::to_string(max_velocity_);
      gh.setRejected(result_, result_.error_string);
      ROS_ERROR("%s", result_.error_string.c_str());
      return;
    }

    reorder_traj_joints(goal.trajectory);

    if (!start_positions_match(goal.trajectory, 0.01)) {
      result_.error_code = result_.INVALID_GOAL;
      result_.error_string = "Goal start doesn't match current pose";
      gh.setRejected(result_, result_.error_string);
      ROS_ERROR("%s", result_.error_string.c_str());
      return;
    }

    std::vector<double> timestamps;
    std::vector<std::vector<double> > positions, velocities;
    if (goal.trajectory.points[0].time_from_start.toSec() != 0.) {
      ROS_WARN(
          "Trajectory's first point should be the current position, with time_from_start set to 0.0 - Inserting point in malformed trajectory");
      timestamps.push_back(0.0);
      std::vector<double> pos, vel;
      pos.resize(num_joints_);
      vel.resize(num_joints_);
      for (int i; i++; i < num_joints_) {
        float f1, f2;
        jointPoll(hJoint_[i], &f1, &f2, NULL);
        pos[i] = f1*M_PI/180.;
        vel[i] = f2*M_PI/180.;
      }
      positions.push_back(pos);
      velocities.push_back(vel);
    }
    for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      timestamps.push_back(goal.trajectory.points[i].time_from_start.toSec());
      positions.push_back(goal.trajectory.points[i].positions);
      velocities.push_back(goal.trajectory.points[i].velocities);
    }

    goal_handle_.setAccepted();
    has_goal_ = true;
    std::thread(&mrDriver::trajThread, this, timestamps, positions,
        velocities).detach();
  }

  void mrDriver::cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> gh) {
    // set the action state to preempted
    ROS_INFO("on_cancel");
    if (has_goal_) {
      if (gh == goal_handle_) {
        stopTraj();
        has_goal_ = false;
      }
    }
    result_.error_code = -100; //nothing is defined for this...?
    result_.error_string = "Goal cancelled by client";
    gh.setCanceled(result_);
  }

  bool mrDriver::validateJointNames() {
    std::vector<std::string> actual_joint_names = joint_names_;
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
        *goal_handle_.getGoal();
    if (goal.trajectory.joint_names.size() != actual_joint_names.size())
      return false;

    for (unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
      unsigned int j;
      for (j = 0; j < actual_joint_names.size(); j++) {
        if (goal.trajectory.joint_names[i] == actual_joint_names[j])
          break;
      }
      if (goal.trajectory.joint_names[i] == actual_joint_names[j]) {
        actual_joint_names.erase(actual_joint_names.begin() + j);
      } else {
        return false;
      }
    }

    return true;
  }

  void mrDriver::reorder_traj_joints(trajectory_msgs::JointTrajectory& traj) {
    /* Reorders trajectory - destructive */
    std::vector<std::string> actual_joint_names = joint_names_;
    std::vector<unsigned int> mapping;
    mapping.resize(actual_joint_names.size(), actual_joint_names.size());
    for (unsigned int i = 0; i < traj.joint_names.size(); i++) {
      for (unsigned int j = 0; j < actual_joint_names.size(); j++) {
        if (traj.joint_names[i] == actual_joint_names[j])
          mapping[j] = i;
      }
    }
    traj.joint_names = actual_joint_names;
    std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
    for (unsigned int i = 0; i < traj.points.size(); i++) {
      trajectory_msgs::JointTrajectoryPoint new_point;
      for (unsigned int j = 0; j < traj.points[i].positions.size(); j++) {
        new_point.positions.push_back(
            traj.points[i].positions[mapping[j]]);
        new_point.velocities.push_back(
            traj.points[i].velocities[mapping[j]]);
        if (traj.points[i].accelerations.size() != 0)
          new_point.accelerations.push_back(
              traj.points[i].accelerations[mapping[j]]);
      }
      new_point.time_from_start = traj.points[i].time_from_start;
      new_traj.push_back(new_point);
    }
    traj.points = new_traj;
  }

  bool mrDriver::has_velocities() {
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
        *goal_handle_.getGoal();
    for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      if (goal.trajectory.points[i].positions.size()
          != goal.trajectory.points[i].velocities.size())
        return false;
    }
    return true;
  }

  bool mrDriver::has_positions() {
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
        *goal_handle_.getGoal();
    if (goal.trajectory.points.size() == 0)
      return false;
    for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      if (goal.trajectory.points[i].positions.size()
          != goal.trajectory.joint_names.size())
        return false;
    }
    return true;
  }

  bool mrDriver::start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps)
  {
    float pos, vel, curr;
    for (unsigned int i = 0; i < traj.points[0].positions.size(); i++)
    {
      jointPoll(hJoint_[i], &pos, &vel, &curr);

      if( fabs(traj.points[0].positions[i] - pos*M_PI/180.) > eps )
      {
        return false;
      }
    }
    return true;
  }

  bool mrDriver::has_limited_velocities() {
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
        *goal_handle_.getGoal();
    for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      for (unsigned int j = 0;
          j < goal.trajectory.points[i].velocities.size(); j++) {
        if (fabs(goal.trajectory.points[i].velocities[j])
            > max_velocity_)
          return false;
      }
    }
    return true;
  }

  bool mrDriver::traj_is_finite() {
    actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
        *goal_handle_.getGoal();
    for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
      for (unsigned int j = 0;
          j < goal.trajectory.points[i].velocities.size(); j++) {
        if (!std::isfinite(goal.trajectory.points[i].positions[j]))
          return false;
        if (!std::isfinite(goal.trajectory.points[i].velocities[j]))
          return false;
      }
    }
    return true;
  }

  void mrDriver::jointInterface(const mr_msgs::JointCommand::ConstPtr& msg) {
    if (joint_names_ != msg->names) {
      ROS_WARN_NAMED("jointInterface", "joint names not match");
      return;
    }
    if (robot_mode_ != msg->command) {
      int i = 0;
      for(; i < num_joints_; i++) {
        int32_t result = jointSetMode(hJoint_[i], (jointMode_t)msg->command, 1000, NULL);
        if (result) break;
      }
      if (i != num_joints_) ROS_INFO("Fail to set mode of all joints, only first %d joints set.", i);
      else robot_mode_ = msg->command;
    }
    else {
      switch (msg->command) {
      case 1: // current
        for(int i = 0; i < num_joints_; i++) {
          // jointSetCurrent(hJoint_[i], msg->cmdcurr[i], -1, NULL);
        }
        break;
      case 2: // speed
        for(int i = 0; i < num_joints_; i++) {
          jointSetSpeed(hJoint_[i], msg->cmdspd[i], -1, NULL);
        }
        break;
      case 3: // position
        for(int i = 0; i < num_joints_; i++) {
          jointSetPosition(hJoint_[i], msg->cmdPos[i], -1, NULL);
        }
        break;
      case 4:

        break;
      default:
        break;
      }
    }
  }

//rostopic pub -r 10 /mra7a/gripper_cmd mr_msgs/Grippeommand 3 1 Gripper1 0 0
  // received messages for one gripper
  void mrDriver::gripperInterface(const mr_msgs::GripperCommand::ConstPtr& msg) {
    for(int i = 0; i < num_grippers_; i++){
      if (msg->name != gripper_names_[i]) continue;
      gripperPush(hGripper_[i], msg->positionL, msg->positionR);

//      gripperSetMode(hGripper_[i], (gripperMode_t)msg->mode, 1000, NULL);
//      switch(msg->mode) {
//      case gripper_none: break;
//      case gripper_openclose: gripperSetOpenState(hGripper_[i], msg->command, 1000, NULL); break;
//      case gripper_position: {
//        gripperSetPosition(hGripper_[i], msg->positionL, msg->positionR, 1000, NULL);
//      }
//      break;
//      case gripper_servo: gripperPush(hGripper_[i], msg->positionL, msg->positionR); break;
//      default: break;
//      }
    }
  }

  void mrDriver::publishRTMsg() {
    float pos, vel, curr;
    ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("/left_arm/joint_states", 1);
//    ros::Publisher gripper_pub = nh_.advertise<mr_msgs::GripperState>("/left_gripper_states", 1);
    ros::Publisher gripper_joint_pub = nh_.advertise<sensor_msgs::JointState>("/left_gripper/joint_states", 1);
//    ros::Publisher wrench_pub = nh_.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
    //ros::Publisher tool_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("tool_velocity", 1);
//    static tf::TransformBroadcaster br;
    while (ros::ok()) {
      sensor_msgs::JointState joint_msg;
      mr_msgs::GripperState gripper_msg;
      joint_msg.name = joint_names_;

      joint_msg.header.stamp = ros::Time::now();
      joint_msg.position.resize(num_joints_);
      joint_msg.velocity.resize(num_joints_);
      joint_msg.effort.resize(num_joints_);
      for (int i = 0; i < num_joints_; i++) {
        jointGetSpeed(hJoint_[i], NULL, -1, NULL);
        jointPoll(hJoint_[i], &pos, &vel, &curr);
        joint_msg.position[i] = pos*M_PI/180.;
        joint_msg.position[i] += joint_offsets_[i];
        joint_msg.velocity[i] = vel*M_PI/180.;
        joint_msg.effort[i] = curr;
      }
      joint_pub.publish(joint_msg);

//      gripper_msg.header.stamp = ros::Time::now();
//      gripper_msg.names = gripper_names_;
//      gripper_msg.positionsL.resize(num_grippers_);
//      gripper_msg.positionsR.resize(num_grippers_);
//      gripper_msg.torqueL.resize(num_grippers_);
//      gripper_msg.torqueR.resize(num_grippers_);
//
//      for (int i = 0; i < num_grippers_; i++) {
//        float posl, posr, torql, torqr;
//        gripperPoll(hGripper_[i], &posl, &posr, &torql, &torqr);
//        gripper_msg.positionsL[i] = posl;
//        gripper_msg.positionsR[i] = posr;
//        gripper_msg.torqueL[i] = torql;
//        gripper_msg.torqueR[i] = torqr;
//      }
//      gripper_pub.publish(gripper_msg);

      sensor_msgs::JointState gripper_joint_msg;
      gripper_joint_msg.name = {"L_left_joint", "L_right_joint"};
      gripper_joint_msg.header.stamp = ros::Time::now();
      gripper_joint_msg.position.resize(2);
      gripper_joint_msg.velocity.resize(2);
      gripper_joint_msg.effort.resize(2);
      for (int i = 0; i < 1; i++) {
        float posl, posr, torql, torqr;
        gripperPoll(hGripper_[i], &posl, &posr, &torql, &torqr);
        if (posl>400) posl=400;
        gripper_joint_msg.position[i] = 0.36-((posl/400)*0.72);
        gripper_joint_msg.velocity[i] = 0.0;
        gripper_joint_msg.effort[i] = 0.0;
        gripper_joint_msg.position[i+1] = ((posl/400)*0.72)-0.36;
        gripper_joint_msg.velocity[i+1] = 0.0;
        gripper_joint_msg.effort[i+1] = 0.0;
       }
       gripper_joint_pub.publish(gripper_joint_msg);


      //std::vector<double> tcp_force = robot_.rt_interface_->robot_state_->getTcpForce();
//      wrench_msg.header.stamp = joint_msg.header.stamp;
//      wrench_msg.wrench.force.x = tcp_force[0];
//      wrench_msg.wrench.force.y = tcp_force[1];
//      wrench_msg.wrench.force.z = tcp_force[2];
//      wrench_msg.wrench.torque.x = tcp_force[3];
//      wrench_msg.wrench.torque.y = tcp_force[4];
//      wrench_msg.wrench.torque.z = tcp_force[5];
//      wrench_pub.publish(wrench_msg);

      // Tool vector: Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
//      std::vector<double> tool_orientation = robot_.rt_interface_->robot_state_->getToolOrientation();

      //Create quaternion
//      tf::Quaternion quat;
//      double w = tool_orientation[0];
//      double x = tool_orientation[1];
//      double y = tool_orientation[2];
//      double z = tool_orientation[3];

//      double rx = atan2(2.0*(w*x+y*z),1-2.0*(x*x+y*y));
//      double ry = asin(2.0*(w*y-z*x));
//      double rz = atan2(2.0*(w*x+x*y),1-2.0*(y*y+z*z));

//      double angle = std::sqrt(std::pow(rx,2) + std::pow(ry,2) + std::pow(rz,2));
//      if (angle < 1e-16) {
//        quat.setValue(0, 0, 0, 1);
//      } else {
//        quat.setRotation(tf::Vector3(rx/angle, ry/angle, rz/angle), angle);
//      }

      //Create and broadcast transform
//      std::vector<double> tool_position = robot_.rt_interface_->robot_state_->getToolPosition();
//      tf::Transform transform;
//      transform.setOrigin(tf::Vector3(tool_position[0], tool_position[1], tool_position[2]));
//      transform.setRotation(quat);
//      br.sendTransform(tf::StampedTransform(transform, joint_msg.header.stamp, base_frame_, tool_frame_));
      std::this_thread::sleep_for(std::chrono::milliseconds((int) (1000.*servoj_time_))); // 10ms
    }
  }
} // end of namespace


int main(int argc, char **argv) {
  ros::init(argc, argv, "left_arm_driver");
  ros::NodeHandle nh;

  ros_control_mr::mrDriver interface(nh);

  ros::AsyncSpinner spinner(6);
  spinner.start();

  ros::waitForShutdown();

  interface.halt();

  exit(0);
}
