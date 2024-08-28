#include <cstdio>
#include <iostream>

#include <vector>

#include <ctime>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>

using namespace std;

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// KDL and URDF headers
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

// For end-effector pose
#include <kdl/frames.hpp> 

class AlexRobot : public rclcpp::Node{

  public:

  AlexRobot() : Node("alex_robot"){

    start_time = std::chrono::steady_clock::now();

    // Publisher for joint_states
    alex_pub = this->create_publisher<sensor_msgs::msg::JointState>( "joint_command", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&AlexRobot::jointCallback, this));

    // Subscriber for joint_states
    alex_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&AlexRobot::jointStateCallback, this, std::placeholders::_1));

    joint_message = sensor_msgs::msg::JointState();
    joint_message.name = {"LeftShoulderPitch", "RightShoulderPitch", "LeftShoulderRoll", "RightShoulderRoll", "LeftShoulderYaw", "RightShoulderYaw","LeftElbowPitch",
                          "RightElbowPitch", "LeftWristYaw", "RightWristYaw", "LeftWristRoll", "RightWristRoll", "LeftGripperYaw", "RightGripperYaw",
                          "Left_index_q1", "Left_middle_q1", "Left_pinky_q1", "Left_ring_q1", "Left_thumb_q1",
                          "Right_index_q1", "Right_middle_q1", "Right_pinky_q1", "Right_ring_q1", "Right_thumb_q1",
                          "Left_index_q2", "Left_middle_q2", "Left_pinky_q2", "Left_ring_q2", "Left_thumb_q2",
                          "Right_index_q2", "Right_middle_q2", "Right_pinky_q2", "Right_ring_q2", "Right_thumb_q2"
    };

    // Initialize URDF and KDL chain
    initializeKinematics();

    // left_arm  =  joint_message.position[0, 2, 4, 6, 8, 10, 12]
    left_arm_des_pos_vec  = {-30 * M_PI/180, 75 * M_PI/180, 0.0, -60 * M_PI/180, 45 * M_PI/180, 0.0, 0.0};
    left_arm_des_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_arm_cur_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_arm_cur_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    left_arm_des_trq_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // right_arm =  joint_message.position[1, 3, 5, 7, 9, 11, 13]
    right_arm_des_pos_vec  = {-30 * M_PI/180, -75 * M_PI/180, 0.0, -60 * M_PI/180, -45 * M_PI/180, 0.0, 0.0};   
    right_arm_des_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    right_arm_cur_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   
    right_arm_cur_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    right_arm_des_trq_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // left_hand  = joint_message.position[14, 15, 16, 17, 18, 24, 25, 26, 27, 28]
    left_hand_des_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_hand_des_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_hand_cur_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_hand_cur_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    left_hand_des_trq_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // right_hand = joint_message.position[19, 20, 21, 22, 23, 29, 30, 31, 32, 33]
    right_hand_des_pos_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    right_hand_des_vel_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    right_hand_cur_pos_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    right_hand_cur_vel_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

    right_hand_des_trq_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // left_arm - end-effector - desired and current
    left_arm_des_end_eff_pose_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_arm_cur_end_eff_pose_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // right_arm - end-effector - desired and current
    right_arm_des_end_eff_pose_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    right_arm_cur_end_eff_pose_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    left_arm_jacobian_matrix = Eigen::MatrixXd::Zero(6, 7);
    right_arm_jacobian_matrix = Eigen::MatrixXd::Zero(6, 7);

    // joint-message - Position
    joint_message.position = {0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 
                              0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180,
                              0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180,
                              0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180};

    // joint-message - Velocity
    joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // joint-message - Effort
    joint_message.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // controller PD gains
    kp_arm = 25;
    kd_arm = 2.5;

    kp_hand = 0.125;
    kd_hand = 0.0125;

    frequency = 0.1;
    amplitude = 1.0;
  }

  // URDF parsing and kinematics setup
  void initializeKinematics() {

      urdf::Model robot_model;
      if (!robot_model.initFile("/home/keyhan/git/alex_robot_models/alex_description/urdf/20240516_Alex_TestStand_FixedHead_PsyonicHands.urdf")) {
          RCLCPP_ERROR(this->get_logger(), "Failed to load URDF file");
          return;
      }

      KDL::Tree robot_tree;
      if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to convert URDF to KDL tree");
          return;
      }

      if (!robot_tree.getChain("base_link", "LeftGripperYawLink", left_arm_chain)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get left arm chain");
      }

      if (!robot_tree.getChain("base_link", "RightGripperYawLink", right_arm_chain)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to get right arm chain");
      }

      left_arm_jacobian_solver = std::make_shared<KDL::ChainJntToJacSolver>(left_arm_chain);
      right_arm_jacobian_solver = std::make_shared<KDL::ChainJntToJacSolver>(right_arm_chain);

      // Initialize forward kinematics solvers
      left_arm_fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(left_arm_chain);
      right_arm_fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(right_arm_chain);
      RCLCPP_INFO(this->get_logger(), "Kinematics initialized successfully");
  }

  // Subscriber callback
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){

    // RCLCPP_INFO(this->get_logger(), "Received joint state update");
    
    for (int i = 0; i < int(left_arm_cur_pos_vec.size()); i++){

      left_arm_cur_pos_vec[i] = msg->position[2 * i];
      left_arm_cur_vel_vec[i] = msg->position[2 * i];

      right_arm_cur_pos_vec[i] = msg->position[2 * i + 1];
      right_arm_cur_vel_vec[i] = msg->position[2 * i + 1];
    }

    for (int i = 0; i < int(left_hand_cur_pos_vec.size()/2); i++){
      left_hand_cur_pos_vec[i]     = msg->position[i + 14];
      left_hand_cur_pos_vec[i + 5] = msg->position[i + 24];
      left_hand_cur_vel_vec[i]     = msg->position[i + 14];
      left_hand_cur_vel_vec[i + 5] = msg->position[i + 24];

      right_hand_cur_pos_vec[i]     = msg->position[i + 19];
      right_hand_cur_pos_vec[i + 5] = msg->position[i + 29];
      right_hand_cur_vel_vec[i]     = msg->position[i + 19];
      right_hand_cur_vel_vec[i + 5] = msg->position[i + 29];
    }

    // print out the first joint name and position
    // if (!msg->name.empty() && !msg->position.empty()) {
    //   RCLCPP_INFO(this->get_logger(), "First joint: %s, Position: %f", 
    //               msg->name[0].c_str(), msg->position[0]);
    // }

  }

  // Publisher callback
  void jointCallback(){

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time;
    time = elapsed.count(); // time in seconds

    for (int i = 0; i < int(left_arm_des_pos_vec.size()); i++){

      joint_message.position[2 * i] = left_arm_des_pos_vec[i];
      joint_message.velocity[2 * i] = left_arm_des_vel_vec[i];

      joint_message.effort[2 * i] = left_arm_des_trq_vec[i];

      joint_message.position[2 * i + 1] = right_arm_des_pos_vec[i];
      joint_message.velocity[2 * i + 1] = right_arm_des_vel_vec[i];

      joint_message.effort[2 * i + 1] = right_arm_des_trq_vec[i];
    }

    for (int i = 0; i < int(left_hand_des_pos_vec.size()/2); i++){
      joint_message.position[i + 14] =  left_hand_des_pos_vec[i];
      joint_message.position[i + 24] =  left_hand_des_pos_vec[i + 5];

      joint_message.velocity[i + 14] =  left_hand_des_vel_vec[i];
      joint_message.velocity[i + 24] =  left_hand_des_vel_vec[i + 5];

      joint_message.effort[i + 14] =  left_hand_des_trq_vec[i];
      joint_message.effort[i + 24] =  left_hand_des_trq_vec[i + 5];

      joint_message.position[i + 19] =  right_hand_des_pos_vec[i];
      joint_message.position[i + 29] =  right_hand_des_pos_vec[i + 5];

      joint_message.velocity[i + 19] =  right_hand_des_vel_vec[i];
      joint_message.velocity[i + 29] =  right_hand_des_vel_vec[i + 5];

      joint_message.effort[i + 19] =  right_hand_des_trq_vec[i];
      joint_message.effort[i + 29] =  right_hand_des_trq_vec[i + 5];
    }

    // RCLCPP_INFO(this->get_logger(), "Callback function running");
    
    target_position = sin(amplitude * M_PI * frequency * time);
    target_velocity = amplitude * M_PI * cos(2 * M_PI * frequency * time);

    // left_arm_des_pos_vec[0] = target_position;
    // right_arm_des_pos_vec[0] = target_position;

    // left_arm_des_vel_vec[0] = target_velocity;
    // right_arm_des_vel_vec[0] = target_velocity;

    for (int i = 0; i < int(left_arm_des_trq_vec.size()); i++){

      left_arm_des_trq_vec[i] = kp_arm * (left_arm_des_pos_vec[i] - left_arm_cur_pos_vec[i]) + kd_arm * (left_arm_des_vel_vec[i] - left_arm_cur_vel_vec[i]);
      right_arm_des_trq_vec[i] = kp_arm * (right_arm_des_pos_vec[i] - right_arm_cur_pos_vec[i]) + kd_arm * (right_arm_des_vel_vec[i] - right_arm_cur_vel_vec[i]);
      
    }    

    for (int i = 0; i < int(left_hand_des_trq_vec.size()); i++){
      
      left_hand_des_trq_vec[i] = kp_hand * (left_hand_des_pos_vec[i] - left_hand_cur_pos_vec[i]) + kd_hand * (left_hand_des_vel_vec[i] - left_hand_cur_vel_vec[i]);
      right_hand_des_trq_vec[i] = kp_hand * (right_hand_des_pos_vec[i] - right_hand_cur_pos_vec[i]) + kd_hand * (right_hand_des_vel_vec[i] - right_hand_cur_vel_vec[i]);
      
    }    

    // for(double &pos: joint_message.position){
    //   pos = target_position;
    // }

    // for(double &pos: joint_message.velocity){
    //   pos = target_velocity;
    // }

    // Compute Jacobian for left and right arms
    computeJacobian();

    // Compute forward kinematics for left and right arms
    computeForwardKinematics();

    // Publishing joint messages
    alex_pub->publish(joint_message);

    std::cout << std::endl << "Time: " << time << std::endl << std::endl;

    // std::cout << "left_arm_des_trq_vec[3] =  " << left_arm_des_trq_vec[1] << std::endl;
    // std::cout << left_arm_des_pos_vec[1] << " --- " << left_arm_cur_pos_vec[1] << " --- " << left_arm_des_pos_vec[1] - left_arm_cur_pos_vec[1] << std::endl;

    // std::cout << "left_hand_des_trq_vec[0] =  " << left_hand_des_trq_vec[0] << std::endl;
    // std::cout << left_hand_des_pos_vec[0] << " --- " << left_hand_cur_pos_vec[0] << " --- " << left_hand_des_pos_vec[0] - left_hand_cur_pos_vec[0] << std::endl;

  }

    // Function to compute Jacobian matrices for both arms
  void computeJacobian() {

    KDL::JntArray left_arm_joint_positions(left_arm_chain.getNrOfJoints());
    KDL::JntArray right_arm_joint_positions(right_arm_chain.getNrOfJoints());

    for (int i = 0; i < int(left_arm_joint_positions.rows()); ++i) {
        left_arm_joint_positions(i) = left_arm_cur_pos_vec[i];
        right_arm_joint_positions(i) = right_arm_cur_pos_vec[i];
    }

    KDL::Jacobian left_arm_jacobian(left_arm_chain.getNrOfJoints());
    KDL::Jacobian right_arm_jacobian(right_arm_chain.getNrOfJoints());

    left_arm_jacobian_solver->JntToJac(left_arm_joint_positions, left_arm_jacobian);
    right_arm_jacobian_solver->JntToJac(right_arm_joint_positions, right_arm_jacobian);

    for (int i = 0; i < int(left_arm_des_pos_vec.size() - 1); i++){
      for (int j = 0; j < int(left_arm_des_pos_vec.size()); j++){
        
        left_arm_jacobian_matrix(i , j) = left_arm_jacobian.data(i, j);
        right_arm_jacobian_matrix(i , j) = right_arm_jacobian.data(i, j);
      }
    }

    std::cout << "Left Arm Jacobian: \n" << left_arm_jacobian_matrix << std::endl;
    std::cout << "\nRight Arm Jacobian: \n" << right_arm_jacobian_matrix << std::endl;

    }

    // Function to compute forward kinematics for both arms
  void computeForwardKinematics() {

    KDL::JntArray left_arm_joint_positions(left_arm_chain.getNrOfJoints());
    KDL::JntArray right_arm_joint_positions(right_arm_chain.getNrOfJoints());

    // Fill in the joint positions for left and right arms
    for (int i = 0; i < int(left_arm_joint_positions.rows()); ++i) {

        left_arm_joint_positions(i) = left_arm_cur_pos_vec[i];
        right_arm_joint_positions(i) = right_arm_cur_pos_vec[i];
    }
    // Define frames for the end-effector positions
    KDL::Frame left_arm_ee_frame;
    KDL::Frame right_arm_ee_frame;

    // Compute forward kinematics for left and right arms
    if (left_arm_fk_solver->JntToCart(left_arm_joint_positions, left_arm_ee_frame) >= 0) {

        // std::cout << std::endl << "Left Arm End-Effector Position: "
        //           << "X: " << left_arm_ee_frame.p.x() << ", "
        //           << "Y: " << left_arm_ee_frame.p.y() << ", "
        //           << "Z: " << left_arm_ee_frame.p.z() << std::endl;

        // Extract orientation as Euler angles (roll, pitch, yaw)
        left_arm_ee_frame.M.GetRPY(left_arm_roll, left_arm_pitch, left_arm_yaw);
        // std::cout << "Left Arm End-Effector Orientation (RPY): "
        //           << "Roll: " << left_arm_roll << ", "
        //           << "Pitch: " << left_arm_pitch << ", "
        //           << "Yaw: " << left_arm_yaw << std::endl;
    } else {

        RCLCPP_ERROR(this->get_logger(), "Failed to compute forward kinematics for left arm.");
    }
    if (right_arm_fk_solver->JntToCart(right_arm_joint_positions, right_arm_ee_frame) >= 0) {

        // std::cout << std::endl << "Right Arm End-Effector Position: "
        //           << "X: " << right_arm_ee_frame.p.x() << ", "
        //           << "Y: " << right_arm_ee_frame.p.y() << ", "
        //           << "Z: " << right_arm_ee_frame.p.z() << std::endl;

        // Extract orientation as Euler angles (roll, pitch, yaw)
        right_arm_ee_frame.M.GetRPY(right_arm_roll, right_arm_pitch, right_arm_yaw);
        // std::cout << "Right Arm End-Effector Orientation (RPY): "
        //           << "Roll: " << right_arm_roll << ", "
        //           << "Pitch: " << right_arm_pitch << ", "
        //           << "Yaw: " << right_arm_yaw << std::endl;
    } else {

        RCLCPP_ERROR(this->get_logger(), "Failed to compute forward kinematics for right arm.");
    }

    left_arm_cur_end_eff_pose_vec[0] = left_arm_ee_frame.p.x();
    left_arm_cur_end_eff_pose_vec[1] = left_arm_ee_frame.p.y();
    left_arm_cur_end_eff_pose_vec[2] = left_arm_ee_frame.p.z();

    left_arm_cur_end_eff_pose_vec[3] =  left_arm_roll;
    left_arm_cur_end_eff_pose_vec[4] =  left_arm_pitch;
    left_arm_cur_end_eff_pose_vec[5] =  left_arm_yaw;

    right_arm_cur_end_eff_pose_vec[0] = right_arm_ee_frame.p.x();
    right_arm_cur_end_eff_pose_vec[1] = right_arm_ee_frame.p.y();
    right_arm_cur_end_eff_pose_vec[2] = right_arm_ee_frame.p.z();

    right_arm_cur_end_eff_pose_vec[3] =  right_arm_roll;
    right_arm_cur_end_eff_pose_vec[4] =  right_arm_pitch;
    right_arm_cur_end_eff_pose_vec[5] =  right_arm_yaw;

    std::cout << std::endl << "Left Arm End-Effector Pose: " << "\tP.x = " << left_arm_cur_end_eff_pose_vec[0] << "\tP.y = " << left_arm_cur_end_eff_pose_vec[1] << "\tP.z = " << left_arm_cur_end_eff_pose_vec[2] 
                                                             << "\tO.x = " << left_arm_cur_end_eff_pose_vec[3] << "\tO.y = " << left_arm_cur_end_eff_pose_vec[4] << "\tO.z = " << left_arm_cur_end_eff_pose_vec[5] << std::endl;

    std::cout << "Right Arm End-Effector Pose: " << "\tP.x = " << right_arm_cur_end_eff_pose_vec[0] << "\tP.y = " << right_arm_cur_end_eff_pose_vec[1] << "\tP.z = " << right_arm_cur_end_eff_pose_vec[2] 
                                                 << "\tO.x = " << right_arm_cur_end_eff_pose_vec[3] << "\tO.y = " << right_arm_cur_end_eff_pose_vec[4] << "\tO.z = " << right_arm_cur_end_eff_pose_vec[5] << std::endl;

    }

  private:

  double kp_arm, kd_arm, kp_hand, kd_hand; 
  double frequency, time, amplitude, target_position, target_velocity;

  double left_arm_roll, left_arm_pitch, left_arm_yaw;
  double right_arm_roll, right_arm_pitch, right_arm_yaw;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr alex_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr alex_sub;

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::time_point<std::chrono::steady_clock> start_time;

  sensor_msgs::msg::JointState joint_message;
  
  std::vector<double> left_arm_des_pos_vec;
  std::vector<double> left_arm_des_vel_vec;
  std::vector<double> left_arm_cur_pos_vec;
  std::vector<double> left_arm_cur_vel_vec;

  std::vector<double> right_arm_des_pos_vec;
  std::vector<double> right_arm_des_vel_vec;
  std::vector<double> right_arm_cur_pos_vec;
  std::vector<double> right_arm_cur_vel_vec;

  std::vector<double> left_hand_des_pos_vec;
  std::vector<double> left_hand_des_vel_vec;
  std::vector<double> left_hand_cur_pos_vec;
  std::vector<double> left_hand_cur_vel_vec;

  std::vector<double> right_hand_des_pos_vec;
  std::vector<double> right_hand_des_vel_vec;
  std::vector<double> right_hand_cur_pos_vec;
  std::vector<double> right_hand_cur_vel_vec;

  std::vector<double> left_arm_des_end_eff_pose_vec;
  std::vector<double> right_arm_des_end_eff_pose_vec;

  std::vector<double> left_arm_cur_end_eff_pose_vec;
  std::vector<double> right_arm_cur_end_eff_pose_vec;

  std::vector<double> left_arm_des_trq_vec;
  std::vector<double> right_arm_des_trq_vec;
  std::vector<double> left_hand_des_trq_vec;
  std::vector<double> right_hand_des_trq_vec;

  // KDL chains for left and right arms
  KDL::Chain left_arm_chain;
  KDL::Chain right_arm_chain;

  // Jacobian solvers
  std::shared_ptr<KDL::ChainJntToJacSolver> left_arm_jacobian_solver;
  std::shared_ptr<KDL::ChainJntToJacSolver> right_arm_jacobian_solver;

  // Forward kinematics solvers
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> left_arm_fk_solver;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> right_arm_fk_solver;

  // Jacobian Matrix
  Eigen::MatrixXd left_arm_jacobian_matrix;
  Eigen::MatrixXd right_arm_jacobian_matrix;

};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlexRobot>());
  rclcpp::shutdown();

  printf("alex_robot_package package executed sucessfully\n");
  return 0;
}
