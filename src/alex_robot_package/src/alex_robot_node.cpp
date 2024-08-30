#include <cstdio>
#include <iostream>
#include <iomanip>
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
    left_arm_des_pos_vec  = Eigen::VectorXd(7);
    left_arm_des_vel_vec  = Eigen::VectorXd(7);
    left_arm_cur_pos_vec  = Eigen::VectorXd(7);
    left_arm_cur_vel_vec  = Eigen::VectorXd(7);
    left_arm_des_trq_vec  = Eigen::VectorXd(7);

    left_arm_des_pos_vec << -30 * M_PI/180, 75 * M_PI/180, 0.0, -60 * M_PI/180, 45 * M_PI/180, 0.0, 0.0;

    // right_arm =  joint_message.position[1, 3, 5, 7, 9, 11, 13]
    right_arm_des_pos_vec = Eigen::VectorXd(7);
    right_arm_des_vel_vec = Eigen::VectorXd(7);
    right_arm_cur_pos_vec = Eigen::VectorXd(7);
    right_arm_cur_vel_vec = Eigen::VectorXd(7);
    right_arm_des_trq_vec = Eigen::VectorXd(7);

    right_arm_des_pos_vec << -30 * M_PI/180, -75 * M_PI/180, 0.0, -60 * M_PI/180, -45 * M_PI/180, 0.0, 0.0;

    // left_hand  = joint_message.position[14, 15, 16, 17, 18, 24, 25, 26, 27, 28]
    left_hand_des_pos_vec = Eigen::VectorXd(10);
    left_hand_des_vel_vec = Eigen::VectorXd(10);
    left_hand_cur_pos_vec = Eigen::VectorXd(10);
    left_hand_cur_vel_vec = Eigen::VectorXd(10);
    left_hand_des_trq_vec = Eigen::VectorXd(10);

    // right_hand = joint_message.position[19, 20, 21, 22, 23, 29, 30, 31, 32, 33]
    right_hand_des_pos_vec = Eigen::VectorXd(10);
    right_hand_des_vel_vec = Eigen::VectorXd(10);
    right_hand_cur_pos_vec = Eigen::VectorXd(10);
    right_hand_cur_vel_vec = Eigen::VectorXd(10);
    right_hand_des_trq_vec = Eigen::VectorXd(10);

    // left_arm - end-effector - position - desired and current
    left_arm_des_end_eff_pose_vec = Eigen::VectorXd(6); // P.x = 0.30 P.y = 0.50  P.z = 0.30  O.x = -2.5 * M_PI/180 O.y = -65 * M_PI/180  O.z = 15 * M_PI/180
    left_arm_cur_end_eff_pose_vec = Eigen::VectorXd(6);
    left_arm_end_eff_pose_error = Eigen::VectorXd(6);

    // right_arm - end-effector - position - desired and current
    right_arm_des_end_eff_pose_vec = Eigen::VectorXd(6); // P.x = 0.30  P.y = -0.50 P.z = 0.30  O.x = 2.5 * M_PI/180  O.y = -65 * M_PI/180  O.z = -15 * M_PI/180
    right_arm_cur_end_eff_pose_vec = Eigen::VectorXd(6);
    right_arm_end_eff_pose_error = Eigen::VectorXd(6);

    // left_arm - end-effector - velocity - desired and current
    left_arm_des_end_eff_vel_vec = Eigen::VectorXd(6);
    left_arm_cur_end_eff_vel_vec = Eigen::VectorXd(6);
    left_arm_end_eff_vel_error = Eigen::VectorXd(6);

    // right_arm - end-effector - velocity - desired and current
    right_arm_des_end_eff_vel_vec = Eigen::VectorXd(6);
    right_arm_cur_end_eff_vel_vec = Eigen::VectorXd(6);
    right_arm_end_eff_vel_error = Eigen::VectorXd(6);

    // left_arm - variable values initialisation
    left_arm_des_vel_vec.setZero();
    left_arm_cur_pos_vec.setZero();
    left_arm_cur_vel_vec.setZero();

    left_arm_des_trq_vec.setZero();

    left_arm_des_end_eff_pose_vec.setZero();
    left_arm_cur_end_eff_pose_vec.setZero();
    left_arm_des_end_eff_vel_vec.setZero();
    left_arm_cur_end_eff_vel_vec.setZero();

    left_hand_des_pos_vec.setZero();
    left_hand_des_vel_vec.setZero();
    left_hand_cur_pos_vec.setZero();
    left_hand_cur_vel_vec.setZero();

    left_hand_des_trq_vec.setZero();

    // right_arm - variable values initialisation
    right_arm_des_vel_vec.setZero();
    right_arm_cur_pos_vec.setZero();
    right_arm_cur_vel_vec.setZero();

    right_arm_des_trq_vec.setZero();

    right_arm_des_end_eff_pose_vec.setZero();
    right_arm_cur_end_eff_pose_vec.setZero();

    right_arm_des_end_eff_vel_vec.setZero();
    right_arm_cur_end_eff_vel_vec.setZero();

    right_hand_des_pos_vec.setZero();
    right_hand_des_vel_vec.setZero();
    right_hand_cur_pos_vec.setZero();
    right_hand_cur_vel_vec.setZero();

    right_hand_des_trq_vec.setZero();

    // jacobian matrices initialisation
    left_arm_jacobian_matrix = Eigen::MatrixXd::Zero(6, 7);
    right_arm_jacobian_matrix = Eigen::MatrixXd::Zero(6, 7);

    // stiffness and damping matrices initialisation
    Kp_matrix = Eigen::MatrixXd::Identity(6, 6);
    Kd_matrix = Eigen::MatrixXd::Identity(6, 6);

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
    kp_arm_joint_space = 25;
    kd_arm_joint_space = 1;

    kp_hand = 0.125;
    kd_hand = 0.00125;

    frequency = 0.05;
    amplitude = 0.05;

    // robot control mode (1 or 2)
    // 1 -> joint-space control mode
    // 2 -> task-space control mode
    control_mode = 1;

    // stiffness and damping - end-effector position
    Kp_Matrix_pos = 250; 
    Kd_Matrix_pos = 25;

    // stiffness and damping - end-effector orientation (1, 0.001)
    Kp_Matrix_ori = 1;
    Kd_Matrix_ori = 0.001;

    // robot jacobian calculation method (1 or 2)
    // 1 -> jacobian calculation: KDL
    // 2 -> jacobian calculation: Numerical Calculation
    jacobian_calculation_method = 1;

    // trajectory tracking activation mode
    trajectory_tracking_activation = 0;

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

    for (int i = 0; i < int(left_arm_cur_pos_vec.size()); i++){

      left_arm_cur_pos_vec[i] = msg->position[2 * i];
      left_arm_cur_vel_vec[i] = msg->velocity[2 * i];

      right_arm_cur_pos_vec[i] = msg->position[2 * i + 1];
      right_arm_cur_vel_vec[i] = msg->velocity[2 * i + 1];
    }

    for (int i = 0; i < int(left_hand_cur_pos_vec.size()/2); i++){
      left_hand_cur_pos_vec[i]     = msg->position[i + 14];
      left_hand_cur_pos_vec[i + 5] = msg->position[i + 24];
      left_hand_cur_vel_vec[i]     = msg->velocity[i + 14];
      left_hand_cur_vel_vec[i + 5] = msg->velocity[i + 24];

      right_hand_cur_pos_vec[i]     = msg->position[i + 19];
      right_hand_cur_pos_vec[i + 5] = msg->position[i + 29];
      right_hand_cur_vel_vec[i]     = msg->velocity[i + 19];
      right_hand_cur_vel_vec[i + 5] = msg->velocity[i + 29];
    }

    left_arm_cur_end_eff_vel_vec = left_arm_jacobian_matrix * left_arm_cur_vel_vec;
    right_arm_cur_end_eff_vel_vec = right_arm_jacobian_matrix * right_arm_cur_vel_vec;
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

    if (trajectory_tracking_activation == 1){
      if (time >= 30){
        target_position_sin = amplitude * sin(M_PI * frequency * time);
        target_velocity_cos = M_PI * cos(2 * M_PI * frequency * time);

        target_position_cos = amplitude * cos(M_PI * frequency * time);
        target_velocity_sin = -M_PI * sin(2 * M_PI * frequency * time);

        // left arm end-effector desired pose
        left_arm_des_end_eff_pose_vec << 0.25, 0.35, 0.30 + target_position_sin, -0.0 * M_PI/180, -90 * M_PI/180, 0.0 * M_PI/180;

        // right arm end-effector desired pose
        right_arm_des_end_eff_pose_vec << 0.25, -0.35, 0.30 + target_position_cos, 0.0 * M_PI/180, -90 * M_PI/180, -0.0 * M_PI/180;

        left_arm_des_end_eff_vel_vec(2) =  target_velocity_sin;
        right_arm_des_end_eff_vel_vec(2) =  target_velocity_cos;

      }
    }

    // left arm end-effector desired pose
    left_arm_des_end_eff_pose_vec << 0.25, 0.35, 0.30, -0.0 * M_PI/180, -90 * M_PI/180, 0.0 * M_PI/180;

    // right arm end-effector desired pose
    right_arm_des_end_eff_pose_vec << 0.25, -0.35, 0.30, 0.0 * M_PI/180, -90 * M_PI/180, -0.0 * M_PI/180;

    // left arm end-effector - pose and velocity error
    left_arm_end_eff_pose_error = left_arm_des_end_eff_pose_vec - left_arm_cur_end_eff_pose_vec;
    left_arm_end_eff_vel_error = left_arm_des_end_eff_vel_vec - left_arm_cur_end_eff_vel_vec;

    // right arm end-effector - pose and velocity error
    right_arm_end_eff_pose_error = right_arm_des_end_eff_pose_vec - right_arm_cur_end_eff_pose_vec;
    right_arm_end_eff_vel_error = right_arm_des_end_eff_vel_vec - right_arm_cur_end_eff_vel_vec;

    if (time >= 15){
      control_mode = 2;
    }

    if (control_mode == 1) {

      // Joint-Space control - robot arms
      for (int i = 0; i < int(left_arm_des_trq_vec.size()); i++){

        left_arm_des_trq_vec[i] = kp_arm_joint_space * (left_arm_des_pos_vec[i] - left_arm_cur_pos_vec[i]) + kd_arm_joint_space * (left_arm_des_vel_vec[i] - left_arm_cur_vel_vec[i]);
        right_arm_des_trq_vec[i] = kp_arm_joint_space * (right_arm_des_pos_vec[i] - right_arm_cur_pos_vec[i]) + kd_arm_joint_space * (right_arm_des_vel_vec[i] - right_arm_cur_vel_vec[i]);
      
      }    

      // Joint-Space control - robot hands
      for (int i = 0; i < int(left_hand_des_trq_vec.size()); i++){
      
        left_hand_des_trq_vec[i] = kp_hand * (left_hand_des_pos_vec[i] - left_hand_cur_pos_vec[i]) + kd_hand * (left_hand_des_vel_vec[i] - left_hand_cur_vel_vec[i]);
        right_hand_des_trq_vec[i] = kp_hand * (right_hand_des_pos_vec[i] - right_hand_cur_pos_vec[i]) + kd_hand * (right_hand_des_vel_vec[i] - right_hand_cur_vel_vec[i]);
      
      }  
    } else if (control_mode == 2){

      Kp_matrix(0,0) = Kp_Matrix_pos; Kp_matrix(1,1) = Kp_Matrix_pos; Kp_matrix(2,2) = Kp_Matrix_pos;
      Kp_matrix(3,3) = Kp_Matrix_ori; Kp_matrix(4,4) = Kp_Matrix_ori; Kp_matrix(5,5) = Kp_Matrix_ori;

      Kd_matrix(0,0) = Kd_Matrix_pos; Kd_matrix(1,1) = Kd_Matrix_pos; Kd_matrix(2,2) = Kd_Matrix_pos;
      Kd_matrix(3,3) = Kd_Matrix_ori; Kd_matrix(4,4) = Kd_Matrix_ori; Kd_matrix(5,5) = Kd_Matrix_ori;

      // Task-Space control
      left_arm_des_trq_vec  = left_arm_jacobian_matrix.transpose() * (Kp_matrix * (left_arm_end_eff_pose_error) + Kd_matrix * (left_arm_end_eff_vel_error));
      right_arm_des_trq_vec = right_arm_jacobian_matrix.transpose() * (Kp_matrix * (right_arm_end_eff_pose_error) + Kd_matrix * (right_arm_end_eff_vel_error));

      // Joint-Space control - robot hands
      for (int i = 0; i < int(left_hand_des_trq_vec.size()); i++){
      
        left_hand_des_trq_vec[i] = kp_hand * (left_hand_des_pos_vec[i] - left_hand_cur_pos_vec[i]) + kd_hand * (left_hand_des_vel_vec[i] - left_hand_cur_vel_vec[i]);
        right_hand_des_trq_vec[i] = kp_hand * (right_hand_des_pos_vec[i] - right_hand_cur_pos_vec[i]) + kd_hand * (right_hand_des_vel_vec[i] - right_hand_cur_vel_vec[i]);
      
      }  
    }
  
    // Compute Jacobian for left and right arms
    computeJacobian();

    // Compute forward kinematics for left and right arms
    computeForwardKinematics();

    // Printing Robot Data
    printRobotData();

    // Publishing joint messages
    alex_pub->publish(joint_message);

  }

  // Function to compute the numerical Jacobian matrices for both arms
  void computeNumericalJacobian(Eigen::MatrixXd& jacobian_matrix, KDL::ChainFkSolverPos_recursive& fk_solver, Eigen::VectorXd& joint_pos_vec, Eigen::VectorXd& end_eff_pose_vec, KDL::Chain& chain) {

    const double eps = 1e-6;  // Small perturbation
    int num_joints = chain.getNrOfJoints();
    KDL::JntArray joint_positions(num_joints);

    // Initialize joint positions
    for (int i = 0; i < num_joints; ++i) {
        joint_positions(i) = joint_pos_vec[i];
    }

    // Loop over each joint and calculate partial derivatives
    for (int j = 0; j < num_joints; ++j) {

        // Store original joint position
        double original_position = joint_positions(j);

        // Perturb positively
        joint_positions(j) = original_position + eps;
        KDL::Frame perturbed_pose_plus;
        fk_solver.JntToCart(joint_positions, perturbed_pose_plus);

        // Perturb negatively
        joint_positions(j) = original_position - eps;
        KDL::Frame perturbed_pose_minus;
        fk_solver.JntToCart(joint_positions, perturbed_pose_minus);

        // Reset joint position
        joint_positions(j) = original_position;

        // Compute numerical derivative for each component of the pose
        for (int i = 0; i < 3; ++i) {  // Position (x, y, z)
            jacobian_matrix(i, j) = (perturbed_pose_plus.p[i] - perturbed_pose_minus.p[i]) / (2 * eps);
        }

        // Extract orientation as RPY and compute numerical derivative for each component
        double roll_plus, pitch_plus, yaw_plus;
        perturbed_pose_plus.M.GetRPY(roll_plus, pitch_plus, yaw_plus);

        double roll_minus, pitch_minus, yaw_minus;
        perturbed_pose_minus.M.GetRPY(roll_minus, pitch_minus, yaw_minus);

        jacobian_matrix(3, j) = (roll_plus - roll_minus) / (2 * eps);
        jacobian_matrix(4, j) = (pitch_plus - pitch_minus) / (2 * eps);
        jacobian_matrix(5, j) = (yaw_plus - yaw_minus) / (2 * eps);
    }
  }

  // Function to compute Jacobian matrices for both arms
  void computeJacobian() {
    
    if (jacobian_calculation_method == 1){
      // Call the KDL Jacobian function for both arms
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
    } else if (jacobian_calculation_method == 2){

      // Call the numerical Jacobian function for both arms
      // Left arm numerical Jacobian
      computeNumericalJacobian(left_arm_jacobian_matrix, *left_arm_fk_solver, left_arm_cur_pos_vec, left_arm_cur_end_eff_pose_vec, left_arm_chain);

      // Right arm numerical Jacobian
      computeNumericalJacobian(right_arm_jacobian_matrix, *right_arm_fk_solver, right_arm_cur_pos_vec, right_arm_cur_end_eff_pose_vec, right_arm_chain);
    }
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

        left_arm_ee_frame.M.GetRPY(left_arm_roll, left_arm_pitch, left_arm_yaw);
    } else {

        RCLCPP_ERROR(this->get_logger(), "Failed to compute forward kinematics for left arm.");
    }
    if (right_arm_fk_solver->JntToCart(right_arm_joint_positions, right_arm_ee_frame) >= 0) {

        right_arm_ee_frame.M.GetRPY(right_arm_roll, right_arm_pitch, right_arm_yaw);
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
  }

  void printRobotData(){

    std::cout << std::endl << "Time: " << time << std::endl << std::endl;

    std::cout << "Left Arm Jacobian: \n" << setprecision(5) << left_arm_jacobian_matrix << std::endl;
    std::cout << "\nRight Arm Jacobian: \n" << right_arm_jacobian_matrix << std::endl;

    std::cout << std::endl << "Left  Arm End-Effector Current Pose: " << "\tP.x = " << left_arm_cur_end_eff_pose_vec[0] << "\tP.y = " << left_arm_cur_end_eff_pose_vec[1] << "\tP.z = " << left_arm_cur_end_eff_pose_vec[2] 
                                                             << "\tO.x = " << left_arm_cur_end_eff_pose_vec[3] * 180/M_PI << "\tO.y = " << left_arm_cur_end_eff_pose_vec[4] * 180/M_PI << "\tO.z = " << left_arm_cur_end_eff_pose_vec[5] * 180/M_PI << std::endl;

    std::cout << "Right Arm End-Effector Current Pose: " << "\tP.x = " << right_arm_cur_end_eff_pose_vec[0] << "\tP.y = " << right_arm_cur_end_eff_pose_vec[1] << "\tP.z = " << right_arm_cur_end_eff_pose_vec[2] 
                                                 << "\tO.x = " << right_arm_cur_end_eff_pose_vec[3] * 180/M_PI << "\tO.y = " << right_arm_cur_end_eff_pose_vec[4] * 180/M_PI << "\tO.z = " << right_arm_cur_end_eff_pose_vec[5] * 180/M_PI << std::endl;

    std::cout << std::endl << "Left  Arm End-effector Pose error: " << "\teP.x = " << left_arm_end_eff_pose_error[0] << "\teP.y = " << left_arm_end_eff_pose_error[1] << "\teP.z = " << left_arm_end_eff_pose_error[2] 
                                                             << "\teO.x = " << left_arm_end_eff_pose_error[3] * 180/M_PI << "\teO.y = " << left_arm_end_eff_pose_error[4] * 180/M_PI << "\teO.z = " << left_arm_end_eff_pose_error[5] * 180/M_PI << std::endl;

    std::cout << "Right Arm End-effector Pose error: " << "\teP.x = " << right_arm_end_eff_pose_error[0] << "\teP.y = " << right_arm_end_eff_pose_error[1] << "\teP.z = " << right_arm_end_eff_pose_error[2] 
                                                 << "\teO.x = " << right_arm_end_eff_pose_error[3] * 180/M_PI << "\teO.y = " << right_arm_end_eff_pose_error[4] * 180/M_PI << "\teO.z = " << right_arm_end_eff_pose_error[5] * 180/M_PI << std::endl;

    std::cout << std::endl << "Left  Arm End-Effector Current Velocity: " << left_arm_cur_end_eff_vel_vec.transpose() << std::endl;
    std::cout << "Right Arm End-Effector Current Velocity: " << right_arm_cur_end_eff_vel_vec.transpose() << std::endl;

    std::cout << std::endl << "Left  Arm Joint Torques:  = " << left_arm_des_trq_vec.transpose() << std::endl;
    std::cout << "Right Arm Joint Torques: = " << right_arm_des_trq_vec.transpose() << std::endl;
  }

  private:

  double kp_arm_joint_space, kd_arm_joint_space, kp_hand, kd_hand, Kp_Matrix_pos, Kp_Matrix_ori, Kd_Matrix_pos, Kd_Matrix_ori; 
  double frequency, time, amplitude, target_position_sin, target_velocity_cos, target_position_cos, target_velocity_sin, trajectory_tracking_activation;
  double control_mode, jacobian_calculation_method;

  double left_arm_roll, left_arm_pitch, left_arm_yaw;
  double right_arm_roll, right_arm_pitch, right_arm_yaw;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr alex_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr alex_sub;

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::time_point<std::chrono::steady_clock> start_time;

  sensor_msgs::msg::JointState joint_message;
  
  Eigen::VectorXd left_arm_des_pos_vec;
  Eigen::VectorXd left_arm_des_vel_vec;
  Eigen::VectorXd left_arm_cur_pos_vec;
  Eigen::VectorXd left_arm_cur_vel_vec;
  Eigen::VectorXd left_arm_des_trq_vec;

  Eigen::VectorXd right_arm_des_pos_vec;
  Eigen::VectorXd right_arm_des_vel_vec;
  Eigen::VectorXd right_arm_cur_pos_vec;
  Eigen::VectorXd right_arm_cur_vel_vec;
  Eigen::VectorXd right_arm_des_trq_vec;

  Eigen::VectorXd left_hand_des_pos_vec;
  Eigen::VectorXd left_hand_des_vel_vec;
  Eigen::VectorXd left_hand_cur_pos_vec;
  Eigen::VectorXd left_hand_cur_vel_vec;
  Eigen::VectorXd left_hand_des_trq_vec;

  Eigen::VectorXd right_hand_des_pos_vec;
  Eigen::VectorXd right_hand_des_vel_vec;
  Eigen::VectorXd right_hand_cur_pos_vec;
  Eigen::VectorXd right_hand_cur_vel_vec;
  Eigen::VectorXd right_hand_des_trq_vec;

  Eigen::VectorXd left_arm_des_end_eff_pose_vec;
  Eigen::VectorXd left_arm_cur_end_eff_pose_vec;
  Eigen::VectorXd left_arm_des_end_eff_vel_vec;
  Eigen::VectorXd left_arm_cur_end_eff_vel_vec;

  Eigen::VectorXd left_arm_end_eff_pose_error;
  Eigen::VectorXd left_arm_end_eff_vel_error;

  Eigen::VectorXd right_arm_des_end_eff_pose_vec;
  Eigen::VectorXd right_arm_cur_end_eff_pose_vec;
  Eigen::VectorXd right_arm_des_end_eff_vel_vec;
  Eigen::VectorXd right_arm_cur_end_eff_vel_vec;

  Eigen::VectorXd right_arm_end_eff_pose_error;
  Eigen::VectorXd right_arm_end_eff_vel_error;

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

  // Stiffness and Damping matrices
  Eigen::MatrixXd Kp_matrix;
  Eigen::MatrixXd Kd_matrix;

};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlexRobot>());
  rclcpp::shutdown();

  printf("alex_robot_package package executed sucessfully\n");
  return 0;
}