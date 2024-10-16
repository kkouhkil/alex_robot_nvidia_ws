#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <vector>
#include <ctime>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>

using namespace std;

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

    // Robot Model:
    // Model = 1 --> Alex Robot - Nub Hands
    // Model = 2 --> Alex Robot - Sake Hands
    // Model = 3 --> Alex Robot - Psyonic Hands
    robot_model = 1;

    // Publisher for joint_states
    alex_pub = this -> create_publisher<sensor_msgs::msg::JointState>( "joint_command", 10);
    timer_ = this -> create_wall_timer(std::chrono::milliseconds(20),
                                       std::bind(&AlexRobot::jointStatePubCallback, this));

    // Subscriber for joint_states
    alex_sub = this->create_subscription<sensor_msgs::msg::JointState>(
              "joint_states", 10, std::bind(&AlexRobot::jointStateSubCallback, this, std::placeholders::_1));

    joint_message = sensor_msgs::msg::JointState();

    if (robot_model == 1){

      // <-- Alex Robot - Nub Hands -->
      joint_message.name = {"SpineYaw", "SpineRoll", "SpinePitch", "LeftShoulderPitch", "NeckYaw", "RightShoulderPitch", "LeftShoulderRoll", 
                            "NeckPitch", "RightShoulderRoll", "LeftShoulderYaw", "RightShoulderYaw", "LeftElbowPitch", "RightElbowPitch", "LeftWristYaw", 
                            "RightWristYaw", "LeftWristRoll", "RightWristRoll", "LeftGripperYaw", "RightGripperYaw"
                            };

      // joint-message - Position
      joint_message.position = {0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 
                                0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180,
                                0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180
                                };

      // joint-message - Velocity
      joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0
                                };

      // joint-message - Effort
      joint_message.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0
                              };

    }else if (robot_model == 2){

      // <-- Alex Robot - Sake Hands -->
      joint_message.name = {"SpineYaw", "SpineRoll", "SpinePitch", "LeftShoulderPitch", "NeckYaw", "RightShoulderPitch", "LeftShoulderRoll", 
                              "NeckPitch", "RightShoulderRoll", "LeftShoulderYaw", "RightShoulderYaw", "LeftElbowPitch", "RightElbowPitch", "LeftWristYaw", 
                              "RightWristYaw", "LeftWristRoll", "RightWristRoll", "LeftGripperYaw", "RightGripperYaw", "Left_GRIPPER_X1", "Left_GRIPPER_X2", 
                              "Right_GRIPPER_X1", "Right_GRIPPER_X2"
                            };

      // joint-message - Velocity
      joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0
                                };

      // joint-message - Effort
      joint_message.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0
                              };

    } else if (robot_model == 3){
      
      // <-- Alex Robot - Psyonic Hands -->
      joint_message.name = {"SpineYaw", "SpineRoll", "SpinePitch", "LeftShoulderPitch", "NeckYaw", "RightShoulderPitch", "LeftShoulderRoll", 
                              "NeckPitch", "RightShoulderRoll", "LeftShoulderYaw", "RightShoulderYaw", "LeftElbowPitch", "RightElbowPitch", "LeftWristYaw", 
                              "RightWristYaw", "LeftWristRoll", "RightWristRoll", "LeftGripperYaw", "RightGripperYaw", "Left_index_q1", "Left_middle_q1", 
                              "Left_pinky_q1", "Left_ring_q1", "Left_thumb_q1", "Right_index_q1", "Right_middle_q1", "Right_pinky_q1", "Right_ring_q1", 
                              "Right_thumb_q1", "Left_index_q2", "Left_middle_q2", "Left_pinky_q2", "Left_ring_q2", "Left_thumb_q2", "Right_index_q2", 
                              "Right_middle_q2", "Right_pinky_q2", "Right_ring_q2", "Right_thumb_q2"
                            };

      // joint-message - Velocity
      joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0
                                };

      // joint-message - Effort
      joint_message.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0
                              };
    }

    // neck =  joint_message.position[4, 7]
    neck_des_pos_vec = Eigen::VectorXd(2);
    neck_cur_pos_vec = Eigen::VectorXd(2);
    neck_des_vel_vec = Eigen::VectorXd(2);
    neck_cur_vel_vec = Eigen::VectorXd(2);
    neck_des_trq_vec  = Eigen::VectorXd(2);

    neck_des_pos_vec << 0 * M_PI/180, 0 * M_PI/180;

    // left_arm  =  joint_message.position[3, 6, 9, 11, 13, 15, 17]
    left_arm_des_pos_vec  = Eigen::VectorXd(7);
    left_arm_des_vel_vec  = Eigen::VectorXd(7);
    left_arm_cur_pos_vec  = Eigen::VectorXd(7);
    left_arm_cur_vel_vec  = Eigen::VectorXd(7);
    left_arm_des_trq_vec  = Eigen::VectorXd(7);

    left_arm_des_pos_vec << -30 * M_PI/180, 75 * M_PI/180, 0.0, -60 * M_PI/180, 45 * M_PI/180, 0.0, 0.0;

    // right_arm =  joint_message.position[5, 8, 10, 12, 14, 16, 18]
    right_arm_des_pos_vec = Eigen::VectorXd(7);
    right_arm_des_vel_vec = Eigen::VectorXd(7);
    right_arm_cur_pos_vec = Eigen::VectorXd(7);
    right_arm_cur_vel_vec = Eigen::VectorXd(7);
    right_arm_des_trq_vec = Eigen::VectorXd(7);

    right_arm_des_pos_vec << -30 * M_PI/180, -75 * M_PI/180, 0.0, -60 * M_PI/180, -45 * M_PI/180, 0.0, 0.0;

    // spine =  joint_message.position[0, 1, 2]
    spine_des_pos_vec = Eigen::VectorXd(3);
    spine_cur_pos_vec = Eigen::VectorXd(3);
    spine_des_vel_vec = Eigen::VectorXd(3);
    spine_cur_vel_vec = Eigen::VectorXd(3);
    spine_des_trq_vec  = Eigen::VectorXd(3);

    spine_des_pos_vec << 0 * M_PI/180, 0 * M_PI/180, 0 * M_PI/180;

    // left_arm - end-effector - position - desired and current
    left_arm_des_end_eff_pose_vec = Eigen::VectorXd(6); 
    left_arm_cur_end_eff_pose_vec = Eigen::VectorXd(6);
    left_arm_end_eff_pose_error = Eigen::VectorXd(6);

    // right_arm - end-effector - position - desired and current
    right_arm_des_end_eff_pose_vec = Eigen::VectorXd(6);
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

    // neck - variable values initialisation
    neck_cur_pos_vec.setZero();
    neck_des_vel_vec.setZero();
    neck_cur_vel_vec.setZero();

    neck_des_trq_vec.setZero(); 

    // left_arm - variable values initialisation
    left_arm_des_vel_vec.setZero();
    left_arm_cur_pos_vec.setZero();
    left_arm_cur_vel_vec.setZero();

    left_arm_des_trq_vec.setZero();

    left_arm_des_end_eff_pose_vec.setZero();
    left_arm_cur_end_eff_pose_vec.setZero();
    left_arm_des_end_eff_vel_vec.setZero();
    left_arm_cur_end_eff_vel_vec.setZero();

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

    right_hand_des_trq_vec.setZero();

    // spine - variable values initialisation
    spine_cur_pos_vec.setZero();
    spine_des_vel_vec.setZero();
    spine_cur_vel_vec.setZero();

    spine_des_trq_vec.setZero(); 

    // controller PD gains - robot arms
    kp_neck_joint_space = 25;
    kd_neck_joint_space = 1;

    // controller PD gains - robot arms
    kp_arm_joint_space = 25;
    kd_arm_joint_space = 1;

    // controller PD gains - spine
    kp_spine_joint_space = 100;
    kd_spine_joint_space = 10;
  }

  // Publisher callback
  void jointStatePubCallback(){

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time;
    time = elapsed.count(); // time in seconds

    // neck =  joint_message[4, 7]
    joint_message.position[4] = neck_des_pos_vec[0];
    joint_message.position[7] = neck_des_pos_vec[1];

    joint_message.velocity[4] = neck_des_vel_vec[0];
    joint_message.velocity[7] = neck_des_vel_vec[1];

    joint_message.effort[4] = neck_des_trq_vec[0];
    joint_message.effort[7] = neck_des_trq_vec[1];

    // left_arm  =  joint_message[3, 6, 9, 11, 13, 15, 17]
    joint_message.position[3] = left_arm_des_pos_vec[0];
    joint_message.position[6] = left_arm_des_pos_vec[1];
    joint_message.position[9] = left_arm_des_pos_vec[2];
    joint_message.position[11] = left_arm_des_pos_vec[3];
    joint_message.position[13] = left_arm_des_pos_vec[4];
    joint_message.position[15] = left_arm_des_pos_vec[5];
    joint_message.position[17] = left_arm_des_pos_vec[6];

    joint_message.velocity[3] = left_arm_des_vel_vec[0];
    joint_message.velocity[6] = left_arm_des_vel_vec[1];
    joint_message.velocity[9] = left_arm_des_vel_vec[2];
    joint_message.velocity[11] = left_arm_des_vel_vec[3];
    joint_message.velocity[13] = left_arm_des_vel_vec[4];
    joint_message.velocity[15] = left_arm_des_vel_vec[5];
    joint_message.velocity[17] = left_arm_des_vel_vec[6];

    joint_message.effort[3] = left_arm_des_trq_vec[0];
    joint_message.effort[6] = left_arm_des_trq_vec[1];
    joint_message.effort[9] = left_arm_des_trq_vec[2];
    joint_message.effort[11] = left_arm_des_trq_vec[3];
    joint_message.effort[13] = left_arm_des_trq_vec[4];
    joint_message.effort[15] = left_arm_des_trq_vec[5];
    joint_message.effort[17] = left_arm_des_trq_vec[6];

    // right_arm =  joint_message[5, 8, 10, 12, 14, 16, 18]
    joint_message.position[5] = right_arm_des_pos_vec[0];
    joint_message.position[8] = right_arm_des_pos_vec[1];
    joint_message.position[10] = right_arm_des_pos_vec[2];
    joint_message.position[12] = right_arm_des_pos_vec[3];
    joint_message.position[14] = right_arm_des_pos_vec[4];
    joint_message.position[15] = right_arm_des_pos_vec[5];
    joint_message.position[18] = right_arm_des_pos_vec[6];

    joint_message.velocity[5] = right_arm_des_vel_vec[0];
    joint_message.velocity[8] = right_arm_des_vel_vec[1];
    joint_message.velocity[10] = right_arm_des_vel_vec[2];
    joint_message.velocity[12] = right_arm_des_vel_vec[3];
    joint_message.velocity[14] = right_arm_des_vel_vec[4];
    joint_message.velocity[15] = right_arm_des_vel_vec[5];
    joint_message.velocity[18] = right_arm_des_vel_vec[6];

    joint_message.effort[5] = right_arm_des_trq_vec[0];
    joint_message.effort[8] = right_arm_des_trq_vec[1];
    joint_message.effort[10] = right_arm_des_trq_vec[2];
    joint_message.effort[12] = right_arm_des_trq_vec[3];
    joint_message.effort[14] = right_arm_des_trq_vec[4];
    joint_message.effort[15] = right_arm_des_trq_vec[5];
    joint_message.effort[18] = right_arm_des_trq_vec[6];

    // spine =  joint_message[0, 1, 2]
    joint_message.position[0] = spine_des_pos_vec[0];
    joint_message.position[1] = spine_des_pos_vec[1];
    joint_message.position[2] = spine_des_pos_vec[2];

    joint_message.velocity[0] = spine_des_vel_vec[0];
    joint_message.velocity[1] = spine_des_vel_vec[1];
    joint_message.velocity[2] = spine_des_vel_vec[2];

    joint_message.effort[0] = spine_des_trq_vec[0];
    joint_message.effort[1] = spine_des_trq_vec[1];
    joint_message.effort[2] = spine_des_trq_vec[2];

    // Joint-Space control - neck
    neck_des_trq_vec = kp_neck_joint_space * (neck_des_pos_vec - neck_cur_pos_vec) + kd_neck_joint_space * (neck_des_vel_vec - neck_cur_vel_vec);

    // Joint-Space control - robot arms
    left_arm_des_trq_vec = kp_arm_joint_space * (left_arm_des_pos_vec - left_arm_cur_pos_vec) + kd_arm_joint_space * (left_arm_des_vel_vec - left_arm_cur_vel_vec);
    right_arm_des_trq_vec = kp_arm_joint_space * (right_arm_des_pos_vec - right_arm_cur_pos_vec) + kd_arm_joint_space * (right_arm_des_vel_vec - right_arm_cur_vel_vec);

    // Joint-Space control - spine
    spine_des_trq_vec = kp_spine_joint_space * (spine_des_pos_vec - spine_cur_pos_vec) + kd_spine_joint_space * (spine_des_vel_vec - spine_cur_vel_vec);

    // Printing Robot Data
    printRobotData();

    // Publishing joint messages
    alex_pub->publish(joint_message);
  }

  // Subscriber callback
  void jointStateSubCallback(const sensor_msgs::msg::JointState::SharedPtr msg){

    // neck =  joint_message.position[4, 7]
    neck_cur_pos_vec[0] = msg->position[4];
    neck_cur_pos_vec[1] = msg->position[7];

    // left_arm  =  joint_message.position[3, 6, 9, 11, 13, 15, 17]
    left_arm_cur_pos_vec[0] = msg->position[3];
    left_arm_cur_pos_vec[1] = msg->position[6];
    left_arm_cur_pos_vec[2] = msg->position[9];
    left_arm_cur_pos_vec[3] = msg->position[11];
    left_arm_cur_pos_vec[4] = msg->position[13];
    left_arm_cur_pos_vec[5] = msg->position[15];
    left_arm_cur_pos_vec[6] = msg->position[17];

    // right_arm =  joint_message.position[5, 8, 10, 12, 14, 16, 18]
    right_arm_cur_pos_vec[0] = msg->position[5];
    right_arm_cur_pos_vec[1] = msg->position[8];
    right_arm_cur_pos_vec[2] = msg->position[10];
    right_arm_cur_pos_vec[3] = msg->position[12];
    right_arm_cur_pos_vec[4] = msg->position[14];
    right_arm_cur_pos_vec[5] = msg->position[16];
    right_arm_cur_pos_vec[6] = msg->position[18];

    // spine =  joint_message.position[0, 1, 2]
    spine_cur_pos_vec[0] = msg->position[0];
    spine_cur_pos_vec[1] = msg->position[1];
    spine_cur_pos_vec[2] = msg->position[2];

  }

  void printRobotData(){

    std::cout << std::endl << "Time: " << time << std::endl;

    std::cout << std::endl << "Neck Current Joint Poisiton: " << neck_cur_pos_vec.transpose() << std::endl; 

    std::cout << std::endl << "Left Arm Current Joint Poisiton: " << left_arm_cur_pos_vec.transpose() << std::endl;

    std::cout << std::endl << "Right Arm Current Joint Poisiton: " << right_arm_cur_pos_vec.transpose() << std::endl;

    std::cout << std::endl << "Spine Current Joint Poisiton: " << spine_cur_pos_vec.transpose() << std::endl;
  }

private:

  int robot_model;
  double kp_neck_joint_space, kd_neck_joint_space, kp_arm_joint_space, kd_arm_joint_space, kp_spine_joint_space, kd_spine_joint_space, Kp_Matrix_pos, Kp_Matrix_ori, Kd_Matrix_pos, Kd_Matrix_ori; 
  double frequency, time, amplitude, target_position_sin, target_velocity_cos, target_position_cos, target_velocity_sin, trajectory_tracking_activation;
  double control_mode, jacobian_calculation_method;

  double left_arm_roll, left_arm_pitch, left_arm_yaw;
  double right_arm_roll, right_arm_pitch, right_arm_yaw;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr alex_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr alex_sub;

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::time_point<std::chrono::steady_clock> start_time;

  sensor_msgs::msg::JointState joint_message;

  Eigen::VectorXd spine_des_pos_vec;
  Eigen::VectorXd spine_cur_pos_vec;
  Eigen::VectorXd spine_des_vel_vec;
  Eigen::VectorXd spine_cur_vel_vec;
  Eigen::VectorXd spine_des_trq_vec;

  Eigen::VectorXd neck_des_pos_vec;
  Eigen::VectorXd neck_cur_pos_vec;
  Eigen::VectorXd neck_des_vel_vec;
  Eigen::VectorXd neck_cur_vel_vec;
  Eigen::VectorXd neck_des_trq_vec;
  
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

  printf("alex_robot_moveable_torso_package executed sucessfully\n");
  return 0;
}
