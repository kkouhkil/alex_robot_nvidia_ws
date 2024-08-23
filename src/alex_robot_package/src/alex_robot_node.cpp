#include <cstdio>
#include <iostream>

#include <vector>

#include <ctime>
#include <chrono>

using namespace std;

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class AlexRobot : public rclcpp::Node{

  public:

  AlexRobot() : Node("alex_robot"){

    alex_pub = this->create_publisher<sensor_msgs::msg::JointState>( "joint_command", 10);

    joint_message = sensor_msgs::msg::JointState();
    joint_message.name = {"LeftShoulderPitch", "RightShoulderPitch", "LeftShoulderRoll", "RightShoulderRoll", "LeftShoulderYaw", "RightShoulderYaw","LeftElbowPitch",
                          "RightElbowPitch", "LeftWristYaw", "RightWristYaw", "LeftWristRoll", "RightWristRoll", "LeftGripperYaw", "RightGripperYaw",
                          "Left_index_q1", "Left_middle_q1", "Left_pinky_q1", "Left_ring_q1", "Left_thumb_q1",
                          "Right_index_q1", "Right_middle_q1", "Right_pinky_q1", "Right_ring_q1", "Right_thumb_q1",
                          "Left_index_q2", "Left_middle_q2", "Left_pinky_q2", "Left_ring_q2", "Left_thumb_q2",
                          "Right_index_q2", "Right_middle_q2", "Right_pinky_q2", "Right_ring_q2", "Right_thumb_q2"
};

    // left_arm  =  joint_message.position[0, 2, 4, 6, 8, 10, 12]
    left_arm_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_arm_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // right_arm =  joint_message.position[1, 3, 5, 7, 9, 11, 13]
    right_arm_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   
    right_arm_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // left_hand  = joint_message.position[14, 15, 16, 17, 18, 24, 25, 26, 27, 28]
    left_hand_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_hand_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // right_hand = joint_message.position[19, 20, 21, 22, 23, 29, 30, 31, 32, 33]
    right_hand_pos_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    right_hand_vel_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

    joint_message.position = {0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 
                              0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180,
                              0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180,
                              0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180};

    joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    step = 20;
    frequency = 1;
    amplitude = 2;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&AlexRobot::jointCallback, this));
  }

  void jointCallback(){

    for (int i = 0; i < left_arm_pos_vec.size(); i++){

      joint_message.position[2 * i] = left_arm_pos_vec[i];
      joint_message.velocity[2 * i] = left_arm_vel_vec[i];

      joint_message.position[2 * i + 1] = right_arm_pos_vec[i];
      joint_message.velocity[2 * i + 1] = right_arm_vel_vec[i];
    }

    for (int i = 0; i < left_hand_pos_vec.size()/2; i++){
      joint_message.position[i + 14] =  left_hand_pos_vec[i];
      joint_message.position[i + 24] =  left_hand_pos_vec[i + 5];
      joint_message.velocity[i + 14] =  left_hand_vel_vec[i];
      joint_message.velocity[i + 24] =  left_hand_vel_vec[i + 5];

      joint_message.position[i + 19] =  right_hand_pos_vec[i];
      joint_message.position[i + 29] =  right_hand_pos_vec[i + 5];
      joint_message.velocity[i + 19] =  right_hand_vel_vec[i];
      joint_message.velocity[i + 29] =  right_hand_vel_vec[i + 5];
    }

    RCLCPP_INFO(this->get_logger(), "Callback function running");

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time;
    time = elapsed.count(); // time in seconds
    
    target_position = sin(amplitude * M_PI * frequency * time);
    target_velocity = amplitude * M_PI * cos(2 * M_PI * frequency * time);

    left_arm_pos_vec[0] = target_position;
    left_arm_vel_vec[0] = target_velocity;

    right_arm_pos_vec[0] = target_position;
    right_arm_vel_vec[0] = target_velocity;

    left_hand_pos_vec[0] = target_position;
    left_hand_vel_vec[0] = target_velocity;

    right_hand_pos_vec[0] = target_position;
    right_hand_vel_vec[0] = target_velocity;

    // for(double &pos: joint_message.position){
    //   pos = target_position;
    // }

    // for(double &pos: joint_message.velocity){
    //   pos = target_velocity;
    // }

    alex_pub->publish(joint_message);

  }

  private:

  double frequency, time, amplitude, step, target_position, target_velocity;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr alex_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_message;
  std::chrono::time_point<std::chrono::steady_clock> start_time;

  std::vector<double> left_arm_pos_vec;
  std::vector<double> left_arm_vel_vec;

  std::vector<double> right_arm_pos_vec;
  std::vector<double> right_arm_vel_vec;

  std::vector<double> left_hand_pos_vec;
  std::vector<double> left_hand_vel_vec;

  std::vector<double> right_hand_pos_vec;
  std::vector<double> right_hand_vel_vec;

};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlexRobot>());
  rclcpp::shutdown();

  printf("hello world alex_robot_package package\n");
  return 0;
}

