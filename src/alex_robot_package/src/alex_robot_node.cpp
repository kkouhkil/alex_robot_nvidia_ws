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

    // left_arm  =  joint_message.position[0, 2, 4, 6, 8, 10, 12]
    left_arm_des_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_arm_des_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_arm_cur_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_arm_cur_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // right_arm =  joint_message.position[1, 3, 5, 7, 9, 11, 13]
    right_arm_des_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   
    right_arm_des_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    right_arm_cur_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   
    right_arm_cur_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // left_hand  = joint_message.position[14, 15, 16, 17, 18, 24, 25, 26, 27, 28]
    left_hand_des_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_hand_des_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_hand_cur_pos_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    left_hand_cur_vel_vec  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // right_hand = joint_message.position[19, 20, 21, 22, 23, 29, 30, 31, 32, 33]
    right_hand_des_pos_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    right_hand_des_vel_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    right_hand_cur_pos_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    right_hand_cur_vel_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 

    joint_message.position = {0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 
                              0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180,
                              0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180,
                              0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180, 0.0 * M_PI/180};

    joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    frequency = 0.5;
    amplitude = 2;
  }

  // Subscriber callback
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){

    // RCLCPP_INFO(this->get_logger(), "Received joint state update");
    
    // Process the received joint state message (msg)
    // You can update the vectors or process the data as needed

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

    // For demonstration, let's just print out the first joint name and position
    // if (!msg->name.empty() && !msg->position.empty()) {
      // RCLCPP_INFO(this->get_logger(), "First joint: %s, Position: %f", 
      //             msg->name[0].c_str(), msg->position[0]);
    // }

  }

  void jointCallback(){

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start_time;
    time = elapsed.count(); // time in seconds

    for (int i = 0; i < int(left_arm_des_pos_vec.size()); i++){

      joint_message.position[2 * i] = left_arm_des_pos_vec[i];
      joint_message.velocity[2 * i] = left_arm_des_vel_vec[i];

      joint_message.position[2 * i + 1] = right_arm_des_pos_vec[i];
      joint_message.velocity[2 * i + 1] = right_arm_des_vel_vec[i];
    }

    for (int i = 0; i < int(left_hand_des_pos_vec.size()/2); i++){
      joint_message.position[i + 14] =  left_hand_des_pos_vec[i];
      joint_message.position[i + 24] =  left_hand_des_pos_vec[i + 5];
      joint_message.velocity[i + 14] =  left_hand_des_vel_vec[i];
      joint_message.velocity[i + 24] =  left_hand_des_vel_vec[i + 5];

      joint_message.position[i + 19] =  right_hand_des_pos_vec[i];
      joint_message.position[i + 29] =  right_hand_des_pos_vec[i + 5];
      joint_message.velocity[i + 19] =  right_hand_des_vel_vec[i];
      joint_message.velocity[i + 29] =  right_hand_des_vel_vec[i + 5];
    }

    // RCLCPP_INFO(this->get_logger(), "Callback function running");
    
    target_position = sin(amplitude * M_PI * frequency * time);
    target_velocity = amplitude * M_PI * cos(2 * M_PI * frequency * time);

    left_arm_des_pos_vec[0] = target_position;
    left_arm_des_vel_vec[0] = target_velocity;

    right_arm_des_pos_vec[0] = target_position;
    right_arm_des_vel_vec[0] = target_velocity;

    left_hand_des_pos_vec[0] = target_position;
    left_hand_des_vel_vec[0] = target_velocity;

    right_hand_des_pos_vec[0] = target_position;
    right_hand_des_vel_vec[0] = target_velocity;

    // for(double &pos: joint_message.position){
    //   pos = target_position;
    // }

    // for(double &pos: joint_message.velocity){
    //   pos = target_velocity;
    // }

    alex_pub->publish(joint_message);

    std::cout << "Time: " << time << std::endl;

  }

  private:

  double frequency, time, amplitude, target_position, target_velocity;

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

};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlexRobot>());
  rclcpp::shutdown();

  printf("hello world alex_robot_package package\n");
  return 0;
}

