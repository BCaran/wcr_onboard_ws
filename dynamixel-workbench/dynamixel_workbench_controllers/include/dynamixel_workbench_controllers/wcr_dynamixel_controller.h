/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_CONTROLLERS_H
#define DYNAMIXEL_WORKBENCH_CONTROLLERS_H

#include <ros/ros.h>
#include <math.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include <dynamixel_workbench_controllers/trajectory_generator.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

//MOTOR'S ID AND NAMES
#define FL_WHEEL 1
#define FL_POSITION 5
#define FR_WHEEL 4
#define FR_POSITION 8
#define BL_WHEEL 2
#define BL_POSITION 6
#define BR_WHEEL 3
#define BR_POSITION 7

//MOTOR INIT POSITIONS
#define FL_INIT_POSITION 2048
#define FR_INIT_POSITION 2048
#define BL_INIT_POSITION 2048
#define BR_INIT_POSITION 2048

#define PI 3.14159265358979323846
#define PULS_PER_DEG 11.378f

// #define DEBUG

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber trajectory_sub_;

  // ROS Service Server
  ros::ServiceServer dynamixel_command_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;

  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;
  sensor_msgs::JointState joint_state_msg_;
  std::vector<WayPoint> pre_goal_;

  bool is_joint_state_topic_;
  bool is_cmd_vel_topic_;
  bool use_moveit_;
  bool is_wcr_robot_;

  double wcr_robot_length_;
  double wcr_robot_width_;
  double wcr_robot_wheel_radius_;
  double D_;
  double Rx_;
  double Ry_;
  double wheel_separation_;
  double wheel_radius_;

  JointTrajectory *jnt_tra_;
  trajectory_msgs::JointTrajectory *jnt_tra_msg_;

  double read_period_;
  double write_period_;
  double pub_period_;

  bool is_moving_;

 public:
  DynamixelController();
  ~DynamixelController();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool getPresentPosition(std::vector<std::string> dxl_name);

  double getReadPeriod(){return read_period_;}
  double getWritePeriod(){return write_period_;}
  double getPublishPeriod(){return pub_period_;}

  void initPublisher(void);
  void initSubscriber(void);

  void initServer();

  void readCallback(const ros::TimerEvent&);
  void writeCallback(const ros::TimerEvent&);
  void publishCallback(const ros::TimerEvent&);

  void initStartPosition();
  void optimazeSpeedAngle(double returnSpeedAngle[], double speed, double angle);

  void wcrCommandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg); 
  void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                   dynamixel_workbench_msgs::DynamixelCommand::Response &res);
};

#endif //DYNAMIXEL_WORKBENCH_CONTROLLERS_H
