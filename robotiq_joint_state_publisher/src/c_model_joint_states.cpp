// Copyright (c) 2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


/** @file s_model_joint_states.cpp
 *  Subscribes to Robotiq state messages on "SModelRobotInput" topic, converts the data to joint values,
 *  and publishes sensor_msgs/JointState messages on "joint_states" topic for Robotiq S-model.
 * 
 *  'rosrun robotiq_joint_state_publisher s_model_joint_states <gripper_prefix>'
 * 
 *  @author jack.thompson(at)utexas.edu
 *  @author karl.kruusamae(at)utexas.edu
 */

#include <vector>
#include <string>
#include <csignal>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <robotiq_c_model_control/CModel_robot_input.h>
#include <robotiq_c_model_control/CModel_robot_output.h>


const double DEG_TO_RAD = M_PI/180.0;

/**
 * Finger class for single Robotiq S-model finger.
 */
class Finger {
 public:
  Finger() { position = 0; }				///< Default constructor for creating Finger, position is set 0
  Finger(int pos) { position = pos; }			///< Create Finger, position is @param pos
  Finger(const Finger &f) { position = f.position; }	///< Create Finger, position is taken form @param Finger
  inline double joint1() const;				///< joint_1 value for Finger
  int position;						///< Position of the Finger
};

/**
 * Calculate joint1 based on the Finger position. Assumes that fingers are in a non-interfering state.
 * See Section 4.1 in https://www.cs.indiana.edu/ftp/techreports/TR711.pdf
 */
inline double Finger::joint1() const {
  return (45.0/255.0 * DEG_TO_RAD) * position;
  //if(0 <= position && position <= 140) return (70.0/148.0 * DEG_TO_RAD) * position;
  //else return 70.0 * DEG_TO_RAD;
}



/**
 * Robotiq S-model with three Fingers.
 */
class Robotiq3 {
 public:
  /** Default constructor for Robotiq3 */
  Robotiq3() {
    joint_positions.resize(11, 0.0);
    prefix = "";
  }
  
  /** Constructor for Robotiq3 */
  Robotiq3(std::string gripper_prefix) {
    joint_positions.resize(1, 0.0);
    prefix = gripper_prefix;    
  }
  void callback(const robotiq_c_model_control::CModel_robot_input::ConstPtr &msg);	///< Callback function for "SModelRobotInput" topic
  Finger finger_left;									///< Robotiq FINGER A
  std::string prefix;									///< Gripper prefix
  std::vector<std::string> jointNames();						///< Joint names
  std::vector<double> joint_positions;							///< Joint values
};

/**
 * Callback function for "SModelRobotInput" topic.
 */
void Robotiq3::callback(const robotiq_c_model_control::CModel_robot_input::ConstPtr &msg) {
  finger_left = Finger(msg->gPO);
  // Set all the joint values
  joint_positions.at(0)  =  finger_left.joint1();
}

/**
 * Assigns appropriate joint names.
 */
inline std::vector<std::string>  Robotiq3::jointNames() {
  // joint names for sensor_msgs::JointState message
  // order matters!
  std::vector<std::string> joint_names(1, "");
  joint_names.at(0).assign("finger_joint");
  return joint_names;
}

/**
 * Main method.
 */
int main(int argc, char *argv[]) {

  // set user-specified prefix
  std::string gripper_prefix;
  if (argc > 1) gripper_prefix = argv[1];
  else gripper_prefix = "";

  // Create Robotiq3
  Robotiq3 robotiq(gripper_prefix);
  
  // ROS init, nodehandle, and rate
  ros::init(argc, argv, "c_model_joint_states");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);  // Hz

  // joint state publisher
  ros::Publisher joint_pub;
  joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

  // robotiq state message subscriber
  ros::Subscriber joint_sub;
  joint_sub = nh.subscribe("CModelRobotInput", 10, &Robotiq3::callback, &robotiq);
  
  // Output JointState message
  sensor_msgs::JointState joint_msg;
  
  // Joint names to JointState message
  joint_msg.name = robotiq.jointNames();

  while (ros::ok()) {
    joint_msg.position = robotiq.joint_positions;
    joint_msg.header.stamp = ros::Time::now();
    joint_pub.publish(joint_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
