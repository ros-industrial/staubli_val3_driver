/*
 * Copyright 2021 Institute for Factory Automation and Production Systems (FAPS)
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
 */

#include "staubli_val3_driver/industrial_robot_client/joint_feedback_relay_handler.h"

#include "industrial_robot_client/robot_state_interface.h"

#include "ros/ros.h"

using industrial_robot_client::joint_feedback_relay_handler::JointFeedbackRelayHandler;
using industrial_robot_client::robot_state_interface::RobotStateInterface;

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "robot_state_interface");

  // launch the default Robot State Interface connection/handlers
  RobotStateInterface rsi;
  rsi.init();

  // add the JointFeedback handler
  JointFeedbackRelayHandler joint_fbk_handler;
  std::vector<std::string> joint_names = rsi.get_joint_names();
  joint_fbk_handler.init(rsi.get_connection(), joint_names);
  rsi.add_handler(&joint_fbk_handler);

  // run the node
  rsi.run();

  return 0;
}
