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

#include "simple_message/log_wrapper.h"

#include <algorithm>

using namespace industrial::joint_feedback_message;
using namespace industrial::shared_types;
using namespace industrial::simple_message;
using namespace industrial::smpl_msg_connection;

namespace industrial_robot_client
{
namespace joint_feedback_relay_handler
{

bool JointFeedbackRelayHandler::init(SmplMsgConnection* connection, std::vector<std::string>& joint_names)
{
  this->pub_joint_control_state_ =
      this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);

  this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states", 1);

  // save "complete" joint-name list, preserving any blank entries for later use
  this->all_joint_names_ = joint_names;

  return init(static_cast<int>(StandardMsgTypes::JOINT_FEEDBACK), connection);
}

bool JointFeedbackRelayHandler::internalCB(SimpleMessage& in)
{
  JointFeedbackMessage joint_fbk_msg;

  if (!joint_fbk_msg.init(in))
  {
    LOG_ERROR("Failed to initialize joint feedback message");
    return false;
  }

  return internalCB(joint_fbk_msg);
}

bool JointFeedbackRelayHandler::internalCB(JointFeedbackMessage& in)
{
  control_msgs::FollowJointTrajectoryFeedback control_state;
  sensor_msgs::JointState sensor_state;
  bool rtn = true;

  if (createMessages(in, &control_state, &sensor_state))
  {
    this->pub_joint_control_state_.publish(control_state);
    this->pub_joint_sensor_state_.publish(sensor_state);
  }
  else
    rtn = false;

  // Reply back to the controller if the sender requested it.
  if (CommTypes::SERVICE_REQUEST == in.getMessageType())
  {
    SimpleMessage reply;
    in.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}

bool JointFeedbackRelayHandler::createMessages(JointFeedbackMessage& msg_in,
                                               control_msgs::FollowJointTrajectoryFeedback* control_state,
                                               sensor_msgs::JointState* sensor_state)
{
  // read joint positions/velocities from JointFeedbackMessage
  std::vector<double> all_joint_pos(all_joint_names_.size());
  std::vector<double> all_joint_vel(all_joint_names_.size());

  industrial::joint_data::JointData pos;
  industrial::joint_data::JointData vel;

  // check if valid
  bool has_pos = msg_in.getPositions(pos);
  bool has_vel = msg_in.getVelocities(vel);

  if (has_pos)
  {
    for (int i = 0; i < all_joint_names_.size(); ++i)
    {
      shared_real value;

      if (pos.getJoint(i, value))
        all_joint_pos[i] = value;
      else
        LOG_ERROR("Failed to parse position value #%d from JointFeedbackMessage", i);
    }
  }

  if (has_vel)
  {
    for (int i = 0; i < all_joint_names_.size(); ++i)
    {
      shared_real value;

      if (vel.getJoint(i, value))
        all_joint_vel[i] = value;
      else
        LOG_ERROR("Failed to parse velocity value #%d from JointFeedbackMessage", i);
    }
  }

  // transform joint data?

  // select specific joints for publishing
  std::vector<double> pub_joint_pos;
  std::vector<double> pub_joint_vel;
  std::vector<std::string> pub_joint_names;

  if (has_pos)
  {
    if (!select(all_joint_pos, all_joint_names_, &pub_joint_pos, &pub_joint_names))
    {
      LOG_ERROR("Failed to select joint positions for publishing");
      return false;
    }
  }

  if (has_vel)
  {
    if (!select(all_joint_vel, all_joint_names_, &pub_joint_vel, &pub_joint_names))
    {
      LOG_ERROR("Failed to select joint velocities for publishing");
      return false;
    }
  }

  // assign values to messages
  control_msgs::FollowJointTrajectoryFeedback tmp_control_state;  // always start with a "clean" message
  tmp_control_state.header.stamp = ros::Time::now();
  tmp_control_state.joint_names = pub_joint_names;
  if (has_pos)
    tmp_control_state.actual.positions = pub_joint_pos;
  if (has_vel)
    tmp_control_state.actual.velocities = pub_joint_vel;
  *control_state = tmp_control_state;

  sensor_msgs::JointState tmp_sensor_state;
  tmp_sensor_state.header.stamp = ros::Time::now();
  tmp_sensor_state.name = pub_joint_names;
  if (has_pos)
    tmp_sensor_state.position = pub_joint_pos;
  if (has_vel)
    tmp_sensor_state.velocity = pub_joint_vel;
  *sensor_state = tmp_sensor_state;

  return true;
}

bool JointFeedbackRelayHandler::select(const std::vector<double>& all_joint_pos,
                                       const std::vector<std::string>& all_joint_names,
                                       std::vector<double>* pub_joint_pos, std::vector<std::string>* pub_joint_names)
{
  ROS_ASSERT(all_joint_pos.size() == all_joint_names.size());

  pub_joint_pos->clear();
  pub_joint_names->clear();

  // skip over "blank" joint names
  for (int i = 0; i < all_joint_pos.size(); ++i)
  {
    if (all_joint_names[i].empty())
      continue;

    pub_joint_pos->push_back(all_joint_pos[i]);
    pub_joint_names->push_back(all_joint_names[i]);
  }

  return true;
}

}  // namespace joint_feedback_relay_handler
}  // namespace industrial_robot_client
