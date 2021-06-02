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

#ifndef JOINT_FEEDBACK_HANDLER_H
#define JOINT_FEEDBACK_HANDLER_H

#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/message_handler.h"
#include "simple_message/messages/joint_feedback_message.h"

#include <string>
#include <vector>

namespace industrial_robot_client
{
namespace joint_feedback_relay_handler
{

/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointFeedbackRelayHandler : public industrial::message_handler::MessageHandler
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;

public:
  /**
   * \brief Constructor
   */
  JointFeedbackRelayHandler(){};

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send replies.
   * \param joint_names list of joint-names for msg-publishing.
   *   - Count and order should match data from robot connection.
   *   - Use blank-name to exclude a joint from publishing.
   *
   * \return true on success, false otherwise (an invalid message type)
   */
  bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::vector<std::string>& joint_names);

protected:
  std::vector<std::string> all_joint_names_;

  ros::Publisher pub_joint_control_state_;
  ros::Publisher pub_joint_sensor_state_;
  ros::NodeHandle node_;

  /**
   * \brief Convert joint feedback message into publish message-types
   *
   * \param[in] msg_in JointFeedback message from robot connection
   * \param[out] control_state FollowJointTrajectoryFeedback message for ROS publishing
   * \param[out] sensor_state JointState message for ROS publishing
   *
   * \return true on success, false otherwise
   */
  virtual bool createMessages(industrial::joint_feedback_message::JointFeedbackMessage& msg_in,
                              control_msgs::FollowJointTrajectoryFeedback* control_state,
                              sensor_msgs::JointState* sensor_state);

  /**
   * \brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] pos_in joint positions, exactly as passed from robot connection.
   * \param[out] pos_out transformed joint positions (in same order/count as input positions)
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
  {
    *pos_out = pos_in;  // by default, no transform is applied
    return true;
  }

  /**
   * \brief Select specific joints for publishing
   *
   * \param[in] all_joint_pos joint positions, in count/order matching robot connection
   * \param[in] all_joint_names joint names, matching all_joint_pos
   * \param[out] pub_joint_pos joint positions selected for publishing
   * \param[out] pub_joint_names joint names selected for publishing
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const std::vector<double>& all_joint_pos, const std::vector<std::string>& all_joint_names,
                      std::vector<double>* pub_joint_pos, std::vector<std::string>* pub_joint_names);

  /**
   * \brief Callback executed upon receiving a joint feedback message
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(industrial::joint_feedback_message::JointFeedbackMessage& in);

private:
  /**
   * \brief Callback executed upon receiving a message
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(industrial::simple_message::SimpleMessage& in);
};  // class JointFeedbackRelayHandler

}  // namespace joint_feedback_relay_handler
}  // namespace industrial_robot_client

#endif /* JOINT_FEEDBACK_HANDLER_H */
