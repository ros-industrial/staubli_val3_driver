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

#include "staubli_val3_driver/system_interface.h"
#include "staubli_val3_driver/simple_message/set_drive_power_message.h"

#include "ros/ros.h"
#include "industrial_msgs/SetDrivePower.h"
#include "simple_message/simple_message.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/tcp_client.h"

#include <string>

using namespace industrial::simple_message;
using namespace industrial::tcp_client;
using namespace staubli::simple_message;

namespace staubli
{
namespace system_interface
{

SystemInterface::SystemInterface() : connection_(nullptr)
{
}

SystemInterface::~SystemInterface()
{
}

bool SystemInterface::init(const std::string& default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  ros::param::param<std::string>("robot_ip_address", ip, default_ip);
  ros::param::param<int>("~port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found. Please set the 'robot_ip_address' parameter");
    return false;
  }
  if (port <= 0 || port > 65535)
  {
    ROS_ERROR("No valid robot IP port found. Please set the '~port' parameter");
    return false;
  }

  // connection.init() requires "char*", not "const char*"
  char* ip_addr = strdup(ip.c_str());

  // create and connect client connection
  auto client = std::make_shared<TcpClient>();
  bool rtn = client->init(ip_addr, port);
  free(ip_addr);

  if (!rtn)
    return false;

  this->connection_ = client;
  ROS_INFO("system_interface: Connecting (%s:%d)", ip_addr, port);

  return this->connection_->makeConnect();
}

void SystemInterface::run()
{
  ros::NodeHandle nh;
  ros::ServiceServer srv = nh.advertiseService("set_drive_power", &SystemInterface::setDrivePowerCb, this);
  ROS_INFO_STREAM("Service " << srv.getService() << " is ready and running");
  ros::spin();
}

bool SystemInterface::setDrivePowerCb(industrial_msgs::SetDrivePower::Request& req,
                                      industrial_msgs::SetDrivePower::Response& res)
{
  if (req.drive_power)
    ROS_INFO("Setting drive power ON");
  else
    ROS_INFO("Setting drive power OFF");

  if (setDrivePower(req.drive_power))
  {
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
    return true;
  }
  else
  {
    res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
    return false;
  }
}

bool SystemInterface::setDrivePower(bool value)
{
  SetDrivePowerMessage msg;
  SimpleMessage send, reply;
  msg.init(value);
  msg.toRequest(send);
  connection_->sendAndReceiveMsg(send, reply);
  return reply.getReplyCode() == ReplyType::SUCCESS;
}

}  // namespace system_interface
}  // namespace staubli
