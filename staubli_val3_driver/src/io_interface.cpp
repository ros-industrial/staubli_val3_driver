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

#include "staubli_val3_driver/io_interface.h"
#include "staubli_val3_driver/simple_message/write_single_io_message.h"

#include "ros/ros.h"
#include "simple_message/simple_message.h"
#include "simple_message/socket/tcp_client.h"

#include <string>

using namespace industrial::simple_message;
using namespace industrial::tcp_client;
using namespace staubli::simple_message;

namespace staubli
{
namespace io_interface
{

// Initialize module names map
const std::map<int, std::string> IOInterface::MODULE_NAMES = {
  { staubli::simple_message::IOModule::USER_IN, "UserIn" },
  { staubli::simple_message::IOModule::BASIC_IN, "BasicIn" },
  { staubli::simple_message::IOModule::BASIC_OUT, "BasicOut" },
  { staubli::simple_message::IOModule::VALVE_OUT, "ValveOut" },
  { staubli::simple_message::IOModule::BASIC_IN_2, "BasicIn-2" },
  { staubli::simple_message::IOModule::BASIC_OUT_2, "BasicOut-2" }
};

IOInterface::IOInterface() : connection_(nullptr)
{
}

IOInterface::~IOInterface()
{
}

bool IOInterface::init(const std::string& default_ip, int default_port)
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
  ROS_INFO("io_interface: Connecting (%s:%d)", ip_addr, port);

  return this->connection_->makeConnect();
}

void IOInterface::run()
{
  ros::NodeHandle nh;
  ros::ServiceServer srv = nh.advertiseService("write_single_io", &IOInterface::writeSingleIO, this);
  ROS_INFO_STREAM("Service " << srv.getService() << " is ready and running");
  ros::spin();
}

bool IOInterface::writeSingleIO(staubli_msgs::WriteSingleIO::Request& req, staubli_msgs::WriteSingleIO::Response& res)
{
  std::string module_name;

  // lookup module name for requested module id
  if (MODULE_NAMES.find(req.module.id) != MODULE_NAMES.end())
  {
    module_name = MODULE_NAMES.at(req.module.id);
  }
  else
  {
    ROS_WARN("Unknown module id given in WriteSingleIO service request!");
    return false;
  }

  if (req.state)
  {
    ROS_INFO("Trying to set pin %d of module '%s'", req.pin, module_name.c_str());
  }
  else
  {
    ROS_INFO("Trying to clear pin %d of module '%s'", req.pin, module_name.c_str());
  }

  if (writeSingleIO(staubli::simple_message::IOModule(req.module.id), req.pin, req.state))
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

bool IOInterface::writeSingleIO(staubli::simple_message::IOModule moduleId, int pin, bool state)
{
  WriteSingleIO write_io;
  write_io.init(moduleId, pin, state);
  WriteSingleIOMessage msg;
  msg.init(write_io);
  SimpleMessage send, reply;
  msg.toRequest(send);
  connection_->sendAndReceiveMsg(send, reply);
  return (reply.getReplyCode() == ReplyTypes::SUCCESS);
}

}  // namespace io_interface
}  // namespace staubli
