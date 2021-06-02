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

#ifndef SYSTEM_INTERFACE
#define SYSTEM_INTERFACE

#include "staubli_val3_driver/simple_message/set_drive_power_message.h"

#include "industrial_msgs/SetDrivePower.h"
#include "simple_message/socket/simple_socket.h"
#include "simple_message/smpl_msg_connection.h"

#include <string>

namespace staubli
{
namespace system_interface
{

class SystemInterface
{
public:
  SystemInterface();

  ~SystemInterface();

  bool init(const std::string& default_ip = "",
            int default_port = industrial::simple_socket::StandardSocketPorts::SYSTEM);

  void run();

  bool setDrivePowerCb(industrial_msgs::SetDrivePower::Request& req, industrial_msgs::SetDrivePower::Response& res);

  bool setDrivePower(bool value);

private:
  std::shared_ptr<industrial::smpl_msg_connection::SmplMsgConnection> connection_;
};

}  // namespace system_interface
}  // namespace staubli

#endif  // SYSTEM_INTERFACE
