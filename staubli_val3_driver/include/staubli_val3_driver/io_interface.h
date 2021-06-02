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

#ifndef IO_INTERFACE
#define IO_INTERFACE

#include "staubli_val3_driver/simple_message/write_single_io.h"

#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/simple_socket.h"
#include "staubli_msgs/WriteSingleIO.h"

#include <string>

namespace staubli
{
namespace io_interface
{

class IOInterface
{
public:
  static const std::map<int, std::string> MODULE_NAMES;

  IOInterface();

  ~IOInterface();

  bool init(const std::string& default_ip = "", int default_port = industrial::simple_socket::StandardSocketPort::IO);

  void run();

  bool writeSingleIO(staubli_msgs::WriteSingleIO::Request& req, staubli_msgs::WriteSingleIO::Response& res);

  bool writeSingleIO(staubli::simple_message::IOModule moduleId, int pin, bool state);

private:
  std::shared_ptr<industrial::smpl_msg_connection::SmplMsgConnection> connection_;
};

}  // namespace io_interface
}  // namespace staubli

#endif  // IO_INTERFACE
