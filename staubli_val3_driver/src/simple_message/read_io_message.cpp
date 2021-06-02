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

#include "staubli_val3_driver/simple_message/read_io_message.h"

#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#include "simple_message/simple_message.h"

using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace staubli
{
namespace simple_message
{

ReadIOMessage::ReadIOMessage()
{
  this->init();
}

ReadIOMessage::~ReadIOMessage()
{
}

bool ReadIOMessage::init(industrial::simple_message::SimpleMessage& msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->states_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload read IO data");
  }
  return rtn;
}

void ReadIOMessage::init()
{
  this->setMessageType(1620);  // TODO: make enum for Stäubli specific standard port numbers
}

bool ReadIOMessage::load(ByteArray* buffer)
{
  bool rtn = false;
  LOG_COMM("Executing read IO message load");
  if (buffer->load(this->states_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to read IO states data");
  }
  return rtn;
}

bool ReadIOMessage::unload(ByteArray* buffer)
{
  bool rtn = false;
  LOG_COMM("Executing read IO message unload");

  if (buffer->unload(this->states_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload read IO data");
  }
  return rtn;
}

}  // namespace simple_message
}  // namespace staubli
