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

#include "staubli_val3_driver/simple_message/set_drive_power_message.h"

#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::byte_array;
using namespace industrial::shared_types;
using namespace industrial::simple_message;

namespace staubli
{
namespace simple_message
{

SetDrivePowerMessage::SetDrivePowerMessage()
{
  this->init();
}

SetDrivePowerMessage::~SetDrivePowerMessage()
{
}

bool SetDrivePowerMessage::init(industrial::simple_message::SimpleMessage& msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->drive_power_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload SetDrivePower data");
  }
  return rtn;
}

void SetDrivePowerMessage::init()
{
  this->setMessageType(1610);  // TODO: make enum for Stäubli specific standard port numbers
}

void SetDrivePowerMessage::init(shared_bool drive_power)
{
  this->init();
  this->drive_power_ = drive_power;
}

bool SetDrivePowerMessage::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  LOG_COMM("Executing SetDrivePower load");

  if (buffer->load(this->drive_power_))
  {

    LOG_COMM("SetDrivePower successfully loaded");
    rtn = true;
  }
  else
  {
    LOG_COMM("SetDrivePower not loaded");
    rtn = false;
  }

  return rtn;
}

bool SetDrivePowerMessage::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  LOG_COMM("Executing SetDrivePower unload");
  if (buffer->unload(this->drive_power_))
  {

    rtn = true;
    LOG_COMM("SetDrivePower successfully unloaded");
  }

  else
  {
    LOG_ERROR("Failed to unload SetDrivePower");
    rtn = false;
  }

  return rtn;
}

}  // namespace simple_message
}  // namespace staubli
