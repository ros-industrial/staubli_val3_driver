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

#include "staubli_val3_driver/simple_message/write_single_io_message.h"

#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace staubli
{
namespace simple_message
{

WriteSingleIOMessage::WriteSingleIOMessage()
{
  this->init();
}

WriteSingleIOMessage::~WriteSingleIOMessage()
{
}

bool WriteSingleIOMessage::init(SimpleMessage& msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->writeSingleIO_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload write single IO data");
  }
  return rtn;
}

void WriteSingleIOMessage::init(WriteSingleIO& writeSingleIO)
{
  this->init();
  this->writeSingleIO_.copyFrom(writeSingleIO);
}

void WriteSingleIOMessage::init()
{
  this->setMessageType(1621);  // TODO: make enum for Stäubli specific standard port numbers
  this->writeSingleIO_.init();
}

bool WriteSingleIOMessage::load(ByteArray* buffer)
{
  bool rtn;
  LOG_COMM("Executing write single IO message load");

  if (buffer->load(this->writeSingleIO_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load write single IO data");
  }
  return rtn;
}

bool WriteSingleIOMessage::unload(ByteArray* buffer)
{
  bool rtn;
  LOG_COMM("Executing write single IO message unload");

  if (buffer->unload(this->writeSingleIO_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload write single IO data");
  }
  return rtn;
}

}  // namespace simple_message
}  // namespace staubli
