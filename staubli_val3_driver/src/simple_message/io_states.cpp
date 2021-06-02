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

#include "staubli_val3_driver/simple_message/io_states.h"

#include "simple_message/log_wrapper.h"
#include "simple_message/shared_types.h"

using namespace industrial::shared_types;

namespace staubli
{
namespace simple_message
{

IOStates::IOStates()
{
}

IOStates::~IOStates()
{
}

bool IOStates::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  LOG_COMM("Executing IO states load");

  if (buffer->load(this->user_in_) && buffer->load(this->valve_out_) && buffer->load(this->basic_in_) &&
      buffer->load(this->basic_out_) && buffer->load(this->basic_in_2_) && buffer->load(this->basic_out_2_) &&
      buffer->load(this->basic_io_valid_) && buffer->load(this->basic_io_2_valid_))
  {

    LOG_COMM("IO states successfully loaded");
    rtn = true;
  }
  else
  {
    LOG_COMM("IO states not loaded");
    rtn = false;
  }

  return rtn;
}

bool IOStates::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  LOG_COMM("Executing IO states unload");
  if (buffer->unload(this->basic_io_2_valid_) && buffer->unload(this->basic_io_valid_) &&
      buffer->unload(this->basic_out_2_) && buffer->unload(this->basic_in_2_) && buffer->unload(this->basic_out_) &&
      buffer->unload(this->basic_in_) && buffer->unload(this->valve_out_) && buffer->unload(this->user_in_))
  {

    rtn = true;
    LOG_COMM("IO states successfully unloaded");
  }

  else
  {
    LOG_ERROR("Failed to unload IO states");
    rtn = false;
  }

  return rtn;
}

}  // namespace simple_message
}  // namespace staubli
