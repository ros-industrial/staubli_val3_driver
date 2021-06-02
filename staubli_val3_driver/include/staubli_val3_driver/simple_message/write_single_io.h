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

#ifndef WRITE_SINGLE_IO_H
#define WRITE_SINGLE_IO_H

#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"

namespace staubli
{
namespace simple_message
{

/**
 * \brief Enumeration mirrors staubli_msgs/IOModule definition
 *
 */
namespace IOModules
{

enum IOModule
{
  UNKNOWN = -1,
  USER_IN = 1,
  VALVE_OUT = 2,
  BASIC_IN = 3,
  BASIC_OUT = 4,
  BASIC_IN_2 = 5,
  BASIC_OUT_2 = 6
};

}  // namespace IOModules
typedef IOModules::IOModule IOModule;

class WriteSingleIO : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  WriteSingleIO();

  /**
   * \brief Destructor
   *
   */
  ~WriteSingleIO();

  /**
   * \brief Initializes an empty WriteSingleIO structure
   *
   */
  void init();

  /**
   * \brief Initializes a full WriteSingleIO structure
   *
   */
  void init(IOModule moduleId, industrial::shared_types::shared_int pin, industrial::shared_types::shared_bool state);

  IOModule getModuleId()
  {
    return IOModule(module_id_);
  }

  industrial::shared_types::shared_int getPin() const
  {
    return pin_;
  }

  industrial::shared_types::shared_bool getState() const
  {
    return state_;
  }

  void setModuleId(industrial::shared_types::shared_int module_id)
  {
    this->module_id_ = module_id;
  }

  void setPin(industrial::shared_types::shared_int pin)
  {
    this->pin_ = pin;
  }

  void setState(industrial::shared_types::shared_bool state)
  {
    this->state_ = state;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(WriteSingleIO& src);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);
  unsigned int byteLength()
  {
    return 2 * sizeof(industrial::shared_types::shared_int) + sizeof(industrial::shared_types::shared_bool);
  }

private:
  /**
   * \brief Identifier of the IOModule (see staubli_msgs/IOModule)
   */
  industrial::shared_types::shared_int module_id_;

  /**
   * \brief Index of the pin
   */
  industrial::shared_types::shared_int pin_;

  /**
   * \brief Pin state
   */
  industrial::shared_types::shared_bool state_;
};

}  // namespace simple_message
}  // namespace staubli

#endif  // WRITE_SINGLE_IO_H
