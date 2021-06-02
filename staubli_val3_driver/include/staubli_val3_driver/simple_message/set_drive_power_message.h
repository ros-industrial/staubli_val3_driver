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

#ifndef SET_DRIVE_POWER_MESSAGE_H
#define SET_DRIVE_POWER_MESSAGE_H

#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"

namespace staubli
{
namespace simple_message
{

/**
 * \brief Message representing the industrial_msgs/SetDrivePower service
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class SetDrivePowerMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  SetDrivePowerMessage();
  /**
   * \brief Destructor
   *
   */
  ~SetDrivePowerMessage();
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage& msg);

  /**
   * \brief Initializes message
   *
   * \param drive power value to initialize from
   *
   */
  void init(industrial::shared_types::shared_bool drive_power);

  /**
   * \brief Initializes a new set drive power message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);

  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_bool);
  }

  industrial::shared_types::shared_bool drive_power_;
};

}  // namespace simple_message
}  // namespace staubli

#endif  // SET_DRIVE_POWER_MESSAGE_H
