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

#ifndef READ_IO_MESSAGE_H
#define READ_IO_MESSAGE_H

#include "staubli_val3_driver/simple_message/io_states.h"

#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"

namespace staubli
{
namespace simple_message
{

/**
 * \brief Class encapsulated read IO message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type).
 *
 * This message simply wraps the staubli::simple_message::IOStates data type.
 * The data portion of this typed message matches IOStates.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class ReadIOMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  ReadIOMessage();
  /**
   * \brief Destructor
   *
   */
  ~ReadIOMessage();
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage& msg);

  //  /**
  //   * \brief Initializes message from a IO states structure
  //   *
  //   * \param states structure to initialize from
  //   *
  //   */
  //  void init(IOStates & states);

  /**
   * \brief Initializes a new read IO message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);

  unsigned int byteLength()
  {
    return this->states_.byteLength();
  }

  IOStates states_;
};

}  // namespace simple_message
}  // namespace staubli

#endif  // READ_IO_MESSAGE_H
