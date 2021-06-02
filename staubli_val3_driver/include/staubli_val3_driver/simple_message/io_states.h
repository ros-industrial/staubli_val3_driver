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

#ifndef IO_STATES_H
#define IO_STATES_H

#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"

namespace staubli
{
namespace simple_message
{

class IOStates : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  IOStates();

  /**
   * \brief Destructor
   *
   */
  ~IOStates();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray* buffer);
  bool unload(industrial::byte_array::ByteArray* buffer);
  unsigned int byteLength()
  {
    return 6 * sizeof(industrial::shared_types::shared_int) + 2 * sizeof(industrial::shared_types::shared_bool);
  }

private:
  /**
   * \brief UserIO inputs
   */
  industrial::shared_types::shared_int user_in_;

  /**
   * \brief UserIO outputs (valve)
   */
  industrial::shared_types::shared_int valve_out_;

  /**
   * \brief BasicIO inputs
   */
  industrial::shared_types::shared_int basic_in_;

  /**
   * \brief BasicIO outputs
   */
  industrial::shared_types::shared_int basic_out_;

  /**
   * \brief BasicIO-2 inputs (2nd IO module)
   */
  industrial::shared_types::shared_int basic_in_2_;

  /**
   * \brief BasicIO-2 outputs (2nd IO module)
   */
  industrial::shared_types::shared_int basic_out_2_;

  /**
   * \brief Flag indicating if BasicIO module 1 is working
   */
  industrial::shared_types::shared_bool basic_io_valid_;

  /**
   * \brief Flag indicating if BasicIO module 2 is working
   */
  industrial::shared_types::shared_bool basic_io_2_valid_;
};

}  // namespace simple_message
}  // namespace staubli

#endif  // IO_STATES_H
