// Copyright (c) 2024 jncfa
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef __I2C_HANDLER_HPP__
#define __I2C_HANDLER_HPP__
#pragma once

#include <cstdint>
#include <mutex>
#include <memory>
#include <string>
#include <type_traits>

#include "ros2_i2ccpp/constants.hpp"
#include "ros2_i2ccpp/transaction.hpp"

namespace ros2_i2ccpp
{

class I2CHandlerImpl;

template<typename Mutex>
class I2CHandler{
  I2CHandler() = delete;
  explicit I2CHandler(uint16_t i2c_addr, std::string i2c_adapter_path = "/dev/i2c-1");
  explicit I2CHandler(std::string i2c_adapter_path);

  /**
    * Get I2C adapter functionality, use I2CControllerFunctionalityFlags constants to check what functionality is supported.
    */
  [[nodiscard]] uint64_t get_adapter_func() const;

  /**
    * Indicate the current I2C device address that is being managed.
    */
  [[nodiscard]] uint64_t get_current_device_addr() const;

  /**
    * Check if the adapter has the functionality indicated by that flag or combination of flags.
    */
  [[nodiscard]] bool has_functionality(uint64_t flag) const;

  /**
    * Check if the adapter has the functionality indicated by these flags
    */
  template<typename ...I2CFunctionalityFlagsT>
  [[nodiscard]] bool has_functionality(I2CFunctionalityFlagsT &&... flags) const
  {
    static_assert(std::conjunction_v<std::is_same<I2CFunctionalityFlagsT,
      I2CControllerFunctionalityFlags>...>&& (sizeof...(I2CFunctionalityFlagsT) > 0),
        "You must supply valid flags to use this function!");
    return has_functionality((... | flags));
  }

  void apply_transaction(I2CTransaction && transaction) const;

  /**
    * Set ten bit functionality.
    */
  void set_pec(bool enable);

private:
  mutable Mutex mut;
  std::unique_ptr<I2CHandlerImpl> handler;
};

struct null_mutex
{
  void lock() {}
  void unlock() {}
};

using ThreadSafeI2CHandler = I2CHandler<std::mutex>;
using ThreadUnsafeI2CHandler = I2CHandler<null_mutex>;

} // namespace ros2_i2ccpp
#endif // __I2C_HANDLER_HPP__
