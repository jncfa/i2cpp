#ifndef __I2C_HANDLER_HPP__
#define __I2C_HANDLER_HPP__
#include "ros2_i2ccpp/constants.hpp"
#include <type_traits>
#pragma once

#include <cstdint>
#include <mutex>
#include <memory>
#include <string>

namespace ros2_i2ccpp
{

class I2CHandlerImpl;

template<typename Mutex>
class I2CHandler{
  friend class I2CTransactionBuilderImpl;

  I2CHandler() = delete;
  I2CHandler(uint16_t i2c_addr, std::string i2c_adapter_path = "/dev/i2c-1");
  I2CHandler(std::string i2c_adapter_path);

  /**
    * Get I2C adapter functionality, use I2C_FUNC_* constants to check what functionality is supported.
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

  /**
   * Read data starting at a given offset.
   */
  template<typename PODType>
  PODType read_data(uint16_t address, uint16_t offset) const;

  /**
   * Write data starting at a given offset.
   */
  template<typename PODType>
  void write_data(uint16_t address, uint16_t offset, const PODType & data) const;

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

template class I2CHandler<std::mutex>;
template class I2CHandler<null_mutex>;

using ThreadSafeI2CHandler = I2CHandler<std::mutex>;
using ThreadUnsafeI2CHandler = I2CHandler<null_mutex>;

} // namespace ros2_i2ccpp
#endif // __I2C_HANDLER_HPP__
