#ifndef __I2C_HANDLER_IMPL_HPP__
#define __I2C_HANDLER_IMPL_HPP__
#include <memory>
#pragma once

extern "C"
{
#include <linux/i2c.h>
}

#include <cstdint>
#include <vector>
#include <string>
#include <bit>

#include "ros2_i2ccpp/transaction.hpp"

namespace ros2_i2ccpp
{

class I2CHandlerImpl{
public:
  I2CHandlerImpl(uint16_t i2c_addr, std::string i2c_adapter_path = "/dev/i2c-1");
  I2CHandlerImpl(std::string i2c_adapter_path);
  ~I2CHandlerImpl();

  [[nodiscard]] inline bool is_opened() const {return i2c_file_desc != INVALID_FILE_DESC;}
  [[nodiscard]] inline uint64_t get_adapter_func() const {return adapter_func;}
  [[nodiscard]] inline uint64_t get_current_device_addr() const {return cached_i2c_addr;}
  [[nodiscard]] inline bool has_functionality(uint64_t flag) const
  {
    return (adapter_func & flag) > 0;
  }

  void write_quick(uint8_t value) const;
  inline void write_quick(uint16_t i2c_addr, uint8_t value)
  {
    set_i2c_device(i2c_addr);
    write_quick(value);
  }


  [[nodiscard]] std::byte read_byte() const;
  [[nodiscard]] inline std::byte read_byte(uint16_t i2c_addr)
  {
    set_i2c_device(i2c_addr);
    return read_byte();
  }

  void write_byte(std::byte value) const;
  inline void write_byte(uint16_t i2c_addr, std::byte value)
  {
    set_i2c_device(i2c_addr);
    write_byte(value);
  }

  [[nodiscard]] std::byte read_byte_at(uint16_t register_addr) const;
  [[nodiscard]] inline std::byte read_byte_at(uint16_t i2c_addr, uint16_t register_addr)
  {
    set_i2c_device(i2c_addr);
    return read_byte_at(register_addr);
  }

  void write_byte_at(uint16_t register_addr, std::byte value) const;
  inline void write_byte_at(uint16_t i2c_addr, uint16_t register_addr, std::byte value)
  {
    set_i2c_device(i2c_addr);
    write_byte_at(register_addr, value);
  }

  [[nodiscard]] uint16_t read_word(uint16_t register_addr) const;
  [[nodiscard]] inline uint16_t read_word(uint16_t i2c_addr, uint16_t register_addr)
  {
    set_i2c_device(i2c_addr);
    return read_word(register_addr);
  }

  void write_word(uint16_t register_addr, uint16_t value) const;
  inline void write_word(uint16_t i2c_addr, uint16_t register_addr, uint16_t value)
  {
    set_i2c_device(i2c_addr);
    write_word(register_addr, value);
  }

  [[nodiscard]] uint16_t process_call(uint16_t register_addr, uint16_t value) const;
  [[nodiscard]] inline uint16_t process_call(
    uint16_t i2c_addr, uint16_t register_addr,
    uint16_t value)
  {
    set_i2c_device(i2c_addr);
    return process_call(register_addr, value);
  }

  [[nodiscard]] uint16_t block_process_call(
    uint16_t register_addr,
    std::vector<std::byte> & value) const;

  [[nodiscard]] inline uint16_t block_process_call(
    uint16_t i2c_addr, uint16_t register_addr,
    std::vector<std::byte> & value)
  {
    set_i2c_device(i2c_addr);
    return block_process_call(register_addr, value);
  }

  [[nodiscard]] std::vector<std::byte> read_block_data(uint8_t register_addr) const;
  [[nodiscard]] inline std::vector<std::byte> read_block_data(
    uint16_t i2c_addr,
    uint8_t register_addr)
  {
    set_i2c_device(i2c_addr);
    return read_block_data(register_addr);
  }

  /**
   * Send block data using the SMBus protocol.
   */
  void write_block_data(uint8_t register_addr, const std::vector<std::byte> & data) const;
  inline void write_block_data(
    uint16_t i2c_addr, uint8_t register_addr,
    const std::vector<std::byte> & data)
  {
    set_i2c_device(i2c_addr);
    write_block_data(register_addr, data);
  }

  template<typename PODType>
  void write_pod_data(uint16_t register_addr, const PODType & data)
  {
    I2CTransactionBuilderImplBuffed builder{cached_i2c_addr};

    builder.add_write(register_addr, data);
    builder.apply_transaction(this);
  }

  template<typename PODType>
  void write_pod_data(uint16_t i2c_addr, uint16_t register_addr, const PODType & data)
  {
    set_i2c_device(i2c_addr);
    write_pod_data(register_addr, data);
  }

  template<typename PODType>
  PODType read_pod_data(uint16_t register_addr) const
  {

  }

  template<typename PODType>
  PODType read_pod_data(uint16_t i2c_addr, uint16_t register_addr)
  {
    set_i2c_device(i2c_addr);
    return read_pod_data<PODType>(register_addr);
  }

  /**
   * Execute a given I2C transaction.
   */
  template<typename Alloc = std::allocator<i2c_msg>>
  void process_i2c_transaction(std::vector<i2c_msg, Alloc> & messages) const;

  /**
   * Set Packet Error Checking (PEC).
   */
  void set_pec(bool enable);

private:
  /**
    * Initialize communication with I2C adapter.
    * Does not set the device address.
    */
  void open(std::string i2c_adapter_path = "/dev/i2c-1");

  /**
    * Initialize communication with I2C adapter.
    * Sets the device address for future operations.
    */
  void open(uint16_t i2c_addr, std::string i2c_adapter_path = "/dev/i2c-1");

  /**
    * Set the device address for future operations.
    */
  void set_i2c_device(uint16_t i2c_addr);

  /**
    * Close communication with I2C adapter.
    */
  void close();

  /**
    * Set ten bit functionality.
    */
  void set_ten_bit(bool enable) const;
  inline void set_ten_bit_if_needed(uint16_t i2c_addr) const
  {
    // check if the i2c address needs 10-bit enabled (>7-bit)
    if (i2c_addr >= 0b1'000'000) {
      set_ten_bit(true);
    }
  }

  static constexpr int32_t INVALID_FILE_DESC = -1;
  static constexpr uint16_t INVALID_I2C_ADDR = 0xFFFF;

  // file descriptor to the i2c adapter/controller
  int32_t i2c_file_desc{INVALID_FILE_DESC};

  // address of the current device we are managing
  uint16_t cached_i2c_addr{INVALID_I2C_ADDR};

  // functionality of the i2c adapter
  uint64_t adapter_func{0};
};

}

#endif // __I2C_HANDLER_IMPL_HPP__
