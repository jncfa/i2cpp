// Copyright (c) 2024 jncfa
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

extern "C"
{
#include <fcntl.h>  //open
#include <unistd.h> // close
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
}

#include "ros2_i2ccpp/impl/i2c_handler_impl.hpp"
#include "ros2_i2ccpp/constants.hpp"
#include "ros2_i2ccpp/exceptions.hpp"

namespace ros2_i2ccpp
{

I2CHandlerImpl::I2CHandlerImpl(uint16_t i2c_addr, std::string i2c_adapter_path)
{
  // open handle to the i2c device immediately
  open(i2c_addr, i2c_adapter_path);
}
I2CHandlerImpl::I2CHandlerImpl(std::string i2c_adapter_path)
{
  open(i2c_adapter_path);
}
I2CHandlerImpl::~I2CHandlerImpl()
{
  close();
}

void I2CHandlerImpl::open(const std::string i2c_adapter_path)
{
  // if we already have a valid file descriptor, throw since we need to close the handle
  if (is_opened()) {
    throw exceptions::IllegalOperationException(
            "File descriptor was already obtained, make sure to close the previous descriptor before opening a new one");
  }

  // get and store fd to i2c adapter
  i2c_file_desc = ::open(i2c_adapter_path.c_str(), O_NONBLOCK | O_RDWR);
  if (i2c_file_desc < 0) {
    throw exceptions::SysException("Unable acquire file descriptor to device");
  }

  // query the adapter functionality for later use
  if (ioctl(i2c_file_desc, I2C_FUNCS, &adapter_func) < 0) {
    throw exceptions::SysException("Unable query adapter functionality");
  }
}

void I2CHandlerImpl::open(const uint16_t i2c_addr, const std::string i2c_adapter_path)
{
  open(i2c_adapter_path);
  set_i2c_device(i2c_addr);
}

void I2CHandlerImpl::close()
{
  if (is_opened()) {
    if (::close(i2c_file_desc) < 0) {
      throw exceptions::SysException("Unable close file descriptor");
    }

    // reset file descriptor and adapter functionality
    i2c_file_desc = INVALID_FILE_DESC;
    adapter_func = 0;

    // as well as the cached i2c address
    cached_i2c_addr = INVALID_I2C_ADDR;
  }
}

void I2CHandlerImpl::set_i2c_device(const uint16_t i2c_addr)
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  // check if we are already using this device
  if (cached_i2c_addr != i2c_addr) {
    // check if the i2c address needs 10-bit enabled
    set_ten_bit_if_needed(i2c_addr);

    // choose which i2c device we want to use for all operations
    if (ioctl(i2c_file_desc, I2CIOControlCommands::SLAVE, i2c_addr) < 0) {
      throw exceptions::SysException("Unable acquire file descriptor to device");
    }
    cached_i2c_addr = i2c_addr;
  }
}

template<typename Alloc>
void I2CHandlerImpl::process_i2c_transaction(std::vector<i2c_msg, Alloc> & messages) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_I2C)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }

  if (messages.size() > I2C_RDWR_IOCTL_MAX_MSGS) {
    throw exceptions::IllegalOperationException(
            "I2C does not support this many messages in a single transaction");
  }
  // initialize transaction block and messages
  i2c_rdwr_ioctl_data transaction_block{messages.data(), static_cast<uint32_t>(messages.size())};

  // apply transaction
  if (ioctl(i2c_file_desc, I2CIOControlCommands::RDWR, &transaction_block) < 0) {
    throw exceptions::SysException("Error executing ioctl request");
  }
}

template void I2CHandlerImpl::process_i2c_transaction(std::pmr::vector<i2c_msg> & messages) const;

void I2CHandlerImpl::write_quick(const uint8_t value) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_QUICK)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }

  if (i2c_smbus_write_quick(i2c_file_desc, value) < 0) {
    throw exceptions::SysException("Unable to write quick");
  }
}

uint8_t I2CHandlerImpl::read_byte() const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_READ_BYTE)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }

  const auto result = i2c_smbus_read_byte(i2c_file_desc);
  if (result < 0) {
    throw exceptions::SysException("Unable to execute request");
  }

  return static_cast<uint8_t>(result);
}

void I2CHandlerImpl::write_byte(const uint8_t value) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_WRITE_BYTE)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation");
  }

  if (i2c_smbus_write_byte(i2c_file_desc, value) < 0) {
    throw exceptions::SysException("Unable to execute request");
  }
}

uint8_t I2CHandlerImpl::read_byte_at(const uint16_t register_addr) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_READ_BYTE_DATA)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }
  const auto result = i2c_smbus_read_byte_data(i2c_file_desc, register_addr);
  if (result < 0) {
    throw exceptions::SysException("Unable to execute request");
  }

  return static_cast<uint8_t>(result);
}

void I2CHandlerImpl::write_byte_at(const uint16_t register_addr, const uint8_t value) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_WRITE_BYTE_DATA)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }

  if (i2c_smbus_write_byte_data(i2c_file_desc, register_addr, value) < 0) {
    throw exceptions::SysException("Unable to execute request");
  }
}

uint16_t I2CHandlerImpl::read_word(const uint16_t register_addr) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_READ_WORD_DATA)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }
  const auto result = i2c_smbus_read_word_data(i2c_file_desc, register_addr);

  if (result < 0) {
    throw exceptions::SysException("Unable to execute request");
  }

  return static_cast<uint16_t>(result);
}

void I2CHandlerImpl::write_word(const uint16_t register_addr, const uint16_t value) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_WRITE_WORD_DATA)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }

  if (i2c_smbus_write_word_data(i2c_file_desc, register_addr, value) < 0) {
    throw exceptions::SysException("Unable to execute request");
  }
}

uint16_t I2CHandlerImpl::process_call(const uint16_t register_addr, const uint16_t value) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_PROC_CALL)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }

  const auto result = i2c_smbus_process_call(i2c_file_desc, register_addr, value);
  if (result < 0) {
    throw exceptions::SysException("Unable to execute request");
  }
  return static_cast<uint16_t>(result);
}

uint16_t I2CHandlerImpl::block_process_call(
  const uint16_t register_addr,
  std::vector<uint8_t> & data) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_BLOCK_PROC_CALL)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }

  const auto result = i2c_smbus_block_process_call(
    i2c_file_desc, register_addr,
    data.size(), data.data());
  if (result < 0) {
    throw exceptions::SysException("Unable to execute request");
  }
  return static_cast<uint16_t>(result);
}

std::vector<uint8_t> I2CHandlerImpl::read_block_data(const uint8_t register_addr) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_READ_BLOCK_DATA)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }

  std::vector<uint8_t> data{I2C_SMBUS_BLOCK_MAX};

  const auto result = i2c_smbus_read_block_data(i2c_file_desc, register_addr, data.data());
  if (result < 0) {
    throw exceptions::SysException("Unable to execute request");
  }
  // resize vector and return it
  data.resize(result);
  return data;
}

void I2CHandlerImpl::write_block_data(
  const uint8_t register_addr,
  const std::vector<uint8_t> & data) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_WRITE_BLOCK_DATA)) {
    throw exceptions::IllegalOperationException("Adapter does not support this operation!");
  }

  const auto result = i2c_smbus_write_block_data(
    i2c_file_desc, register_addr,
    data.size(), data.data());
  if (result < 0) {
    throw exceptions::SysException("Unable to execute request");
  }
}

void I2CHandlerImpl::set_ten_bit(bool enable) const
{
  // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  // check if the adapter supports 10-bit addressing
  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_10BIT_ADDR)) {
    throw exceptions::IllegalOperationException("Adapter does not support 10-bit addressing");
  }

  if (ioctl(i2c_file_desc, I2CIOControlCommands::TENBIT, enable ? 1 : 0) < 0) {
    throw exceptions::SysException("Error setting 10-bit addressing");
  }
}

void I2CHandlerImpl::set_pec(bool enable)
{
    // ensure we have a valid file descriptor
  if (!is_opened()) {
    throw exceptions::IllegalOperationException("File descriptor is invalid");
  }

  // check if the adapter supports PEC addressing
  if (!has_functionality(I2CControllerFunctionalityFlags::FUNC_SMBUS_PEC)) {
    throw exceptions::IllegalOperationException("Adapter does not support PEC");
  }

  if (ioctl(i2c_file_desc, I2CIOControlCommands::PEC, enable ? 1 : 0) < 0) {
    throw exceptions::SysException("Error setting PEC");
  }
}

} // namespace ros2_i2ccpp
