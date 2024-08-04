// Copyright (c) 2024 jncfa
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef ROS2_I2CCPP__EXCEPTIONS_HPP_
#define ROS2_I2CCPP__EXCEPTIONS_HPP_

#include <stdexcept>
#include <errno.h>
#include <system_error>

namespace ros2_i2ccpp::exceptions
{

/**
 * An exception for unexpected errors from a syscall or other miscellaneous library calls.
 * The exception also fetches the error code from errno by default, but can pass the error code from the function if given.
 */
class SysException : public std::system_error
{
public:
  using system_error::system_error;

  SysException(std::string what)
  : system_error(error_code_from_errno(errno), what)
  {
  }

  SysException(std::string what, int error_code)
  : system_error(error_code_from_errno(error_code), what) {}

  static std::error_code error_code_from_errno(int errno_code)
  {
    return std::make_error_code(static_cast<std::errc>(errno_code));
  }
};

/**
 * An exception when we attempt to perform an action that is forbidden in the current state.
 * This could be trying to initialize something that has been initialized or terminate something that has been already terminated.
 */
class IllegalOperationException : public std::runtime_error
{
  using runtime_error::runtime_error;
};

} // namespace ros2_i2ccpp::exceptions

#endif //ROS2_I2CCPP__EXCEPTIONS_HPP_
