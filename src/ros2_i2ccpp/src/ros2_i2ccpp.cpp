// Copyright (c) 2024 jncfa
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "ros2_i2ccpp/ros2_i2ccpp.hpp"
#include "ros2_i2ccpp/exceptions.hpp"
#include "ros2_i2ccpp/pmr_shared_ptr.hpp"
#include "ros2_i2ccpp/constants.hpp"
#include "ros2_i2ccpp/transaction.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <memory>
#include <memory_resource>
#include <mutex>
#include <sys/types.h>
#include <type_traits>
#include <vector>
#include <optional>
#include <execution>

extern "C"
{
#include <fcntl.h>  //open
#include <unistd.h> // close
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
}
