// Copyright (c) 2024 jncfa
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef ROS2_I2CCPP_PMR_SHARED_PTR_HPP_
#define ROS2_I2CCPP_PMR_SHARED_PTR_HPP_
#pragma once

#include <memory>
#include <memory_resource>
#include <utility>

namespace ros2_i2ccpp
{

template<typename T, typename ... Args>
auto
make_shared_pmr(std::pmr::memory_resource & mr, Args &&... args)
{
  return std::allocate_shared<T>(std::pmr::polymorphic_allocator<T>(&mr),
    std::forward<Args>(args)...);
}

} // namespace ros2_i2ccpp

#endif // ROS2_I2CCPP_PMR_SHARED_PTR_HPP_
