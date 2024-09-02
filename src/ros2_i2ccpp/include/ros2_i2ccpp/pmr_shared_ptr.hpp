#ifndef ROS2_I2CCPP_PMR_UNIQUE_PTR_HPP_
#define ROS2_I2CCPP_PMR_UNIQUE_PTR_HPP_
#pragma once

#include <memory>
#include <memory_resource>
#include <utility>

template<typename T, typename ... Args>
auto
make_shared_pmr(std::pmr::memory_resource & mr, Args &&... args)
{
  return std::allocate_shared<T>(std::pmr::polymorphic_allocator<T>(&mr),
    std::forward<Args>(args)...);
}
#endif // ROS2_I2CCPP_PMR_UNIQUE_PTR_HPP_
