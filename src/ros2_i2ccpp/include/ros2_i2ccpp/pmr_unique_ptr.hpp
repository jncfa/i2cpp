#ifndef ROS2_I2CCPP_PMR_UNIQUE_PTR_HPP_
#define ROS2_I2CCPP_PMR_UNIQUE_PTR_HPP_
#pragma once

#include <memory>
#include <memory_resource>
#include <utility>

template<typename Alloc>
struct alloc_deleter
{
  alloc_deleter(const Alloc & a)
  : a(a) {}

  typedef typename std::allocator_traits<Alloc>::pointer pointer;

  void operator()(pointer p) const
  {
    Alloc aa(a);
    std::allocator_traits<Alloc>::destroy(aa, std::addressof(*p));
    std::allocator_traits<Alloc>::deallocate(aa, p, 1);
  }

private:
  Alloc a;
};

template<typename T, typename Alloc, typename ... Args>
auto
allocate_unique(const Alloc & alloc, Args &&... args)
{
  using AT = std::allocator_traits<Alloc>;
  static_assert(std::is_same<typename AT::value_type, std::remove_cv_t<T>>{}(),
                "Allocator has the wrong value_type");

  Alloc a(alloc);
  auto p = AT::allocate(a, 1);
  try {
    AT::construct(a, std::addressof(*p), std::forward<Args>(args)...);
    using D = alloc_deleter<Alloc>;
    return std::unique_ptr<T, D>(p, D(a));
  } catch (...) {
    AT::deallocate(a, p, 1);
    throw;
  }
}

template<typename Tp>
using unique_ptr_pmr = std::unique_ptr<Tp, alloc_deleter<std::pmr::polymorphic_allocator<Tp>>>;


template<typename T, typename ... Args>
auto
make_unique_pmr(std::pmr::memory_resource & mr, Args &&... args)
{
  return allocate_unique<T>(std::pmr::polymorphic_allocator<T>(&mr), std::forward<Args>(args)...);
}
#endif // ROS2_I2CCPP_PMR_UNIQUE_PTR_HPP_
