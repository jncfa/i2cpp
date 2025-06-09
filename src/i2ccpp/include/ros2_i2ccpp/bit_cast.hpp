// Copyright (c) 2024 jncfa
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once
#include <type_traits>
#include <cstring>

namespace ros2_i2ccpp
{

/**
 * Obtain a value of type To by reinterpreting the object representation of From.
 * Every bit in the value representation of the returned To object is equal to the
 * corresponding bit in the object representation of from.
 * The values of padding bits in the returned To object are unspecified.
 */
template<class To, class From>
[[nodiscard]] constexpr To bit_cast(From const & a) noexcept
{
  static_assert(!std::is_pointer_v<From>,
                "bit_cast must not be used on pointer types");
  static_assert(!std::is_pointer_v<To>,
                "bit_cast must not be used on pointer types");
  static_assert(!std::is_reference_v<From>,
                "bit_cast must not be used on reference types");
  static_assert(!std::is_reference_v<To>,
                "bit_cast must not be used on reference types");
  static_assert(
      sizeof(From) == sizeof(To),
      "bit_cast requires source and destination types to be the same size");
  static_assert(std::is_trivially_copyable_v<From>,
                "bit_cast requires the source type to be trivially copyable");
  static_assert(
      std::is_trivially_copyable_v<To>,
      "bit_cast requires the destination type to be trivially copyable");

  static_assert(
      std::is_trivially_constructible_v<To>,
      "bit_cast requires the destination type to be trivially constructible");

#if (__has_builtin(__builtin_bit_cast))
  return __builtin_bit_cast(To, a);
#else
  static_assert(sizeof(T) == sizeof(U));  //  At least two spaces is best between code and comments

  T result;
  std::memcpy(&result, &a, sizeof(T));
  return result;
#endif
}

} // namespace ros2_i2ccpp
