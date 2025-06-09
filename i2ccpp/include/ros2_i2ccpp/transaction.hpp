// Copyright (c) 2024 jncfa
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef ROS2_I2CCPP_TRANSACTION_HPP_
#define ROS2_I2CCPP_TRANSACTION_HPP_
#pragma once

#include <cstdint>
#include <type_traits>
#include <utility>
#include <memory_resource>
#include <memory>
#include <array>

#include "ros2_i2ccpp/constants.hpp"
#include "ros2_i2ccpp/exceptions.hpp"
#include "ros2_i2ccpp/pmr_shared_ptr.hpp"
#include "ros2_i2ccpp/bit_cast.hpp"
class i2c_msg;

namespace ros2_i2ccpp
{


class I2CTransactionSegment{
public:
  explicit I2CTransactionSegment(uint16_t address_)
  : address(address_) {}
  virtual ~I2CTransactionSegment() = default;

  template<typename ...MessageFlagsT>
  I2CTransactionSegment & append_flags(const MessageFlagsT... flags)
  {
    if constexpr (sizeof...(flags) > 0) {
      message_flags |= (... | flags);
    }
    return *this;
  }

  virtual uint8_t * get_data() = 0;
  virtual uint16_t get_data_size() const = 0;

  uint16_t get_address() const {return address;}
  uint16_t get_message_flags() const {return message_flags;}

private:
  uint16_t address;
  uint16_t message_flags;
};

template<typename PODType>
class I2CReadTransactionSegment : public I2CTransactionSegment {
  static_assert(std::is_standard_layout_v<PODType>, "Type data must be of a POD type");

public:
  template<typename ...MessageFlagsT>
  explicit I2CReadTransactionSegment(
    uint16_t address_, PODType & data_,
    MessageFlagsT... flags)
  : I2CTransactionSegment(address_), data(data_),
    buffer(bit_cast<std::array<uint8_t, sizeof(PODType)>>(data))
  {
    append_flags(I2CMessageFlags::M_RD, std::forward<MessageFlagsT>(flags)...);
  }

  ~I2CReadTransactionSegment()
  {
    data = bit_cast<PODType>(buffer);
  }

  uint8_t * get_data() final
  {
    return buffer.data();
  }

  uint16_t get_data_size() const final
  {
    return buffer.size();
  }

private:
  // to avoid UB, we make a buffer to access the data as uint8_t*
  // and when the segment is destroyed, we copy everything to data
  PODType & data;
  std::array<uint8_t, sizeof(PODType)> buffer;
};

template<typename PODType>
class I2CWriteTransactionSegment : public I2CTransactionSegment {
  static_assert(std::is_standard_layout_v<PODType>&& std::is_trivially_copyable_v<PODType>,
      "Type data must be of a POD type and trivially copyable");

public:
  template<typename ...MessageFlagsT>
  explicit I2CWriteTransactionSegment(
    uint16_t address_, const PODType data_,
    MessageFlagsT... flags)
  : I2CTransactionSegment(address_), buffer(bit_cast<std::array<uint8_t, sizeof(PODType)>>(data_))
  {
    append_flags(flags ...);
  }

  uint8_t * get_data() final
  {
    return buffer.data();
  }

  uint16_t get_data_size() const final
  {
    return buffer.size();
  }

private:
  std::array<uint8_t, sizeof(PODType)> buffer;
};

class I2CTransaction{
public:
  // need to fulfill rule of 5

  explicit I2CTransaction(
    std::pmr::memory_resource & mr,
    std::pmr::vector<std::shared_ptr<I2CTransactionSegment>> && messages)
  :mem_resource(mr), transaction_segments(std::move(messages)) {}

  ~I2CTransaction() = default;

  I2CTransaction(I2CTransaction && other) noexcept
  : mem_resource(other.mem_resource),
    transaction_segments(std::exchange(other.transaction_segments, {})) {}

  I2CTransaction & operator=(I2CTransaction && other) noexcept
  {
    transaction_segments = std::exchange(other.transaction_segments, {});
    mem_resource = other.mem_resource;

    return *this;
  }

  I2CTransaction(const I2CTransaction &) = delete;
  I2CTransaction & operator=(const I2CTransaction &) = delete;

  std::pmr::memory_resource & getMemoryResource() {return mem_resource;}
  std::pmr::vector<std::shared_ptr<I2CTransactionSegment>> & getSegments()
  {
    return transaction_segments;
  }

private:
  // memory resource that will be used for all memory allocations
  std::reference_wrapper<std::pmr::memory_resource> mem_resource;

  // list of transaction segments
  std::pmr::vector<std::shared_ptr<I2CTransactionSegment>> transaction_segments;
};

/**
 * Builds a entire transaction.
 */
class I2CTransactionBuilderImpl {
public:
  explicit I2CTransactionBuilderImpl(std::pmr::memory_resource & mr, uint16_t device_address_)
  : device_address(device_address_), transaction_segments(&mr), mem_resource(mr) {}

  template<typename PODType, typename ...MessageFlagsT>
  I2CTransactionBuilderImpl & add_write(const PODType & pod, MessageFlagsT... flags)
  {
    static_assert(std::conjunction_v<std::is_same<I2CMessageFlags, MessageFlagsT>...>,
        "These should be all I2CMessageFlags!");
    return add_write_impl(pod, std::forward<MessageFlagsT>(flags)...);
  }

  template<typename PODType, typename ...MessageFlagsT>
  I2CTransactionBuilderImpl & add_write(
    uint16_t offset, const PODType & pod,
    MessageFlagsT... flags)
  {
    if (current_offset != offset) {
      // add an extra write to change the current offset
      add_write_impl(offset);
      current_offset = offset;
    }

    return add_write_impl(pod, std::forward<MessageFlagsT>(flags)...);
  }

  template<typename PODType, typename ...MessageFlagsT>
  I2CTransactionBuilderImpl & add_read(PODType & pod, MessageFlagsT... flags)
  {
    static_assert(std::conjunction_v<std::is_same<I2CMessageFlags, MessageFlagsT>...>,
        "These should be all I2CMessageFlags!");
    return add_read_impl(pod, std::forward<MessageFlagsT>(flags)...);
  }

  template<typename PODType, typename ...MessageFlagsT>
  I2CTransactionBuilderImpl & add_read(uint16_t offset, PODType & pod, MessageFlagsT... flags)
  {
    static_assert(std::conjunction_v<std::is_same<I2CMessageFlags, MessageFlagsT>...>,
        "These should be all I2CMessageFlags!");

    if (current_offset != offset) {
      // add an extra write to change the current offset
      add_write_impl(offset);
      current_offset = offset;

      // do not send start for this case
      return add_read_impl(pod, I2CMessageFlags::M_NOSTART, std::forward<MessageFlagsT>(flags)...);
    }

    return add_read_impl(pod, std::forward<MessageFlagsT>(flags)...);
  }

  I2CTransaction getTransaction();

private:
  template<typename PODType, typename ...MessageFlagsT>
  I2CTransactionBuilderImpl & add_write_impl(const PODType & pod, MessageFlagsT... flags)
  {
    static_assert(std::conjunction_v<std::is_same<I2CMessageFlags, MessageFlagsT>...>,
        "These should be all I2CMessageFlags!");
    emplace_transaction<I2CWriteTransactionSegment<PODType>>(device_address,
        pod, flags ...);

    return *this;
  }

  template<typename PODType, typename ...MessageFlagsT>
  I2CTransactionBuilderImpl & add_read_impl(PODType & pod, MessageFlagsT... flags)
  {
    static_assert(std::conjunction_v<std::is_same<I2CMessageFlags, MessageFlagsT>...>,
        "These should be all I2CMessageFlags!");
    emplace_transaction<I2CReadTransactionSegment<PODType>>(device_address,
        pod, std::forward<MessageFlagsT>(flags)...);
    return *this;
  }

  template<typename TransactionType, typename ...ArgsT>
  void emplace_transaction(ArgsT &&... args)
  {
    if(transaction_segments.size() + 1 >= I2CConstants::I2C_TRANSACTION_IOCTL_MAX_MSGS) {
      throw exceptions::IllegalOperationException(
          "Unable to push any more messages to the transaction queue!");
    }

    transaction_segments.push_back(make_shared_pmr<TransactionType>(mem_resource,
        std::forward<ArgsT>(args)...));
  }

  // i2c slave address that the transaction will apply to
  uint16_t device_address{0};

  // current offset
  uint16_t current_offset{0};

  // list of transaction segments
  std::pmr::vector<std::shared_ptr<I2CTransactionSegment>> transaction_segments;

  // memory resource that will be used for all memory allocations
  std::pmr::memory_resource & mem_resource;
};

/**
 * Single shot transaction builder.
 */
template<uint32_t array_size = 10'000>
class I2CTransactionBuilderSingleShot : public I2CTransactionBuilderImpl {
public:
  explicit I2CTransactionBuilderSingleShot(uint16_t device_address_)
  : I2CTransactionBuilderImpl(mbr, device_address_) {}

private:
  std::array<uint8_t, array_size> buffer{};
  std::pmr::monotonic_buffer_resource mbr{buffer.data(), buffer.size()};
};

template<uint32_t array_size = 10'000>
class I2CTransactionBuilderPooled : public I2CTransactionBuilderImpl {
public:
  explicit I2CTransactionBuilderPooled(
    uint16_t device_address_,
    std::pmr::pool_options options = {})
  : pool_mr(options, &mbr), I2CTransactionBuilderImpl(pool_mr, device_address_) {}

private:
  std::array<uint8_t, array_size> buffer{};
  std::pmr::monotonic_buffer_resource mbr{buffer.data(), buffer.size()};
  std::pmr::unsynchronized_pool_resource pool_mr;
};

/**
 * Transaction builder using the default allocator.
 */
class I2CTransactionBuilder : public I2CTransactionBuilderImpl {
public:
  explicit I2CTransactionBuilder(uint16_t device_address_)
  : I2CTransactionBuilderImpl(*std::pmr::get_default_resource(), device_address_) {}
};
} // namespace ros2_i2ccpp
#endif // ROS2_I2CCPP_TRANSACTION_HPP_
