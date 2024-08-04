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
#include "ros2_i2ccpp/pmr_unique_ptr.hpp"

template<typename Mutex>
class I2CHandler;
class I2CHandlerImpl;

namespace ros2_i2ccpp
{
class I2CTransactionSegment{
public:
  I2CTransactionSegment(uint16_t address_)
  : address(address_) {}
  virtual ~I2CTransactionSegment() = default;

  template<typename ...MessageFlagsT>
  I2CTransactionSegment & append_flags(const MessageFlagsT... flags)
  {
    static_assert(sizeof...(flags) > 0, "append_flags() must have at least one argument!");

    message_flags |= (... | flags);
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
  I2CReadTransactionSegment(
    uint16_t address_, PODType & data_,
    MessageFlagsT... flags)
  : data(data_), I2CTransactionSegment(address_)
  {
    append_flags(I2CMessageFlags::M_RD, std::forward<MessageFlagsT>(flags)...);
  }

private:
  uint8_t * get_data() final
  {
    return std::addressof(data);
  }

  uint16_t get_data_size() const final
  {
    return sizeof(PODType);
  }

  PODType & data;
};

template<typename PODType>
class I2CWriteTransactionSegment : public I2CTransactionSegment {
  static_assert(std::is_standard_layout_v<PODType>&& std::is_trivially_copyable_v<PODType>,
      "Type data must be of a POD type and trivially copyable");

public:
  template<typename ...MessageFlagsT>
  I2CWriteTransactionSegment(
    uint16_t address_, const PODType & data_,
    MessageFlagsT... flags)
  : data(data_), I2CTransactionSegment(address_)
  {
    append_flags(flags ...);
  }

private:
  uint8_t * get_data() final
  {
    return std::addressof(&data);
  }

  uint16_t get_data_size() const final
  {
    return sizeof(PODType);
  }

  PODType data;
};

/**
 * Builds a entire transaction, and executes it with a I2CHandler.
 */
class I2CTransactionBuilderImpl {
public:
  I2CTransactionBuilderImpl(std::pmr::memory_resource & mr, uint16_t device_address_)
  : device_address(device_address_), transaction_segments(&mr), mem_resource(mr) {}

  template<typename Mutex>
  void apply_transaction(const I2CHandler<Mutex> & handler);
  void apply_transaction(I2CHandlerImpl & handler);

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
    }

    return add_read_impl(pod, std::forward<MessageFlagsT>(flags)...);
  }

private:
  template<typename PODType, typename ...MessageFlagsT>
  I2CTransactionBuilderImpl & add_write_impl(const PODType & pod, MessageFlagsT... flags)
  {
    static_assert(std::conjunction_v<std::is_same<I2CMessageFlags, MessageFlagsT>...>,
        "These should be all I2CMessageFlags!");
    emplace_transaction<I2CWriteTransactionSegment<PODType>>(mem_resource, device_address,
        pod, flags ...);

    return *this;
  }

  template<typename PODType, typename ...MessageFlagsT>
  I2CTransactionBuilderImpl & add_read_impl(PODType & pod, MessageFlagsT... flags)
  {
    static_assert(std::conjunction_v<std::is_same<I2CMessageFlags, MessageFlagsT>...>,
        "These should be all I2CMessageFlags!");
    emplace_transaction<I2CReadTransactionSegment<PODType>>(mem_resource, device_address,
        pod, std::forward<MessageFlagsT>(flags)...);
    return *this;
  }

  template<typename TransactionType, typename ...ArgsT>
  void emplace_transaction(ArgsT &&... args)
  {
    if(transaction_segments.size() + 1 >= _I2C_RDWR_IOCTL_MAX_MSGS) {
      throw exceptions::IllegalOperationException(
          "Unable to push any more messages to the transaction queue!");
    }

    transaction_segments.push_back(make_unique_pmr<TransactionType>(mem_resource,
        std::forward<ArgsT>(args)...));
  }

  // i2c slave address that the transaction will apply to
  uint16_t device_address{0};

  // current offset
  uint16_t current_offset{0};

  // list of transaction segments
  std::pmr::vector<unique_ptr_pmr<I2CTransactionSegment>> transaction_segments;

  // memory resource that will be used for all memory allocations
  std::pmr::memory_resource & mem_resource;
};

/**
 * Transaction builder with its own 1KB array so you dont need to worry about keeping your own array.
 * If you need a larger array, then consider building your own.
 */
class I2CTransactionBuilderImplBuffed : public I2CTransactionBuilderImpl {
public:
  I2CTransactionBuilderImplBuffed(uint16_t device_address_)
  : I2CTransactionBuilderImpl(mbr, device_address_) {}

private:
  std::array<uint8_t, 1'000> buffer{};
  std::pmr::monotonic_buffer_resource mbr{buffer.data(), buffer.size()};
};


class I2CTransactionBuilder{
public:
  template<typename ... I2CMessageFlagsT>
  I2CTransactionBuilder & addWrite(uint32_t address, I2CMessageFlagsT... message_flags);

  template<typename ... I2CMessageFlagsT>
  I2CTransactionBuilder & addRead(uint32_t address, I2CMessageFlagsT... message_flags);

private:
  I2CTransactionBuilder & addWrite(uint32_t address, uint64_t message_flags);
};
}
#endif // ROS2_I2CCPP_TRANSACTION_HPP_
