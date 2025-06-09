#ifndef I2CPP_HPP
#define I2CPP_HPP

#include <cstdint>
#include "i2cpp/constants.hpp"
#include "i2cpp_export.h"
// this is a rewrite of the ros2_i2ccpp library

// how this should work
// 1. I2CHandler class should be a pimpl class that manages the I2C communication
// 2. I2CHandlerImpl class should be the implementation of the I2C communication

// 3. Communication with I2C should be done via a builder pattern (I2CTransactionBuilder)
// 4. I2CTransactionBuilder should be a class that builds I2C transactions, from sequences of operations


class I2COperation {
public:
    virtual ~I2COperation() = default;
    virtual uint64_t get_message_flags() const = 0;
    virtual uint64_t get_data_size() const = 0;
    virtual const void* get_data() const = 0;
};

template <typename T>
class I2CWriteOperation : public I2COperation{
public:
    template <I2CMessageFlags ...Flags>
    I2CWriteOperation(T&& data, Flags&&... flags) : data_(std::move(data)), flags_{flags ...} {}

private:
    T data_;
    uint16_t flags_;
};


// read operations are the most complicated
// as we need to do a bit_cast
class I2CReadOperation : public I2COperation {
public:
    template <I2CMessageFlags ...Flags>
    I2CWriteOperation(T&& data, Flags&&... flags) : data_(std::move(data)), flags_{flags...} {}

private:
    T data_;
    uint16_t flags_;
};



enum I2CMessageFlags: uint16_t
{
  M_WR = 0x0000, // I2C message flag to indicate a write transfer (from master to slave).
  M_RD = 0x0001,        // I2C message flag to indicate a read transfer (from slave to master).
  M_TEN = 0x0010,       // I2C message flag to indicate a 10-bit address. The controller must support FUNC_10BIT_ADDR to use this.
  M_DMA_SAFE = 0x0200,  // (Do not use outside of kernel-space!!) I2C message flag to indicate that the message buffer is DMA-safe.
  M_RECV_LEN = 0x0400,  // I2C message flag to indicate that the message data length is the first received byte.
  M_NO_RD_ACK = 0x0800, // I2C message flag to omit a master acknowledge/non-acknowledge in a read transfer.
  M_IGNORE_NAK = 0x1000,        // I2C message flag to ignore a non-acknowledge. The controller must support FUNC_PROTOCOL_MANGLING to use this.
  M_REV_DIR_ADDR = 0x2000,      // I2C message flag to reverse the direction flag. The controller must support FUNC_PROTOCOL_MANGLING to use this.
  M_NOSTART = 0x4000,   // I2C message flag to omit start condition and slave address. The controller must support FUNC_PROTOCOL_MANGLING to use this.
  M_STOP = 0x8000,      // I2C message flag to signal a stop condition even if this is not the last message. The controller must support FUNC_PROTOCOL_MANGLING to use this.
};

typename<typename... FlagsT>
uint16_t convert_flags(FlagsT&&... flags){
    // check if all FlagsT are I2CMessageFlags
    static_assert((std::is_same_v<FlagsT, I2CMessageFlags>...)) || (std::is_same_v<FlagsT, I2CControllerFunctionalityFlags>...), "FlagsT must be I2CMessageFlags or I2CControllerFunctionalityFlags");
    return (static_cast<uint16_t>(flags) | ...)
}




#endif // I2CPP_HPP
