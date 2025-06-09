#ifndef I2CPP_CONSTANTS_HPP
#define I2CPP_CONSTANTS_HPP

#include <cstdint>

namespace i2cpp
{

// Based on i2c.h, i2c-dev.c, i2c-dev.h and the documentation at https://www.kernel.org/doc/Documentation/i2c
// and https://ftp.rtems.org/pub/rtems/releases/5/5.1/docs/html/doxygen/group__I2CLinux.html

enum I2CConstants: int
{
  I2C_TRANSACTION_IOCTL_MAX_MSGS = 42,
  SMBUS_BLOCK_MAX = 32
};

/**
 * Flags that indicate the functionality of a given I2C controller.
 */
enum I2CControllerFunctionalityFlags: uint64_t
{
  FUNC_I2C =                    0x00000001, // Plain i2c-level commands (Pure SMBus adapters typically can not do these)
  FUNC_10BIT_ADDR =             0x00000002, // Handles the 10-bit address extensions
  FUNC_PROTOCOL_MANGLING =      0x00000004, // Knows about the I2C_M_IGNORE_NAK, I2C_M_REV_DIR_ADDR and I2C_M_NO_RD_ACK flags (which modify the I2C protocol!)
  FUNC_SMBUS_PEC =              0x00000008, // Supports Packet Error Checking (PEC)
  FUNC_NOSTART =                0x00000010, // Can skip repeated start sequence
  FUNC_SLAVE =                  0x00000020, // Indicates the device is operating only as a slave interface.
  FUNC_SMBUS_BLOCK_PROC_CALL =  0x00008000, // Handles the SMBus block_process_call command
  FUNC_SMBUS_QUICK =            0x00010000, // Handles the SMBus write_quick command
  FUNC_SMBUS_READ_BYTE =        0x00020000, // Handles the SMBus read_byte command
  FUNC_SMBUS_WRITE_BYTE =       0x00040000, // Handles the SMBus write_byte command
  FUNC_SMBUS_READ_BYTE_DATA =   0x00080000, // Handles the SMBus read_byte_data command
  FUNC_SMBUS_WRITE_BYTE_DATA =  0x00100000, // Handles the SMBus write_byte_data command
  FUNC_SMBUS_READ_WORD_DATA =   0x00200000, // Handles the SMBus read_word_data command
  FUNC_SMBUS_WRITE_WORD_DATA =  0x00400000, // Handles the SMBus write_byte_data command
  FUNC_SMBUS_PROC_CALL =        0x00800000, // Handles the SMBus process_call command
  FUNC_SMBUS_READ_BLOCK_DATA =  0x01000000, // Handles the SMBus read_block_data command
  FUNC_SMBUS_WRITE_BLOCK_DATA = 0x02000000, // Handles the SMBus write_block_data command
  FUNC_SMBUS_READ_I2C_BLOCK =   0x04000000, // Handles the SMBus read_i2c_block_data command
  FUNC_SMBUS_WRITE_I2C_BLOCK =  0x08000000, // Handles the SMBus write_i2c_block_data command
  FUNC_SMBUS_HOST_NOTIFY =      0x10000000, // Indicates that it supports Host Notify command

  /* Combo-d features as defined in i2c.h */

  FUNC_SMBUS_BYTE       = (FUNC_SMBUS_READ_BYTE | FUNC_SMBUS_WRITE_BYTE), // Handles the SMBus read_byte and write_byte commands
  FUNC_SMBUS_BYTE_DATA = (FUNC_SMBUS_READ_BYTE_DATA | FUNC_SMBUS_WRITE_BYTE_DATA), // Handles the SMBus read_byte_data and write_byte_data commands
  FUNC_SMBUS_WORD_DATA = (FUNC_SMBUS_READ_WORD_DATA | FUNC_SMBUS_WRITE_WORD_DATA), // Handles the SMBus read_word_data and write_word_data commands
  FUNC_SMBUS_BLOCK_DATA =(FUNC_SMBUS_READ_BLOCK_DATA | FUNC_SMBUS_WRITE_BLOCK_DATA), // Handles the SMBus read_block_data and write_block_data commands
  FUNC_SMBUS_I2C_BLOCK = (FUNC_SMBUS_READ_I2C_BLOCK | FUNC_SMBUS_WRITE_I2C_BLOCK), // Handles the SMBus read_i2c_block_data and write_i2c_block_data commands
  FUNC_SMBUS_EMUL       = (FUNC_SMBUS_QUICK | FUNC_SMBUS_BYTE | FUNC_SMBUS_BYTE_DATA |
    FUNC_SMBUS_WORD_DATA | FUNC_SMBUS_PROC_CALL | FUNC_SMBUS_WRITE_BLOCK_DATA |
    FUNC_SMBUS_I2C_BLOCK |
    FUNC_SMBUS_PEC), // Handles all SMBus commands that can be emulated by a real I2C adapter (using the transparent emulation layer)
  FUNC_SMBUS_EMUL_ALL = (FUNC_SMBUS_EMUL | FUNC_SMBUS_READ_BLOCK_DATA | FUNC_SMBUS_BLOCK_PROC_CALL) // Expands FUNC_SMBUS_EMUL if M_RECV_LEN is also supported
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

enum I2CIOControlCommands: uint64_t
{
  RETRIES = 0x701, // Sets the count of transfer retries in case a slave device does not acknowledge a transaction.
  TIMEOUT = 0x702, // Sets the transfer timeout in 10ms units.
  SLAVE = 0x703, // Sets the slave address.
  SLAVE_FORCE = 0x706, // Forces setting the slave address.
  TENBIT = 0x704, // Enables 10-bit addresses if argument is non-zero, otherwise disables 10-bit addresses.
  FUNCS = 0x705, // Gets the I2C controller functionality information.
  RDWR = 0x707, // Performs a combined read/write transfer.
  PEC = 0x708, // Enables System Management Bus (SMBus) Packet Error Checking (PEC) if argument is non-zero, otherwise disables PEC.
};

}  // namespace i2cpp

#endif // I2CPP_CONSTANTS_HPP
