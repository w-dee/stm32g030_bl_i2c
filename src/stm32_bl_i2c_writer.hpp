#pragma once
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include <functional>




class TwoWire;
struct STM32_Bootloader_Init_Struct
{
  TwoWire * wire = nullptr; //!< TwoWire instance to communicate with the target STM32. Specify null if the TwoWire instance is not begin()'ed.
  uint8_t target_slave_address = 0x56; //!< Target slave address of the STM32 in 7bit address representation.
  size_t flash_page_size = 2048; //!< Flash page size. This might differ from variant to variant. See data sheet or reference manual.

  /*
    Function object Wire.begin(); 
    By unknown reason, some Wire implementation (at least STM32)
    does wrongly initialized when the bus is in illegal state (eg. SCL is pulled low).
    User must provide the function that calls Wire.begin(with your arguments); and
    return the pointer to the TwoWire object.
  */
  std::function<TwoWire * () > wire_begin;

  /*
    Function object Wire.end();
    User must provide the function that calls Wire.end();
    The function is called with the pointer to TwoWire object which is in turn
    returned by wire_begin().
    see wire_begin for detail.
  */
  std::function<void (TwoWire *)> wire_end;

  int nrst_pin = -1; //!< The host pin connected to the target reset pin.
  int boot0_pin = -1; //!< The host pin connected to the target boot0 pin.

};


class STM32_Bootloader_I2C_Writer : private STM32_Bootloader_Init_Struct
{
  static constexpr uint8_t STM32_BL_I2C_CMD_GET = 0x00; //!< "GET" command
  static constexpr uint8_t STM32_BL_I2C_CMD_GET_PROTOCOL_VERSION = 0x01; //!< "GET Protocol Version" command
  static constexpr uint8_t STM32_BL_I2C_CMD_NO_STRETCH_ERASE_MEMORY = 0x45; //!< No-stretch erase memory command
  static constexpr uint8_t STM32_BL_I2C_CMD_NO_STRETCH_WRITE_MEMORY = 0x32; //!< No-stretch write memory command
  static constexpr uint8_t STM32_BL_I2C_CMD_NO_STRETCH_GET_CHECK_SUM = 0xa1; //!< No-stretch get check sum command
  
  static constexpr uint8_t STM32_BL_I2C_RET_ACK = 0x79; //!< Acknowledge response from STM32 boot loader 
  static constexpr uint8_t STM32_BL_I2C_RET_NACK = 0x1f; //!< Not acknowledge response from STM32 boot loader
  static constexpr uint8_t STM32_BL_I2C_RET_BUSY = 0x76; //!< Busy response from STM32 boot loader
  static constexpr uint32_t STM32_BL_I2C_DISCOVERY_RETRY_TIMEOUT  = 10000; //!< Target discovery time out in ms
  static constexpr uint32_t STM32_BL_I2C_DISCOVERY_RETRY_WAIT_TIME = 1000; //!< Target discovery wait time between try
  static constexpr uint32_t STM32_BL_FLASH_MEMORY_START = 0x08000000; //!< Address as in flat memory space at which the flash memory is mapped

private:
  /* send bl cmd to the target. returns 0 for success.
  */
  int send_cmd(uint8_t cmd);

  // read 1 byte from the target. returns < 0 when failed.
  int read();

  // wait ack from the target. if nack is received or timedout, returns non-zero value.
  int wait_ack();

  // issue target a reset
  void target_reset_into_bl();


public:
  // The constructor.
  // @param   init    initialisation structure.
  STM32_Bootloader_I2C_Writer(const STM32_Bootloader_Init_Struct &init) :
    STM32_Bootloader_Init_Struct(init) {;}


  // Write the firmware.
  // @param binary    the firmware binary
  // @param size      the binary size. the size must be multiple of 4.
  // @param address   the start address of the binary. 0 is the first position of flash.
  // @return          false for error, true for success.
  // @note Address must be aligned with flash page size specified with the constructor.
  //   Size does not need to be aligned, but the any entire flash page which the
  //   binary (even one byte in the page) is to be written, is to be erased.
  //   Writing to the option byte is not supported yet.
  bool write(const uint8_t *binary, uint32_t size, uint32_t address);

  // Issue reset to the target STM32
  // @note
  // This performs normal hardware reset (not bootloader reset) to the target STM32
  void target_reset();

};
