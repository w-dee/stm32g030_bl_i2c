#include "stm32_bl_i2c_writer.hpp"
#include "Wire.h"
#include "debug.h"

#define Wire ERROR // in this module, global Wire instance must not be used. use _Wire instead.

int STM32_Bootloader_I2C_Writer::send_cmd(uint8_t cmd)
{
    uint8_t chksum = 0xff;
    dbg_printf("Sending command 0x%02x to the slave address 0x%02x\r\n", cmd, target_slave_address);
    wire->beginTransmission(target_slave_address);
    wire->write(cmd);
    chksum ^= cmd;
    wire->write(chksum);
    return wire->endTransmission(); // returns 0 for success
}


// read 1 byte from the target. returns < 0 when failed.
int STM32_Bootloader_I2C_Writer::read()
{
  wire->requestFrom(target_slave_address, (uint8_t)1, (uint8_t)1);
  int ret = wire->read();
  if(ret > 256) ret = -1;
  return ret;
}

// wait ack from the target. if nack is received or timedout, returns non-zero value.
int STM32_Bootloader_I2C_Writer::wait_ack()
{
  uint32_t limit = millis() + 10000; // TODO: magic number
  while((int32_t)(millis() - limit)< 0)
  {
    switch(read())
    {
      case STM32_BL_I2C_RET_BUSY:
        break;
      case STM32_BL_I2C_RET_NACK:
        dbg_printf("NACK received.\r\n");
        return -1; // nack returned
      case STM32_BL_I2C_RET_ACK:
        return 0;
    }
  }
  dbg_printf("No response from the target.\r\n");
  return -2;
}

void STM32_Bootloader_I2C_Writer::target_reset()
{
    // perform target reset
    if(wire) wire_end(wire), wire = nullptr;
    dbg_printf("Target reset.\r\n");
    pinMode(boot0_pin, OUTPUT);
    digitalWrite(boot0_pin, HIGH);
    pinMode(nrst_pin, OUTPUT);
    digitalWrite(nrst_pin, LOW);
    delay(10);
    // release target reset pin
    pinMode(nrst_pin, INPUT);
    delay(2); // according to the RM, BOOT0 pin is sampled at 4th clock since the reset.
    pinMode(boot0_pin, INPUT);
    delay(20);
    wire = wire_begin();
}


bool STM32_Bootloader_I2C_Writer::write(const uint8_t *binary, uint32_t size, uint32_t address)
{
  if(address % flash_page_size != 0)
  {
    dbg_printf("The address %08d must be aligned with flash page size(%d)\r\n", address, flash_page_size);
    return false;
  }


  uint32_t ret;
  uint32_t count;

  uint32_t limit;
  bool first_try = true;
  for(limit = millis() +  STM32_BL_I2C_DISCOVERY_RETRY_TIMEOUT;
    (int32_t)(millis() - limit) < 0; )
  {
    if(!first_try)
    {
        target_reset();
        dbg_printf("Waiting for retry.\r\n");
        delay(STM32_BL_I2C_DISCOVERY_RETRY_WAIT_TIME);
        dbg_printf("Retrying.\r\n");
    }
    first_try = false;

    target_reset();

    // the target STM32 slave address is 0x56
    // (sometimes it becomes 0x57)
    // send "GET" command
    send_cmd(STM32_BL_I2C_CMD_GET);
    // receive ACK
    ret = read();
    if(ret != STM32_BL_I2C_RET_ACK) 
    {
        dbg_printf("The target STM32 does not return the response.\r\n");
        continue;
    }

    count = read();
    if(count >= 255)
    {
      dbg_printf("Illegal feature count returned from GET command.\r\n");
      continue;
    }
    dbg_printf("Feature count: %d\r\n", count);
    ret = read();
    if(ret == STM32_BL_I2C_RET_ACK)
    {
      // undocumented in AN4221 ?
      // My STM32G030 returns ACK as a protocol version; Rest of
      // read is all NACK. May be a shrunken version of BL is used.
      dbg_printf("ACK received as a protocol version. Continuing.\r\n");
      goto successfully_discovered;
    }

    dbg_printf("Protocol version: 0x%02x\r\n", wire->read());

    dbg_printf("Features :");
    for(int i = 0; i < count; ++i)
    {
      dbg_printf("%02X ", read());
    }
    dbg_printf("\r\n");
    ret = read();
    if(ret != STM32_BL_I2C_RET_ACK) continue;

  }

  dbg_printf("Target STM32 not found.\r\n");
  return false;

successfully_discovered:
  // get protocol version
  send_cmd(STM32_BL_I2C_CMD_GET_PROTOCOL_VERSION);
  ret = read();
  if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;
  ret = read();
  dbg_printf("Protocol Version: %d.%d\r\n", ret >> 4, ret & 0x0f);
  ret = read();
  if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;


  // erase and write the firmware
  for(; address < size; address += 256)
  {
    if((address % flash_page_size) == 0)
    {
      uint16_t page_num = (uint16_t)(address / flash_page_size);
      dbg_printf("Erasing page number %d (flash address %08x)\r\n", (int)page_num, (int)address);

      // the page must be erased
      send_cmd(STM32_BL_I2C_CMD_NO_STRETCH_ERASE_MEMORY); // 0x45 = No-Stretch Erase Memory command
      ret = read();
      if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;
      
      // send number of pages to be erased
      wire->beginTransmission(target_slave_address);
      wire->write(0);
      wire->write(0);
      wire->write(0); // for single page erase
      wire->endTransmission();
      if(wait_ack()) goto illegal_response;

      // send page number to be erased
      wire->beginTransmission(target_slave_address);
      wire->write(page_num >> 8);
      wire->write(page_num & 0xff);
      wire->write(((page_num >> 8) | page_num) & 0xff); // XOR checksum
      wire->endTransmission();
      if(wait_ack()) goto illegal_response;
      dbg_printf("Ok.\r\n");
    }

    // write flash content
    size_t num_bytes = std::min((size_t)256, (size_t)(size - address));
    dbg_printf("Writing %d bytes to address %08x\r\n", (int)num_bytes, (int)address + STM32_BL_FLASH_MEMORY_START);

    // write content
    send_cmd(STM32_BL_I2C_CMD_NO_STRETCH_WRITE_MEMORY); // 0x32 = No-Stretch Write Memory command
    ret = read();
    if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;
    
    // send start address
    uint32_t start_addr = address + STM32_BL_FLASH_MEMORY_START;
    uint8_t bytes[4] = {
      (uint8_t )(start_addr >> 24),
      (uint8_t )(start_addr >> 16),
      (uint8_t )(start_addr >>  8),
      (uint8_t )(start_addr >>  0)
    };
    wire->beginTransmission(target_slave_address);
    wire->write(bytes[0]);
    wire->write(bytes[1]);
    wire->write(bytes[2]);
    wire->write(bytes[3]);
    wire->write(bytes[0] ^ bytes[1] ^ bytes[2] ^ bytes[3]);
    wire->endTransmission();
    if(wait_ack()) goto illegal_response;

    // send content
    wire->beginTransmission(target_slave_address);
    wire->write(num_bytes - 1);
    uint8_t sum = num_bytes - 1;
    for(size_t i = address; i < address + num_bytes; ++i)
    {
      wire->write(binary[i]);
      sum ^= binary[i];
    }
    wire->write(sum); // XOR checksum
    wire->endTransmission();
    if(wait_ack()) goto illegal_response;

    dbg_printf("Write ok.\r\n");

  }

  dbg_printf("Firmware write done.\r\n");
#if 0
  // verify
  {
    dbg_printf("Checking written firmware's CRC.\r\n");
    uint32_t firmware_bytes = BM_FIRMWARE_BINARY_SIZE;
    uint8_t fb_bytes[4]  = {
      (uint8_t )(firmware_bytes >> 24),
      (uint8_t )(firmware_bytes >> 16),
      (uint8_t )(firmware_bytes >>  8),
      (uint8_t )(firmware_bytes >>  0)
    };
    bl_send_cmd(0xa1); // 0xa1 = No-Stretch GetCheckSum command
    ret = bl_read();
    if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;

    dbg_printf("Sending start address.\r\n");
    wire->beginTransmission(STM32G030_BL_I2C_SLAVE_ADDRESS);
    wire->write(0x08);
    wire->write(0);
    wire->write(0);
    wire->write(0);
    wire->write(0x08); // checksum
    wire->endTransmission();
    if(bl_wait_ack()) goto illegal_response;
      
    // send compute size
    dbg_printf("Sending firmware size.\r\n");
    wire->beginTransmission(STM32G030_BL_I2C_SLAVE_ADDRESS);
    wire->write(fb_bytes[0]);
    wire->write(fb_bytes[1]);
    wire->write(fb_bytes[2]);
    wire->write(fb_bytes[3]);
    wire->write(fb_bytes[0] ^ fb_bytes[1] ^ fb_bytes[2] ^ fb_bytes[3]);
    wire->endTransmission();
    if(bl_wait_ack()) goto illegal_response;

    dbg_printf("Waiting for the calculation done.\r\n");
    if(bl_wait_ack()) goto illegal_response;

    dbg_printf("Receiving the CRC.\r\n");
    wire->requestFrom((int)STM32G030_BL_I2C_SLAVE_ADDRESS, (int)5);
    fb_bytes[0] = wire->read();
    fb_bytes[1] = wire->read();
    fb_bytes[2] = wire->read();
    fb_bytes[3] = wire->read();
    ret = wire->read();
    dbg_printf("Received : %02x %02x %02x %02x (XOR checksum %02x)\r\n", fb_bytes[0] , fb_bytes[1] , fb_bytes[2] , fb_bytes[3], ret);
    ret ^= fb_bytes[0];
    ret ^= fb_bytes[1];
    ret ^= fb_bytes[2];
    ret ^= fb_bytes[3];
    if(0 != ret)
    {
      dbg_printf("XOR checksum of CRC result is invalid.\r\n");
      goto illegal_response;
    }
    dbg_printf("The firmware CRC is : %02x%02x%02x%02x\r\n", fb_bytes[0] , fb_bytes[1] , fb_bytes[2] , fb_bytes[3]);
    dbg_printf("Expected CRC is     : %08x\r\n", BM_FIRMWARE_BINARY_CRC);
    if(BM_FIRMWARE_BINARY_CRC != 
      (uint32_t)(fb_bytes[0] << 24) +
      (uint32_t)(fb_bytes[1] << 16) +
      (uint32_t)(fb_bytes[2] << 8) +
      (uint32_t)(fb_bytes[3] << 0) )
    {
      dbg_printf("CRC check failed.\r\n");
      goto illegal_response;
    }
  }
#endif
  dbg_printf("Firmware has been written successfully.\r\n");

  return true;

nack_received:
  dbg_printf("Target STM32 does not return ACK.\r\n");
  goto illegal_response;

illegal_response:
  dbg_printf("STM32 firmware uploading failed.\r\n");
  return false;

}
