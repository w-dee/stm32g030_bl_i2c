#include <Arduino.h>
#include "debug.h"
#include "Wire.h"
#include <algorithm>

#define SDA_PIN PB9 // pin46
#define SCL_PIN PB8 // pin45

extern "C" {
  // Override SystemClock_Config() to use the internal clock
  void SystemClock_Config(void)
  {
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
      Error_Handler();
    }
  }
}

void dump_i2c()
{
    dbg_printf("I2C subsystem initializing...\n");

    dbg_printf("I2C map:\n  ");
    int addr;
    for(addr = 0; addr < 128; addr++ )
    {
        if(addr == 0 || addr == 127)
        {
            dbg_printf("xx ");
            continue;
        }
        Wire.beginTransmission(addr);
        int error = Wire.endTransmission();
    
        if (error == 0)
        {
            dbg_printf("%02x ", addr);
        }
        else
        {
            dbg_printf("-- ");
        }
        if((addr & 0x0f) == 0x0f) dbg_printf("\n  ");
    }
    dbg_printf("\n");
}

constexpr int STM32G030_NRST_PIN = PA4; // the host pin connected to the target reset pin
constexpr uint8_t STM32G030_BL_I2C_SLAVE_ADDRESS = 0x56;
constexpr size_t STM32G030_PAGE_SIZE = 2048; // 2048 bytes page size (may differ by product)
constexpr uint8_t STM32_BL_I2C_CMD_GET = 0x00;
constexpr uint8_t STM32_BL_I2C_RET_ACK = 0x79;
constexpr uint8_t STM32_BL_I2C_RET_NACK = 0x1f;
constexpr uint8_t STM32_BL_I2C_RET_BUSY = 0x76;
constexpr int STM32_BL_I2C_DISCOVERY_RETRY  = 3;
constexpr uint32_t STM32_BL_FLASH_MEMORY_START = 0x08000000;

#include "binary.h"

void wire_start()
{
  Wire.begin((uint32_t)SDA_PIN, (uint32_t)SCL_PIN);
}

void wire_end()
{
  Wire.end();
}

static int bl_send_cmd(uint8_t cmd, const uint8_t * payload = nullptr, size_t payload_count = 0)
{
    uint8_t chksum = 0xff;
    dbg_printf("Sending command 0x%02x to the slave address 0x%02x\r\n", cmd, STM32G030_BL_I2C_SLAVE_ADDRESS);
    Wire.beginTransmission(STM32G030_BL_I2C_SLAVE_ADDRESS);
    Wire.write(cmd);
    chksum ^= cmd;
    for(size_t i = 0; i < payload_count; ++ i)
    {
      Wire.write(payload[i]);
      chksum ^= payload[i];
    }
    Wire.write(chksum);
    return Wire.endTransmission(); // returns 0 for success
}

// read 1 byte from the target. returns < 0 when failed.
static int bl_read()
{
  Wire.requestFrom(STM32G030_BL_I2C_SLAVE_ADDRESS, (uint8_t)1, (uint8_t)1);
  int ret = Wire.read();
  if(ret > 256) ret = -1;
  return ret;
}

// wait ack from the target. if nack is received or timedout, returns non-zero value.
static int bl_wait_ack()
{
  uint32_t limit = millis() + 10000; // TODO: magic number
  while((int32_t)(millis() - limit)< 0)
  {
    switch(bl_read())
    {
      case STM32_BL_I2C_RET_BUSY:
        break;
      case STM32_BL_I2C_RET_NACK:
        dbg_printf("NACK received.\r\n");
        return -1; // nack retuned
      case STM32_BL_I2C_RET_ACK:
        return 0;
    }
  }
  dbg_printf("No responce from the target.\r\n");
  return -2;
}

bool write_stm32g030_firmware()
{
  uint32_t ret;
  uint32_t count;

  for(int retry = 1; retry <=  STM32_BL_I2C_DISCOVERY_RETRY; ++ retry)
  {
    dbg_printf("Discovering target STM32, try %d\r\n", retry);
    wire_end();

    dbg_printf("Checking SCL pin.\r\n");
    // check SCL pin. By unknown reason, STM32 BL may
    // pull SCL pin for a long time (about 7 sec).
    pinMode(SCL_PIN, INPUT_PULLUP);
    if(!digitalRead(SCL_PIN))
    {
      dbg_printf("SCL pin is pulled down. Waiting for release.\r\n");
      uint32_t limit = millis() + 10000; // TODO: remove magic number
      while((int32_t)(millis() - limit)< 0)
      {
        if(digitalRead(SCL_PIN))
        {
         dbg_printf("SCL pin released. Continuing.\r\n");
         goto target_reset;
        }
      }
    }
    else
    {
      goto target_reset;
    }

    wire_start();
    dbg_printf("SCL not released. Quitting.\r\n");
    return false;

target_reset:
    wire_start();
    // perfom target reset
    dbg_printf("Target reset\r\n");
    pinMode(STM32G030_NRST_PIN, OUTPUT);
    digitalWrite(STM32G030_NRST_PIN, LOW);
    delay(10);
    // release target reset pin
    pinMode(STM32G030_NRST_PIN, INPUT);
    delay(200);
    // the target STM32 slave address is 0x56
    // (sometimes it becomes 0x57)
    // send "GET" command
    bl_send_cmd(0x00);
    // receive ACK
    ret = bl_read();
    if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;

    count = bl_read();
    if(count >= 255)
    {
      dbg_printf("Illegal feature count returned from GET command.\r\n");
      goto illegal_responce;
    }
    dbg_printf("Feature count: %d\r\n", count);
    ret = bl_read();
    if(ret == STM32_BL_I2C_RET_ACK)
    {
      // undocumented in AN4221 ?
      // My STM32G030 returns ACK as a protocol version; Rest of
      // read is all NACK. May be a shrinken version of BL is used.
      dbg_printf("ACK received as a protocol version. Continuing.\r\n");
      goto successfully_discovered;
    }

    dbg_printf("Protocol version: 0x%02x\r\n", Wire.read());

    dbg_printf("Features :");
    for(int i = 0; i < count; ++i)
    {
      dbg_printf("%02X ", bl_read());
    }
    dbg_printf("\r\n");
    ret = bl_read();
    if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;

  }

  dbg_printf("Target STM32 not found.\r\n");
  return false;

successfully_discovered:
  // get protocol version
  bl_send_cmd(0x01);
  ret = bl_read();
  if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;
  ret = bl_read();
  dbg_printf("Protocol Version: %d.%d\r\n", ret >> 4, ret & 0x0f);
  ret = bl_read();
  if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;


  // erase and write the firmware
  for(size_t address = 0; address < BM_FIRMWARE_BINARY_SIZE; address += 256)
  {
    if((address % STM32G030_PAGE_SIZE) == 0)
    {
      uint16_t page_num = (uint16_t)(address / STM32G030_PAGE_SIZE);
      dbg_printf("Erasing page number %d (flash address %08x)\r\n", (int)page_num, (int)address);

      // the page must be erased
      bl_send_cmd(0x45); // 0x45 = No-Stretch Erase Memory command
      ret = bl_read();
      if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;
      
      // send number of pages to be erased
      Wire.beginTransmission(STM32G030_BL_I2C_SLAVE_ADDRESS);
      Wire.write(0);
      Wire.write(0);
      Wire.write(0); // for single page erase
      Wire.endTransmission();
      if(bl_wait_ack()) goto illegal_responce;

      // send page number to be erased
      Wire.beginTransmission(STM32G030_BL_I2C_SLAVE_ADDRESS);
      Wire.write(page_num >> 8);
      Wire.write(page_num & 0xff);
      Wire.write(((page_num >> 8) | page_num) & 0xff); // XOR checksum
      Wire.endTransmission();
      if(bl_wait_ack()) goto illegal_responce;
      dbg_printf("Ok.\r\n");
    }

    // write flash content
    size_t num_bytes = std::min((size_t)256, BM_FIRMWARE_BINARY_SIZE - address);
    dbg_printf("Writing %d bytes to address %08x\r\n", (int)num_bytes, (int)address + STM32_BL_FLASH_MEMORY_START);

    // write content
    bl_send_cmd(0x32); // 0x32 = No-Stretch Write Memory command
    ret = bl_read();
    if(ret != STM32_BL_I2C_RET_ACK) goto nack_received;
    
    // send start address
    uint32_t start_addr = address + STM32_BL_FLASH_MEMORY_START;
    uint8_t bytes[4] = {
      (uint8_t )(start_addr >> 24),
      (uint8_t )(start_addr >> 16),
      (uint8_t )(start_addr >>  8),
      (uint8_t )(start_addr >>  0)
    };
    Wire.beginTransmission(STM32G030_BL_I2C_SLAVE_ADDRESS);
    Wire.write(bytes[0]);
    Wire.write(bytes[1]);
    Wire.write(bytes[2]);
    Wire.write(bytes[3]);
    Wire.write(bytes[0] ^ bytes[1] ^ bytes[2] ^ bytes[3]);
    Wire.endTransmission();
    if(bl_wait_ack()) goto illegal_responce;

    // send content
    Wire.beginTransmission(STM32G030_BL_I2C_SLAVE_ADDRESS);
    Wire.write(num_bytes - 1);
    uint8_t sum = num_bytes - 1;
    for(size_t i = address; i < address + num_bytes; ++i)
    {
      Wire.write(BM_FIRMWARE_BINARY[i]);
      sum ^= BM_FIRMWARE_BINARY[i];
    }
    Wire.write(sum); // XOR checksum
    Wire.endTransmission();
    if(bl_wait_ack()) goto illegal_responce;

    dbg_printf("Write ok.\r\n");

  }

  dbg_printf("Firmware write done.\r\n");

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
    Wire.beginTransmission(STM32G030_BL_I2C_SLAVE_ADDRESS);
    Wire.write(0x08);
    Wire.write(0);
    Wire.write(0);
    Wire.write(0);
    Wire.write(0x08); // checksum
    Wire.endTransmission();
    if(bl_wait_ack()) goto illegal_responce;
      
    // send compute size
    dbg_printf("Sending firmware size.\r\n");
    Wire.beginTransmission(STM32G030_BL_I2C_SLAVE_ADDRESS);
    Wire.write(fb_bytes[0]);
    Wire.write(fb_bytes[1]);
    Wire.write(fb_bytes[2]);
    Wire.write(fb_bytes[3]);
    Wire.write(fb_bytes[0] ^ fb_bytes[1] ^ fb_bytes[2] ^ fb_bytes[3]);
    Wire.endTransmission();
    if(bl_wait_ack()) goto illegal_responce;

    dbg_printf("Waiting for the calculation done.\r\n");
    if(bl_wait_ack()) goto illegal_responce;

    dbg_printf("Receiving the CRC.\r\n");
    Wire.requestFrom((int)STM32G030_BL_I2C_SLAVE_ADDRESS, (int)5);
    fb_bytes[0] = Wire.read();
    fb_bytes[1] = Wire.read();
    fb_bytes[2] = Wire.read();
    fb_bytes[3] = Wire.read();
    ret = Wire.read();
    dbg_printf("Received : %02x %02x %02x %02x (XOR checksum %02x)\r\n", fb_bytes[0] , fb_bytes[1] , fb_bytes[2] , fb_bytes[3], ret);
    ret ^= fb_bytes[0];
    ret ^= fb_bytes[1];
    ret ^= fb_bytes[2];
    ret ^= fb_bytes[3];
    if(0 != ret)
    {
      dbg_printf("XOR checksum of CRC result is invalid.\r\n");
      goto illegal_responce;
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
      goto illegal_responce;
    }
  }

  dbg_printf("Firmware has been written successfully.\r\n");

  return true;

nack_received:
  dbg_printf("Target STM32 does not return ACK.\r\n");
  goto illegal_responce;

illegal_responce:
  dbg_printf("STM32 firmware uploading failed.\r\n");
  return false;

}

void setup() {
  wire_start();
  write_stm32g030_firmware();
}

void loop() {
  // put your main code here, to run repeatedly:
  dump_i2c();
  delay(1000);
}
