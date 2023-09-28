#include <Arduino.h>
#include "debug.h"
#include "Wire.h"
#include <algorithm>
#include "stm32_bl_i2c_writer.hpp"

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


#include "binary.h"

void wire_start()
{
  Wire.begin((uint32_t)SDA_PIN, (uint32_t)SCL_PIN);
}

void wire_end()
{
  Wire.end();
}
constexpr int STM32G030_NRST_PIN = PA4; // the host pin connected to the target reset pin
constexpr int STM32G030_BOOT0_PIN = PA5; // the host pin connected to the target boot0 pin


void setup() {
  STM32_Bootloader_Init_Struct bl_init;
  bl_init.wire = nullptr;
  bl_init.target_slave_address = 0x56;
  bl_init.flash_page_size = 2048;
  bl_init.wire_begin = [] () -> TwoWire * { Wire.begin((uint32_t)SDA_PIN, (uint32_t)SCL_PIN); return &Wire; };
  bl_init.wire_end = [] (TwoWire * wire) -> void { wire->end(); };
  bl_init.nrst_pin = STM32G030_NRST_PIN;
  bl_init.boot0_pin = STM32G030_BOOT0_PIN;
  STM32_Bootloader_I2C_Writer writer(bl_init);

  writer.write(BM_FIRMWARE_BINARY, BM_FIRMWARE_BINARY_SIZE, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  dump_i2c();
  delay(1000);
}
