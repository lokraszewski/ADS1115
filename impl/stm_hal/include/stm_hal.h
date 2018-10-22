#pragma once

#if !defined(USE_HAL_DRIVER)
#error HAL DRIVER NOT DEFINED.
#endif /* USE_HAL_DRIVER */

/* change this to whatever hal you are using */
#include "error.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>




namespace stm_hal
{

class i2c
{
  using size_t = uint32_t;

public:
  i2c(void* const arg);
  ~i2c();
  ADS1115::Error begin(const uint8_t address);
  ADS1115::Error write(uint8_t* data, const size_t length);
  ADS1115::Error read(uint8_t* data, const size_t length);

private:
  static constexpr auto    TIMEOUT = 100;
  I2C_HandleTypeDef* const p_i2c;
  uint8_t                  m_address;
};

} // namespace stm_hal
