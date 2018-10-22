#include "stm_hal.h"

namespace stm_hal
{

namespace
{
ADS1115::Error get_err(const HAL_StatusTypeDef err)
{
  switch (err)
  {
  case HAL_OK: return ADS1115::Error::NONE;
  case HAL_BUSY: return ADS1115::Error::BUSY;
  case HAL_TIMEOUT: return ADS1115::Error::TIMEOUT;
  case HAL_ERROR:
  default: return ADS1115::Error::ERROR;
  }
}
}; // namespace

i2c::i2c(void* const arg) : p_i2c(static_cast<I2C_HandleTypeDef*>(arg)) {  }
i2c::~i2c() {}

ADS1115::Error i2c::begin(const uint8_t address)
{
  m_address = address;
  return ADS1115::Error::NONE;
}
ADS1115::Error i2c::write(uint8_t* data, const size_t length) { return get_err(HAL_I2C_Master_Transmit(p_i2c, m_address, data, length, TIMEOUT)); }
ADS1115::Error i2c::read(uint8_t* data, const size_t length) { return get_err(HAL_I2C_Master_Receive(p_i2c, m_address, data, length, TIMEOUT)); }

} // namespace stm_hal
