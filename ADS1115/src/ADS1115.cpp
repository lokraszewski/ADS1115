#include "ADS1115.h"

namespace ADS1115
{
ADC::ADC(std::string port, const uint8_t address) : m_impl(new i2cImpl(port)), m_address(address) {}

ADC::~ADC() { delete m_impl; }

uint16_t ADC::read_config_register(void)
{
  uint8_t tx[1] = {RegisterAddress::Config};
  uint8_t rx[2];

  m_impl->begin(m_address);
  m_impl->write(tx, 1);
  m_impl->read(rx, 2);

  return (rx[0] << 8) | rx[1];
}
} // namespace ADS1115
