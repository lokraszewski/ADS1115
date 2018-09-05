#pragma once

#include <exception>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef __linux__
#include "unix.h" // i2cImpl
#else
#error "No imeplementation found. "
#endif

namespace ADS1115
{

enum RegisterAddress : uint8_t
{
  Conversion = 0b00,
  Config     = 0b01,
  Lo_thresh  = 0b10,
  Hi_thresh  = 0b11,
};

enum AddressPin : uint8_t
{
  GND = 0b1001000,
  VDD = 0b1001001,
  SDA = 0b1001010,
  SCL = 0b1001011,
};

inline bool is_valid_address(const uint8_t address)
{
  // The 2 lowest pins are selectable so we need to check the upper 6 bits.
  return (address & ~0b11) == 0b1001000;
}

class ADC
{
public:
  ADC(const std::string port, const uint8_t address);
  virtual ~ADC();

  uint16_t read_config_register(void);

private:
  i2cImpl*      m_impl;
  const uint8_t m_address;
};

} // namespace ADS1115
