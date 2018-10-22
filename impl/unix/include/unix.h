#pragma once

#include "error.h"
#include <cstdint>
#include <exception>
#include <fcntl.h> //Needed for I2C port
#include <iostream>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace unix_i2c
{

using std::size_t;

class i2c
{

public:
  i2c(const void* arg);
  i2c(const char* const arg);
  i2c(uint) = delete;
  ~i2c();
  ADS1115::Error begin(const uint8_t address) const;
  ADS1115::Error write(uint8_t* data, const size_t length) const;
  ADS1115::Error read(uint8_t* data, const size_t length) const;

private:
  int m_file;
};

} // namespace unix_i2c
