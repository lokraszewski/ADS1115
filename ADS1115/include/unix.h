#pragma once

#include <cstdint>
#include <fcntl.h> //Needed for I2C port
#include <iostream>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace ADS1115
{

using std::size_t;

class i2cImpl
{

public:
  i2cImpl(std::string path)
  {
    if ((m_file = open(path.c_str(), O_RDWR)) < 0)
    {
      throw m_file;
    }
  }
  virtual ~i2cImpl()
  {
    if (m_file > 0)
      close(m_file);
  }
  void begin(uint8_t address) const;
  void write(uint8_t* data, size_t length) const;
  void read(uint8_t* data, size_t length) const;

private:
  int m_file;
};

} // namespace ADS1115
