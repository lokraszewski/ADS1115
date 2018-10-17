/*
 * @Author: Lukasz
 * @Date:   05-09-2018
 * @Last Modified by:   Lukasz
 * @Last Modified time: 17-10-2018
 */

#include "unix.h"
#include <exception>

namespace unix_i2c
{
i2c::i2c(const void* arg)
{
  if ((m_file = open(static_cast<const char*>(arg), O_RDWR)) < 0)
  {
    throw std::runtime_error("I2C Cannot open port ");
  }
}

i2c::i2c(const char* const arg)
{
  if ((m_file = open(arg, O_RDWR)) < 0)
  {
    throw std::runtime_error("I2C Cannot open port ");
  }
}
i2c::~i2c()
{
  if (m_file > 0)
    close(m_file);
}

void i2c::begin(const uint8_t address) const
{
  if (ioctl(m_file, I2C_SLAVE, address) < 0)
  {
    throw std::runtime_error("Failed to acquire bus access and/or talk to slave.");
  }
}

void i2c::write(uint8_t* data, const size_t length) const
{

  if (::write(m_file, data, length) != static_cast<int>(length))
  {
    throw std::runtime_error("Failed to write to the i2c bus.");
  }
}
void i2c::read(uint8_t* data, const size_t length) const
{

  if (::read(m_file, data, length) != static_cast<int>(length))
  {
    throw std::runtime_error("Failed to read from the i2c bus.");
  }
}
} // namespace unix_i2c
