/*
 * @Author: Lukasz
 * @Date:   05-09-2018
 * @Last Modified by:   Lukasz
 * @Last Modified time: 05-09-2018
 */

#include "unix.h"
namespace ADS1115
{

void i2cImpl::begin(uint8_t address) const
{
  if (ioctl(m_file, I2C_SLAVE, address) < 0)
  {
    printf("Failed to acquire bus access and/or talk to slave.\n");
    throw;
  }
}

void i2cImpl::write(uint8_t* data, size_t length) const
{

  if (::write(m_file, data, length) != length) // write() returns the number of bytes actually written, if it doesn't match then an error
  {
    /* ERROR HANDLING: i2c transaction failed */
    printf("Failed to write to the i2c bus.\n");
    throw;
  }
}
void i2cImpl::read(uint8_t* data, size_t length) const
{

  if (::read(m_file, data, length) != length) // read() returns the number of bytes actually read, if it doesn't match then an error
                                              // occurred (e.g. no response from the device)
  {
    printf("Failed to read from the i2c bus.\n");
    throw;
  }
}
} // namespace ADS1115
