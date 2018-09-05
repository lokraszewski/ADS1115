#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>

#include "ADS1115.h"

using std::cerr;
using std::cout;
using std::endl;
using std::exception;
using std::string;
using std::vector;

void my_sleep(unsigned long milliseconds)
{
#ifdef _WIN32
  Sleep(milliseconds); // 100 ms
#else
  usleep(milliseconds * 1000); // 100 ms
#endif
}

void print_usage()
{
  cerr << "Usage: ADS1115_example ";
  cerr << "<i2c port address> " << endl;
  cerr << "<i2c address in hex> " << endl;
}

int run(int argc, char **argv)
{

  if (argc < 2)
  {
    print_usage();
    return 0;
  }

  // Argument 1 is the serial port or enumerate flag
  // Argument 2 is the address of the chip in hex.
  string  port(argv[1]);
  uint8_t address = std::stoul(argv[2], nullptr, 16);

  if (!ADS1115::is_valid_address(address))
  {
    std::cout << "Address invalid, possible addresses include: " << std::endl;
    std::cout << "\t ADR pin to GND: 0x" << std::hex << ADS1115::AddressPin::GND << std::endl;
    std::cout << "\t ADR pin to VDD: 0x" << std::hex << ADS1115::AddressPin::VDD << std::endl;
    std::cout << "\t ADR pin to SDA: 0x" << std::hex << ADS1115::AddressPin::SDA << std::endl;
    std::cout << "\t ADR pin to SCL: 0x" << std::hex << ADS1115::AddressPin::SCL << std::endl;
    return -1;
  }

  std::cout << "Openning ADS1115 at " << port << " with address: " << address << std::endl;

  ADS1115::ADC adc(port, address);
  std::cout << std::hex << adc.read_config_register() << std::endl;

  return 0;
}

int main(int argc, char **argv)
{
  try
  {
    return run(argc, argv);
  }
  catch (exception &e)
  {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}
