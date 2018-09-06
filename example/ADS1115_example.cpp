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

  auto config_fsr = ADS1115::FullScaleRange::FSR_4_096V;
  auto config_dr  = ADS1115::DataRate::SPS_860;

  std::cout << "Setting FSR to +-" << config_fsr << std::endl;
  std::cout << "Setting DR to " << config_dr << std::endl;

  adc.set_fsr(config_fsr);
  adc.set_data_rate(config_dr);

  std::cout << "ADC Configuration" << std::endl;
  std::cout << "\tfsr             : " << adc.get_fsr() << std::endl;
  std::cout << "\tmultiplexing    : " << adc.get_multiplexing() << std::endl;
  std::cout << "\tdata rate       : " << adc.get_data_rate() << std::endl;
  std::cout << "\tconversion mode : " << adc.get_conversion_mode() << std::endl;

  std::cout << ADS1115::Multiplex::AIN0 << " = " << adc.read(ADS1115::Multiplex::AIN0) << " V" << std::endl;
  std::cout << ADS1115::Multiplex::AIN1 << " = " << adc.read(ADS1115::Multiplex::AIN1) << " V" << std::endl;
  std::cout << ADS1115::Multiplex::AIN2 << " = " << adc.read(ADS1115::Multiplex::AIN2) << " V" << std::endl;
  std::cout << ADS1115::Multiplex::AIN3 << " = " << adc.read(ADS1115::Multiplex::AIN3) << " V" << std::endl;

  std::cout << ADS1115::Multiplex::AIN0_AIN1 << " = " << adc.read(ADS1115::Multiplex::AIN0_AIN1) << " V" << std::endl;
  std::cout << ADS1115::Multiplex::AIN0_AIN3 << " = " << adc.read(ADS1115::Multiplex::AIN0_AIN3) << " V" << std::endl;
  std::cout << ADS1115::Multiplex::AIN1_AIN3 << " = " << adc.read(ADS1115::Multiplex::AIN1_AIN3) << " V" << std::endl;
  std::cout << ADS1115::Multiplex::AIN2_AIN3 << " = " << adc.read(ADS1115::Multiplex::AIN2_AIN3) << " V" << std::endl;

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
