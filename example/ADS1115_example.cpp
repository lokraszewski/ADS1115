#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>

#include "ADS1115.h"

using endl;
using std::cerr;
using std::cout;
using std::exception;
using std::string;
using std::vector;

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
    cout << "Address invalid, possible addresses include: " << endl;
    cout << "\t ADR pin to GND: 0x" << std::hex << ADS1115::AddressPin::GND << endl;
    cout << "\t ADR pin to VDD: 0x" << std::hex << ADS1115::AddressPin::VDD << endl;
    cout << "\t ADR pin to SDA: 0x" << std::hex << ADS1115::AddressPin::SDA << endl;
    cout << "\t ADR pin to SCL: 0x" << std::hex << ADS1115::AddressPin::SCL << endl;
    return -1;
  }

  cout << "Openning ADS1115 at " << port << " with address: " << address << endl;

  ADS1115::ADC adc(port, address);

  auto config_fsr = ADS1115::FullScaleRange::FSR_2_048V;
  auto config_dr  = ADS1115::DataRate::SPS_860;

  cout << "Setting FSR to +-" << config_fsr << endl;
  cout << "Setting DR to " << config_dr << endl;

  adc.set_fsr(config_fsr);
  adc.set_data_rate(config_dr);

  cout << "ADC Configuration" << endl;
  cout << "\tfsr             : " << adc.get_fsr() << endl;
  cout << "\tmultiplexing    : " << adc.get_multiplexing() << endl;
  cout << "\tdata rate       : " << adc.get_data_rate() << endl;
  cout << "\tconversion mode : " << adc.get_conversion_mode() << endl;

  cout << ADS1115::Multiplex::AIN0 << " = " << adc.read(ADS1115::Multiplex::AIN0) << " V" << endl;
  cout << ADS1115::Multiplex::AIN1 << " = " << adc.read(ADS1115::Multiplex::AIN1) << " V" << endl;
  cout << ADS1115::Multiplex::AIN2 << " = " << adc.read(ADS1115::Multiplex::AIN2) << " V" << endl;
  cout << ADS1115::Multiplex::AIN3 << " = " << adc.read(ADS1115::Multiplex::AIN3) << " V" << endl;

  cout << ADS1115::Multiplex::AIN0_AIN1 << " = " << adc.read(ADS1115::Multiplex::AIN0_AIN1) << " V" << endl;
  cout << ADS1115::Multiplex::AIN0_AIN3 << " = " << adc.read(ADS1115::Multiplex::AIN0_AIN3) << " V" << endl;
  cout << ADS1115::Multiplex::AIN1_AIN3 << " = " << adc.read(ADS1115::Multiplex::AIN1_AIN3) << " V" << endl;
  cout << ADS1115::Multiplex::AIN2_AIN3 << " = " << adc.read(ADS1115::Multiplex::AIN2_AIN3) << " V" << endl;

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
