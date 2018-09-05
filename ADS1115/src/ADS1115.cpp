#include "ADS1115.h"

namespace ADS1115
{
ADC::ADC(std::string port, const uint8_t address) : m_config(DEFAULT_CFG), m_impl(new i2cImpl(port)), m_address(address) {}

ADC::~ADC() { delete m_impl; }



double ADC::get_fsr_voltage(FullScaleRange fsr)
{
switch(fsr)
{
  case FSR_6_144V:
    return 6.144L;
  case FSR_4_096V:
    return 4.096L;
  case FSR_2_048V:
    return 2.048L;
  case FSR_1_024V:
    return 1.024L;
  case FSR_0_512V:
    return 0.512L;
  default:
    return 0.256L;
}
}

void ADC::reset(void) const
{
  uint8_t tx[1] = {0x06};
  m_impl->begin(0);
  m_impl->write(tx, 1);
}

uint16_t ADC::read_register(RegisterAddress reg_address) 
{

  uint8_t tx[1] = {static_cast<uint8_t>(reg_address)};
  uint8_t rx[2];

  m_impl->begin(m_address);
  m_impl->write(tx, 1);
  m_impl->read(rx, 2);

  uint16_t value = (rx[0] << 8) | rx[1];

  if(reg_address == RegisterAddress::Config)
  {
    //Update chaced values.
    m_config = value;
  }

  return value;
}

void ADC::write_register(RegisterAddress reg_address, const uint16_t value) 
{
  uint8_t tx[3] = {static_cast<uint8_t>(reg_address), static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};


  m_impl->begin(m_address);
  m_impl->write(tx, 3);

  if(reg_address == RegisterAddress::Config)
  {
    m_config = value;
  }
}

uint16_t ADC::read_config_register(void)  { return read_register(RegisterAddress::Config); }
void     ADC::write_config_register(const uint16_t cfg) 
{
  write_register(RegisterAddress::Config, cfg);
}

void ADC::set_multiplexing(const Multiplex mult) 
{
  auto cfg = read_config_register() & ~(0b111 << 12);
  cfg |= mult;
  write_config_register(cfg);
}


bool ADC::is_conversion_done(void) 
{
  auto cfg = read_config_register();
  /* if in continuous mode then just read. Otherwise wait for conversion to end.*/
  return (cfg & Config::OS) || !(cfg & Config::MODE);
}

void ADC::start_conversion(void) 
{
  auto cfg = read_config_register() | Config::OS;
  write_config_register(cfg);
}

int16_t   ADC::read_raw() 
{
  return static_cast<int16_t>(read_register(RegisterAddress::Conversion));
}

double ADC::raw_to_voltage(const int16_t raw_value )
{
  const FullScaleRange fsr = static_cast<FullScaleRange>((m_config >> 9) & 0b111); 
  const auto fsr_v = get_fsr_voltage(fsr);
  double result = raw_value *  (fsr_v / static_cast<double>(0x7FFF));

  std::cout << "FSR: " << fsr_v << std::endl;
  std::cout << "RAW: " << (int)raw_value << std::endl;
  std::cout << "RES: " << result << std::endl;

  return result;
}

double ADC::read(const Multiplex mux)  {
  //Set the correct multiplexing settings. 
  set_multiplexing(mux);

  start_conversion();
  while(!is_conversion_done());

  auto read = read_raw();

  return raw_to_voltage(read);
}

} // namespace ADS1115
