#include "ADS1115.h"

namespace ADS1115
{
ADC::ADC(std::string port, const uint8_t address) : m_config(DEFAULT_CFG), m_impl(new i2cImpl(port)), m_address(address) {}

ADC::~ADC() { delete m_impl; }

uint16_t ADC::read_register(RegisterAddress reg_address)
{
  uint8_t tx[1] = {static_cast<uint8_t>(reg_address)};
  uint8_t rx[2];

  m_impl->begin(m_address);
  m_impl->write(tx, 1);
  m_impl->read(rx, 2);

  return (rx[0] << 8) | rx[1];
}

void ADC::write_register(RegisterAddress reg_address, const uint16_t value)
{
  uint8_t tx[3] = {static_cast<uint8_t>(reg_address), static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
  m_impl->begin(m_address);
  m_impl->write(tx, 3);
}

uint16_t ADC::read_config(void) { return read_register(RegisterAddress::Config); }
void     ADC::write_config(const uint16_t cfg) { write_register(RegisterAddress::Config, cfg); }
void     ADC::write_config(void) { write_config(m_config); }
void     ADC::write_config_default(void) { write_config(DEFAULT_CFG); }

bool ADC::is_conversion_done(void)
{
  auto cfg = read_config();
  /* if in continuous mode then just read. Otherwise wait for conversion to end.*/
  return (cfg & Config::OS) || !(cfg & Config::MODE);
}

double ADC::read(const Multiplex mux)
{
  set_multiplexing(mux);
  return raw_to_voltage(read_raw());
}
int16_t ADC::read_raw()
{
  write_config(m_config | Config::OS);
  while(!is_conversion_done());
  return read_register(RegisterAddress::Conversion);
}
void ADC::set_fsr(const FullScaleRange fsr)
{
  m_config &= ~FSR_0_256V2;
  m_config |= fsr;
}
void ADC::set_multiplexing(const Multiplex mult)
{

  m_config &= ~AIN3;
  m_config |= mult;
}
void ADC::set_data_rate(const DataRate dr)
{

  m_config &= ~SPS_860;
  m_config |= dr;
}
double ADC::raw_to_voltage(const int16_t raw_value)
{

  const FullScaleRange fsr    = static_cast<FullScaleRange>((m_config & FullScaleRange::FSR_0_256V2));
  const auto           fsr_v  = get_fsr_voltage(fsr);
  double               result = raw_value * (fsr_v / static_cast<double>(0x7FFF));

  std::cout << "<";
  std::cout << " FSR(INT): " << (int)fsr;
  std::cout << " FSR: " << fsr_v;
  std::cout << " RAW: " << (int)raw_value;
  std::cout << " RES: " << result;
  std::cout << ">";

  return result;
}


} // namespace ADS1115
