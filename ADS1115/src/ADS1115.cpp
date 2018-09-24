#include "ADS1115.h"

namespace ADS1115
{

ADC::ADC(std::string port, const uint8_t address) : m_impl(new i2cImpl(port)), m_address(address), m_config(DEFAULT_CFG) {}

ADC::~ADC() { delete m_impl; }

/**
 * @author     lokraszewski
 * @date       06-Sep-2018
 * @brief      Reads a register.
 *
 * @param[in]  reg_address  The register address
 *
 * @return     Register value
 *
 * @details    Used to read the registers of the ADC.
 */
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
void     ADC::write_config(const uint16_t cfg)
{

  std::cout << std::hex << " [CFG: 0x" << cfg << "] ";

  write_register(RegisterAddress::Config, cfg);
}
void ADC::write_config(void) { write_config(m_config); }
void ADC::write_config_default(void) { write_config(DEFAULT_CFG); }

bool ADC::is_conversion_done(void)
{
  auto cfg = read_config();
  /* if in continuous mode then just read. Otherwise wait for conversion to end.*/
  return (cfg & Config::OS);
}

double ADC::read(const Multiplex mux)
{
  set_multiplexing(mux);
  return raw_to_voltage(read_raw());
}
double ADC::read(void) { return raw_to_voltage(read_raw()); }

int16_t ADC::read_raw()
{
  // Conviently we can send the configuration and the conversion request in one
  // command.
  write_config(m_config | Config::OS);

  // Once the conversion has been requested we must wait for it do be finished.
  while (!is_conversion_done())
    ;

  // The conversion register stores the latest result.
  auto r = read_register(RegisterAddress::Conversion);

  std::cout << std::hex << " [RAW: 0x" << r << "] ";
  return r;
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
void ADC::set_conversion_mode(const ConversionMode mode)
{
  if (mode)
  {
    m_config |= ConversionMode::SingleShot;
  }
  else
  {
    m_config &= ~ConversionMode::SingleShot;
  }
}

const FullScaleRange ADC::get_fsr(void) const { return static_cast<FullScaleRange>((m_config & FullScaleRange::FSR_0_256V2)); }
const Multiplex      ADC::get_multiplexing(void) const { return static_cast<Multiplex>((m_config & Multiplex::AIN3)); }
const DataRate       ADC::get_data_rate(void) const { return static_cast<DataRate>((m_config & DataRate::SPS_860)); }
const ConversionMode ADC::get_conversion_mode(void) const { return static_cast<ConversionMode>(m_config & ConversionMode::SingleShot); }

double ADC::raw_to_voltage(const int16_t raw_value)
{
  const auto fsr_v = get_fsr_voltage(get_fsr());
  return raw_value * (fsr_v / static_cast<double>(0x7FFF));
}

} // namespace ADS1115
