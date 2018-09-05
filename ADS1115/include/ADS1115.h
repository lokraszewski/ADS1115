
#include <exception>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef __linux__
#include "unix.h" // i2cImpl
#else
#error "No imeplementation found. "
#endif

#pragma once

namespace ADS1115
{

enum RegisterAddress : uint8_t
{
  Conversion = 0b00,
  Config     = 0b01,
  Lo_thresh  = 0b10,
  Hi_thresh  = 0b11,
};

enum AddressPin : uint8_t
{
  GND = 0b1001000,
  VDD = 0b1001001,
  SDA = 0b1001010,
  SCL = 0b1001011,
};

enum FullScaleRange : uint16_t
{
  FSR_6_144V  = (0b000) << 9, // : FSR = ±6.144 V(1)
  FSR_4_096V  = (0b001) << 9, // : FSR = ±4.096 V(1)
  FSR_2_048V  = (0b010) << 9, // : FSR = ±2.048 V (default)
  FSR_1_024V  = (0b011) << 9, // : FSR = ±1.024 V
  FSR_0_512V  = (0b100) << 9, // : FSR = ±0.512 V
  FSR_0_256V  = (0b101) << 9, // : FSR = ±0.256 V
  FSR_0_256V1 = (0b110) << 9, // : FSR = ±0.256 V
  FSR_0_256V2 = (0b111) << 9, // : FSR = ±0.256 V
};

enum Multiplex : uint16_t
{
  AIN0_AIN1 = (0b000) << 12, // : AINP = AIN0 and AINN = AIN1 (default)
  AIN0_AIN3 = (0b001) << 12, // : AINP = AIN0 and AINN = AIN3
  AIN1_AIN3 = (0b010) << 12, // : AINP = AIN1 and AINN = AIN3
  AIN2_AIN3 = (0b011) << 12, // : AINP = AIN2 and AINN = AIN3
  AIN0      = (0b100) << 12, // : AINP = AIN0 and AINN = GND
  AIN1      = (0b101) << 12, // : AINP = AIN1 and AINN = GND
  AIN2      = (0b110) << 12, // : AINP = AIN2 and AINN = GND
  AIN3      = (0b111) << 12, // : AINP = AIN3 and AINN = GND
};

enum DataRate : uint16_t
{
  SPS_8   = (0b000) << 5, // : 8 SPS
  SPS_16  = (0b001) << 5, // : 16 SPS
  SPS_32  = (0b010) << 5, // : 32 SPS
  SPS_64  = (0b011) << 5, // : 64 SPS
  SPS_128 = (0b100) << 5, // : 128 SPS (default)
  SPS_250 = (0b101) << 5, // : 250 SPS
  SPS_475 = (0b110) << 5, // : 475 SPS
  SPS_860 = (0b111) << 5, // : 860 SPS
};

/**
@author     lokraszewski
@date       05-Sep-2018
@brief      Comparator queue

@details    These bits perform two functions. When set to 11, the comparator is
            disabled and the ALERT/RDY pin is set to a high-impedance state.
            When set to any other value, the ALERT/RDY pin and the comparator
            function are enabled, and the set value determines the number of
            successive conversions exceeding the upper or lower threshold
            required before asserting the ALERT/RDY pin. These bits serve no
            function on the ADS1113.
*/
enum ComparatorQueue : uint16_t
{
  ONE_CONVERSION   = 0b00, //! Assert after one conversion
  TWO_CONVERSIONS  = 0b01, //! Assert after two conversions
  FOUR_CONVERSIONS = 0b10, //! Assert after four conversions
  DISABLE          = 0b11, //! Disable comparator and set ALERT/RDY pin to high-impedance (default)
};

enum Config : uint16_t
{
  OS = (1 << 15),                 /* When writing: 0 : No effect 1 : Start a single conversion (when in
                                   power-down state).  When reading: 0 : Device is currently
                                   performing a conversion 1 : Device is not currently performing a
                                   conversion */
  MODE = (1 << 10),               /* Device operating mode This bit controls the operating mode. 0 :
                                   Continuous-conversion mode 1 : Single-shot mode or power-down state
                                   (default) */
  COMPARATOR_MODE = (1 << 4),     /* Comparator mode (ADS1114 and ADS1115 only) This bit configures the
                                  comparator operating mode. This bit serves no function on the
                                  ADS1113. 0 : Traditional comparator (default) 1 : Window comparator */
  COMPARATOR_POL = (1 << 3),      /* Comparator polarity(ADS1114 and ADS1115 only) This bit controls the
                                  polarity of the ALERT pin.This bit serves no function on the
                                  ADS1113. 0 : Active low(default) */
  COMPARATOR_LATCHING = (1 << 2), /* Latching comparator (ADS1114 and ADS1115 only) This bit controls
                                   * whether the ALERT/RDY pin latches after being asserted or clears
                                   * after conversions are within the margin of the upper and lower
                                   * threshold values. This bit serves no function on the ADS1113. 0 :
                                   * Nonlatching comparator . The ALERT/RDY pin does not latch when
                                   * asserted (default). 1 : Latching comparator. The asserted
                                   * ALERT/RDY pin remains latched until conversion data are read by
                                   * the master or an appropriate SMBus alert response is sent by the
                                   * master. The device responds with its address, and it is the
                                   * lowest address currently asserting the ALERT/RDY bus line.
                                   */
};

inline bool is_valid_address(const uint8_t address)
{
  // The 2 lowest pins are selectable so we need to check the upper 6 bits.
  return (address & ~0b11) == 0b1001000;
}

inline double get_fsr_voltage(const FullScaleRange fsr)
{
  switch (fsr)
  {
  case FSR_6_144V: return 6.144L;
  case FSR_4_096V: return 4.096L;
  case FSR_2_048V: return 2.048L;
  case FSR_1_024V: return 1.024L;
  case FSR_0_512V: return 0.512L;
  default: return 0.256L;
  }
}

class ADC
{
public:
  ADC(const std::string port, const uint8_t address);
  virtual ~ADC();

  uint16_t read_register(RegisterAddress reg_address);
  void     write_register(RegisterAddress reg_address, const uint16_t value);
  uint16_t read_config(void);
  void     write_config(const uint16_t cfg);
  void     write_config(void);
  void     write_config_default(void);
  bool     is_conversion_done(void);
  double   read(const Multiplex mux);
  int16_t  read_raw();

  void set_fsr(const FullScaleRange fsr);
  void set_multiplexing(const Multiplex mult);
  void set_data_rate(const DataRate dr);

private:
  static constexpr auto DEFAULT_CFG = 0x0583;
  static constexpr auto BIT_NUM     = 15;
  const uint8_t         m_address;
  uint16_t              m_config; /* Packed structure of the configuration. */
  i2cImpl* const        m_impl;

  double raw_to_voltage(const int16_t raw_value);
};

} // namespace ADS1115
