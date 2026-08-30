
#include "mbed.h"

class ADS1015 {
private:
  // REGISTER ADDRESSES
  static constexpr const char _CONV_ = 0x00;   // Conversion register
  static constexpr const char _CONFIG_ = 0x01; // Config register
  static constexpr const char _TH_LO_ = 0x02;  // Lo_thresh register
  static constexpr const char _TH_HI_ = 0x03;  // Hi_thresh register

private:
  int i;
  I2C &m_i2c;
  int m_8bit_addr;

public:
  // Constructor
  ADS1015(I2C &_i2c, const int _8bit_addr)
      : i(0), m_i2c(_i2c), m_8bit_addr(_8bit_addr){};

public:
  struct config_param {
    // OS: Operational status/single-shot conversion start
    // 0 : No effect
    // 1 : Begin a single conversion (when in power-down mode)
    unsigned short OS{0x00};
    // MUX[2:0]: Input multiplexer configuration
    // 000 : AINP = AIN0 and AINN = AIN1
    // 110 : AINP = AIN2 and AINN = GND
    unsigned short MUX{0x00}; // 0x06
    // PGA[2:0]: Programmable gain amplifier configuration
    // 001 : FS = ±4.096V
    // 010 : FS = ±2.048V (default)
    unsigned short PGA{0x01};
    // MODE: Device operating mode
    // 0 : Continuous conversion mode
    // 1 : Power-down single-shot mode (default)
    unsigned short MODE{0x01};
    // DR[2:0]: Data rate
    // 100 : 1600SPS (default)
    // 101 : 2400SPS
    // 110 : 3300SPS
    unsigned short DR{0x04};
    // COMP_MODE: Comparator mode
    // 0 : Traditional comparator with hysteresis (default)
    unsigned short COMP_MODE{0x00};
    // COMP_POL: Comparator polarity
    // 0 : Active low (default)
    unsigned short COMP_POL{0x00};
    // COMP_LAT: Latching comparator
    // 0 : Non-latching comparator (default)
    unsigned short COMP_LAT{0x00};
    // COMP_QUE: Comparator queue and disable
    // 11 : Disable comparator (default)
    unsigned short COMP_QUE{0x03};

    unsigned short get_full_register_value() const {
      return ((OS & 0x01) << 15) | ((MUX & 0x07) << 12) | ((PGA & 0x07) << 9) |
             ((MODE & 0x01) << 8) | ((DR & 0x07) << 5) |
             ((COMP_MODE & 0x01) << 4) | ((COMP_POL & 0x01) << 3) |
             ((COMP_LAT & 0x01) << 2) | ((COMP_QUE & 0x03) << 0);
    }
  };

  void config(const config_param &_c) {
    // unsigned short CONFIG = ((OS & 0x01) << 15) | ((MUX & 0x07) << 12) |
    //                         ((PGA & 0x07) << 9) | ((MODE & 0x01) << 8) |
    //                         ((DR & 0x07) << 5) | ((COMP_MODE & 0x01) << 4) |
    //                         ((COMP_POL & 0x01) << 3) |
    //                         ((COMP_LAT & 0x01) << 2) | ((COMP_QUE & 0x03) <<
    //                         0);
    writeRegister_16((int)_CONFIG_, _c.get_full_register_value());
  }

  void writeRegister_8(const int regAddress, const unsigned char data) {
    char buffer[2];
    buffer[0] = (char)regAddress;
    buffer[1] = (char)data;
    m_i2c.write(m_8bit_addr, buffer, 2);
  }

  void writeRegister_16(const int regAddress, const unsigned short data) {
    char buffer[3];
    buffer[0] = (char)regAddress;
    memcpy(buffer + 1, &data, sizeof(data));
    swap_16(buffer + 1);
    m_i2c.write(m_8bit_addr, buffer, 3);
  }

  void swap_16(char *_v) {
    char buf = _v[0];
    _v[0] = _v[1];
    _v[1] = buf;
  }
  short readRegister_16(const int regAddress) {
    char buffer[2];
    buffer[0] = (char)regAddress;
    m_i2c.write(m_8bit_addr, buffer, 1);
    m_i2c.read(m_8bit_addr, buffer, 2);
    swap_16(buffer);
    short ret; // = buffer[] << 8 + buffer[];
    memcpy(&ret, buffer, sizeof(buffer));
    return ret;
  }

public:
  short getAIN0_1() {
    config_param c;
    c.OS = 1;   // 1 : Begin a single conversion (when in power-down mode)
    c.MUX = 0;  // 000 : AINP = AIN0 and AINN = AIN1
    c.PGA = 1;  // 001 : FS = ±4.096V
    c.MODE = 1; // 1 : Power-down single-shot mode (default)
    c.DR = 6;   // 110 : 3300SPS
    config(c);
    thread_sleep_for((uint32_t)1);
    return readRegister_16(_CONV_);
  }
//   short getAIN0() {
//     config(1, 4, 1, 1, 4);
//     return readRegister_16(_CONV_);
//   }
//   short getAIN1() {
//     config(1, 5, 1, 1, 4);
//     return readRegister_16(_CONV_);
//   }
  short getAIN2() {
    config_param c;
    c.OS = 1;   // 1 : Begin a single conversion (when in power-down mode)
    c.MUX = 6;  // 110 : AINP = AIN2 and AINN = GND
    c.PGA = 1;  // 001 : FS = ±4.096V
    c.MODE = 1; // 1 : Power-down single-shot mode (default)
    c.DR = 6;   // 110 : 3300SPS
    config(c);
    thread_sleep_for((uint32_t)1);
    return readRegister_16(_CONV_);
  }
  //void setAIN2_config() { config(1, 6, 1, 0, 4); }
  //short getAIN() { return readRegister_16(_CONV_); }
  //short getAIN3() {
  //  config(1, 7, 1, 1, 4);
  //  return readRegister_16(_CONV_);
  //}
  short getConfig() { return readRegister_16(_CONFIG_); }
};
