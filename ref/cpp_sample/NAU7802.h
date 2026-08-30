
#include "mbed.h"

class NAU7802 {
private:
  // static constexpr const char _STATUS_ = 0x00;    // Status Register
  // static constexpr const char _CONTROL_ = 0x01;   // Control Register
  // static constexpr const char _MSB_ = 0x03;       // MSB of data register
  // static constexpr const char _LSB_ = 0x04;       // LSB of data register
  // static constexpr const char _PGA_ = 0x05;       // PGA (Gain) Control
  // Register static constexpr const char _CALIBRATE_ = 0x06; // Calibration
  // Register

  // REGISTER ADDRESSES
  static constexpr const char _PU_CTRL_ = 0x00; // Power control register
  static constexpr const char _CTRL1_ = 0x01;   // Control/config register #1
  static constexpr const char _CTRL2_ = 0x02;   // Control/config register #2
  static constexpr const char _ADCO_B2_ = 0x12; // ADC ouput LSB
  static constexpr const char _ADC_ = 0x15;     // ADC / chopper control
  static constexpr const char _PGA_ = 0x1B;     // PGA control
  static constexpr const char _POWER_ = 0x1C;   // power control
  static constexpr const char _REVISION_ID_ = 0x1F; // Chip revision ID

public:
  enum class E_LDOVoltage : unsigned char {
    NAU7802_4V5,
    NAU7802_4V2,
    NAU7802_3V9,
    NAU7802_3V6,
    NAU7802_3V3,
    NAU7802_3V0,
    NAU7802_2V7,
    NAU7802_2V4,
    NAU7802_EXTERNAL,
  };

  enum class E_Gain : unsigned char {
    NAU7802_GAIN_1,
    NAU7802_GAIN_2,
    NAU7802_GAIN_4,
    NAU7802_GAIN_8,
    NAU7802_GAIN_16,
    NAU7802_GAIN_32,
    NAU7802_GAIN_64,
    NAU7802_GAIN_128,
  };

  enum class E_SampleRate : unsigned char {
    NAU7802_RATE_10SPS = 0,
    NAU7802_RATE_20SPS = 1,
    NAU7802_RATE_40SPS = 2,
    NAU7802_RATE_80SPS = 3,
    NAU7802_RATE_320SPS = 7,
  };

  enum class E_Calibration : unsigned char {
    NAU7802_CALMOD_INTERNAL = 0,
    NAU7802_CALMOD_OFFSET = 2,
    NAU7802_CALMOD_GAIN = 3,
  };

private:
  int i;
  I2C &m_i2c;
  int m_8bit_addr;

public:
  // Constructor
  NAU7802(I2C &_i2c, const int _8bit_addr)
      : i(0), m_i2c(_i2c), m_8bit_addr(_8bit_addr){};

  void initialize() {

    // disable ADC chopper clock
    write_gpio_mask(_ADC_, 0x3, 0x18);

    // use low ESR caps
    write_gpio_bit(_PGA_, 0, 6);

    // PGA stabilizer cap on output
    write_gpio_bit(_POWER_, 1, 7);
  }

  void enable(bool flag) {
    if (!flag) {
      write_gpio_bit(_PU_CTRL_, 0, 2); // pu_analog
      write_gpio_bit(_PU_CTRL_, 0, 1); // pu_digital
    } else {
      write_gpio_bit(_PU_CTRL_, 1, 2); // pu_analog
      write_gpio_bit(_PU_CTRL_, 1, 1); // pu_digital
      // RDY: Analog part wakeup stable plus Data Ready after exiting power-down
      // mode 600ms
      thread_sleep_for((uint32_t)600);
      write_gpio_bit(_PU_CTRL_, 1, 4); // pu_start
    }
  }

  bool available(void) {
    return read_gpio_bit(_PU_CTRL_, 5); // conv_ready
  }

  int read() {
    return readRegister_32(_ADCO_B2_); // MSBFIRST
  }

  void reset() {
    write_gpio_bit(_PU_CTRL_, 1, 0); // reg_reset
    thread_sleep_for((uint32_t)10);
    write_gpio_bit(_PU_CTRL_, 0, 0); // reg_reset
    write_gpio_bit(_PU_CTRL_, 1, 1); // pu_digital
  }

  void setLDO(E_LDOVoltage voltage) {
    if (voltage == E_LDOVoltage::NAU7802_EXTERNAL) {
      // special case!
      write_gpio_bit(_PU_CTRL_, 0, 7);
    } else {
      // internal LDO
      write_gpio_bit(_PU_CTRL_, 1, 7);
      write_gpio_mask(_CTRL1_, (unsigned short)voltage, 0x38);
    }
  }

  void setGain(E_Gain gain) {
    write_gpio_mask(_CTRL1_, (unsigned short)gain, 0x07);
  }

  void setRate(E_SampleRate rate) {
    write_gpio_mask(_CTRL2_, (unsigned short)rate, 0x70);
  }

  void calibrate(E_Calibration mode) {
    write_gpio_mask(_CTRL2_, (unsigned short)mode, 0x03);
    write_gpio_bit(_CTRL2_, (unsigned short)1, 2);
  }

private:
  void write_gpio_bit(const int regAddress, int value, int bit_number) {
    unsigned short reg_value = readRegister(regAddress);
    if (value == 0)
      reg_value &= ~(1 << bit_number);
    else
      reg_value |= 1 << bit_number;
    writeRegister(regAddress, (unsigned short)reg_value);
  }

  void write_gpio_mask(const int regAddress, unsigned short data,
                       unsigned short mask) {
    unsigned short reg_value = readRegister(regAddress);
    reg_value = (reg_value & ~mask) | data;
    writeRegister(regAddress, (unsigned short)reg_value);
  }

  int read_gpio_bit(const int regAddress, int bit_number) {
    unsigned short reg_value = readRegister(regAddress);
    return ((reg_value >> bit_number) & 0x0001);
  }

  int read_gpio_mask(const int regAddress, unsigned short mask) {
    unsigned short reg_value = readRegister(regAddress);
    return (reg_value & mask);
  }

  void writeRegister(const int regAddress, const unsigned char data) {
    char buffer[2];
    buffer[0] = (char)regAddress;
    buffer[1] = (char)data;
    m_i2c.write(m_8bit_addr, buffer, 2);
  }

  void writeRegister(const int regAddress, const unsigned short data) {
    char buffer[3];
    buffer[0] = (char)regAddress;
    memcpy(buffer + 1, &data, sizeof(data));
    m_i2c.write(m_8bit_addr, buffer, 3);
  }

  void swap_16(char* _v){
      char buf = _v[0];
      _v[0] = _v[1];
      _v[1] = buf;
  }
  void swap_32(char* _v){
      char buf = _v[0];
      _v[0] = _v[3];
      _v[3] = buf;
      buf = _v[1];
      _v[1] = _v[2];
      _v[2] = buf;
  }
  short readRegister(const int regAddress) {
    char buffer[2];
    buffer[0] = (char)regAddress;
    m_i2c.write(m_8bit_addr, buffer, 1);
    m_i2c.read(m_8bit_addr, buffer, 2);
    short ret;
    memcpy(&ret, buffer, sizeof(buffer));
    return ret;
  }
  int readRegister_32(const int regAddress) {
    char buffer[4];
    buffer[0] = (char)regAddress;
    m_i2c.write(m_8bit_addr, buffer, 1);
    m_i2c.read(m_8bit_addr, buffer + 1, 3);
    buffer[0] = 0;
    swap_32(buffer);
    int ret;
    memcpy(&ret, buffer, sizeof(buffer));
    // extend sign bit
    if (ret & 0x800000) {
      ret |= 0xFF000000;
    }
    return ret;
  }
};
