
#include "mbed.h"

class MCP23017 {
private:
  // REGISTER ADDRESSES
  static constexpr const char _IODIR_ = 0x00;
  static constexpr const char _IPOL_ = 0x02;
   static constexpr const char _GPINTEN_ = 0x04;
   static constexpr const char _DEFVAL_ = 0x06;
   static constexpr const char _INTCON_ = 0x08;
   static constexpr const char _IOCON_ = 0x0A;
  static constexpr const char _GPPU_ = 0x0C;
   static constexpr const char _INTF_ = 0x0E;
   static constexpr const char _INTCAP_ = 0x10;
  static constexpr const char _GPIO_ = 0x12;
  static constexpr const char _OLAT_ = 0x14;

  // static constexpr const char _I2C_BASE_ADDRESS_ = 0x40;

  static constexpr const char _DIR_OUTPUT_ = 0;
  static constexpr const char _DIR_INPUT_ = 1;

private:
  int i;
  I2C &m_i2c;
  int m_8bit_addr;
  void (*m_bno055_rx_gyro)(float, float, float, float, float, float);
  void (*m_bno055_rx_chip_id)(unsigned char);
  unsigned short shadow_GPIO; // General-Purpose Input/Output
  unsigned short shadow_IODIR;
  unsigned short shadow_GPPU; // General-Purpose Pull-Up Resistors
  unsigned short shadow_IPOL; // Cached copies of the register values

public:
  // Constructor
  MCP23017(I2C &_i2c, const int _8bit_addr)
      : i(0), m_i2c(_i2c), m_8bit_addr(_8bit_addr), m_bno055_rx_gyro(NULL),
        m_bno055_rx_chip_id(NULL) {
    reset(); // initialise chip to power-on condition
  };

  void reset() {
    // First make sure that the device is in BANK=0 mode
    writeRegister(0x05, (unsigned char)0x00);
    // set all registers to zero (last of 10 registers is _OLAT_)
    for (int reg_addr = 0x00; reg_addr < 0x16; reg_addr ++)
      writeRegister(reg_addr, (unsigned char)0x00);
    // set direction registers to inputs
    writeRegister(_IODIR_, (unsigned short)0xFFFF);

    shadow_IODIR = 0xFFFF;
    shadow_GPIO = 0;
    shadow_GPPU = 0;
    shadow_IPOL = 0;
  }

  void write_gpio_bit(int value, int bit_number) {
    if (value == 0)
      shadow_GPIO &= ~(1 << bit_number);
    else
      shadow_GPIO |= 1 << bit_number;
    writeRegister(_GPIO_, (unsigned short)shadow_GPIO);
  }

  void write_gpio_mask(unsigned short data, unsigned short mask) {
    shadow_GPIO = (shadow_GPIO & ~mask) | data;
    writeRegister(_GPIO_, (unsigned short)shadow_GPIO);
  }

  int read_gpio_bit(int bit_number) {
    shadow_GPIO = readRegister(_GPIO_);
    return ((shadow_GPIO >> bit_number) & 0x0001);
  }

  int read_gpio_mask(unsigned short mask) {
    shadow_GPIO = readRegister(_GPIO_);
    return (shadow_GPIO & mask);
  }

  void config(unsigned short dir_config, unsigned short pullup_config,
              unsigned short polarity_config) {
    shadow_IODIR = dir_config;
    writeRegister(_IODIR_, (unsigned short)shadow_IODIR);
    shadow_GPPU = pullup_config;
    writeRegister(_GPPU_, (unsigned short)shadow_GPPU);
    shadow_IPOL = polarity_config;
    writeRegister(_IPOL_, (unsigned short)shadow_IPOL);
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
    // tmp_data.value16 = data;
    // buffer[1] = tmp_data.value8[0];
    // buffer[2] = tmp_data.value8[1];
    m_i2c.write(m_8bit_addr, buffer, 3);
  }

  int readRegister(const int regAddress) {
    char buffer[2];
    buffer[0] = (char)regAddress;
    m_i2c.write(m_8bit_addr, buffer, 1);
    m_i2c.read(m_8bit_addr, buffer, 2);
    return ((int)(buffer[0] + (buffer[1] << 8)));
  }

  int digitalRead(int pin) {
    shadow_GPIO = readRegister(_GPIO_);
    if (shadow_GPIO & (1 << pin)) {
      return 1;
    } else {
      return 0;
    }
  }

  void digitalWrite(int pin, int val) {
    // If this pin is an INPUT pin, a write here will
    // enable the internal pullup
    // otherwise, it will set the OUTPUT voltage
    // as appropriate.
    bool isOutput = !(shadow_IODIR & 1 << pin);

    if (isOutput) {
      // This is an output pin so just write the value
      if (val)
        shadow_GPIO |= 1 << pin;
      else
        shadow_GPIO &= ~(1 << pin);
      writeRegister(_GPIO_, (unsigned short)shadow_GPIO);
    } else {
      // This is an input pin, so we need to enable the pullup
      if (val) {
        shadow_GPPU |= 1 << pin;
      } else {
        shadow_GPPU &= ~(1 << pin);
      }
      writeRegister(_GPPU_, (unsigned short)shadow_GPPU);
    }
  }

  unsigned short digitalWordRead() {
    shadow_GPIO = readRegister(_GPIO_);
    return shadow_GPIO;
  }

  void digitalWordWrite(unsigned short w) {
    shadow_GPIO = w;
    writeRegister(_GPIO_, (unsigned short)shadow_GPIO);
  }

  void inputPolarityMask(unsigned short mask) { writeRegister(_IPOL_, mask); }

  void inputOutputMask(unsigned short mask) {
    shadow_IODIR = mask;
    writeRegister(_IODIR_, (unsigned short)shadow_IODIR);
  }

  void internalPullupMask(unsigned short mask) {
    shadow_GPPU = mask;
    writeRegister(_GPPU_, (unsigned short)shadow_GPPU);
  }
};
