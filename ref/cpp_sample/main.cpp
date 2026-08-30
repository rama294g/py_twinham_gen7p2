
#include "ADS1015.h"
#include "AEAQM0802.h"
#include "BNO055.h"
#include "MCP23017.h"
#include "NAU7802.h"
#include "UART2.h"
#include "algo.h"
#include "display.h"
#include "max32625pico.h"
#include <cstdio>
#include <initializer_list>
#include <string>
#include "splash_window.h"

// #define __SCREEN__


MAX32625PICO pico(MAX32625PICO::IOH_3V3, MAX32625PICO::VIO_IOH,
                  MAX32625PICO::VIO_IOH);

// Ports
DigitalIn DIN_TACT(P0_2);
DigitalIn DIN_KEY_U(P4_7);
DigitalIn DIN_KEY_D(P0_7);
DigitalIn DIN_KEY_L(P0_6);
DigitalIn DIN_KEY_R(P0_3);
DigitalIn ENC_MTR1(P4_4);
DigitalIn ENC_MTR2(P4_5);
DigitalIn ENC_TYRE(P4_6);
DigitalOut LED_R(LED1, LED_OFF);
DigitalOut LED_G(LED2, LED_OFF);
DigitalOut LED_B(LED3, LED_OFF);
PwmOut pulse_out_left(P0_4);
PwmOut pulse_out_right(P0_5);

UART2 m_debug2(UART1_TX, UART1_RX);
BNO055 m_gyro2(UART2_TX, UART2_RX);
I2C m_i2c(I2C0_SDA, I2C0_SCL);
MCP23017 m_mcp23017(m_i2c, 0x20 << 1);
ADS1015 m_ads1015(m_i2c, 0x48 << 1);
NAU7802 m_nau7802(m_i2c, 0x2A << 1);
SSD1305 m_screen(m_i2c, 0x3C << 1, m_debug2);
AEAQM0802 m_lcd(m_i2c);

const float dt = 0.01;
unsigned char m_chip_id = 0;
float m_accx = 0;
float m_accy = 0;
float m_accz = 0;
float m_gyrox = 0;
float m_gyroy = 0;
float m_gyroz = 0;

void bno055_rx_chip_id(unsigned char _chip_id) { m_chip_id = _chip_id; }
void bno055_rx_gyro(float _accx, float _accy, float _accz, float _gyrox,
                    float _gyroy, float _gyroz) {
  m_accx = -_accy;
  m_accy = +_accz;
  m_accz = -_accx;
  m_gyrox = -_gyroy * (180.0 / M_PI);
  m_gyroy = +_gyroz * (180.0 / M_PI);
  m_gyroz = -_gyrox * (180.0 / M_PI);
}

bool is_device_connected(uint8_t address) {
  char data; // Dummy data to write
  int result = m_i2c.read(address, &data, 1);
  return result == 0;
}

void send_message(const std::string &_str, std::initializer_list<float> values,
                  const std::string &_unit, const bool _crlf) {
  std::string expression = _str + ": ";
  bool is_first = true;
  for (float value : values) {
    if (!is_first)
      expression += ", ";
    is_first = false;
    char buffer[12] = {0};
    sprintf(buffer, "%6d", (int)round(value));
    // const int v = (int)round(_val[i] * 1000);
    expression += std::string(buffer);
  }
  expression += _unit;
  if (_crlf)
    expression += "\r\n";
  m_debug2.add_uart_data((unsigned char *)expression.c_str(),
                         expression.length());
}
void send_message3(const char *_str, const int _str_len, float _val) {
  char buffer[12];
  sprintf(buffer, "%d",
          (int)round(_val * 10)); // You can change precision as needed
  // std::string expression = _str + ": " + std::string(buffer) + "\r\n";
  m_debug2.add_uart_data((unsigned char *)_str, _str_len);
  m_debug2.add_uart_data((unsigned char *)buffer, 12);
  char com[] = "\r\n";
  m_debug2.add_uart_data((unsigned char *)com, 2);
}

int main() {

  rgb::rLED = &LED_R;
  rgb::gLED = &LED_G;
  rgb::bLED = &LED_B;
  rgb(0, 0, 1).apply();

  m_debug2.init();
  m_gyro2.attach_rx_callback(&bno055_rx_gyro, &bno055_rx_chip_id);
  NVIC_SetPriority(UART0_IRQn, 31);
  pulse_out_left.period_us(100);  // 10kHz
  pulse_out_right.period_us(100); // 10kHz

  thread_sleep_for((uint32_t)100); // 100ms
  rgb(1, 0, 0).apply();
  thread_sleep_for((uint32_t)100); // 100ms
  rgb(0, 0, 1).apply();

  m_i2c.frequency(400 * 1000);
  uint32_t delay = 10;
  thread_sleep_for(delay);
  m_gyro2.tx_CHIP_ID();
  thread_sleep_for(delay);
  m_gyro2.tx_OPR_MODE();
  thread_sleep_for(delay);
  m_mcp23017.config(0xFD, 0x00, 0x00);
  thread_sleep_for(delay);
  // m_nau7802.initialize();
  m_nau7802.reset();
  thread_sleep_for(delay);
  m_nau7802.setLDO(NAU7802::E_LDOVoltage::NAU7802_2V4);
  thread_sleep_for(delay);
  m_nau7802.setGain(NAU7802::E_Gain::NAU7802_GAIN_128);
  thread_sleep_for(delay);
  m_nau7802.setRate(NAU7802::E_SampleRate::NAU7802_RATE_10SPS);
  thread_sleep_for(delay);
  m_nau7802.enable(true);
  thread_sleep_for(delay);
  m_ads1015.config(ADS1015::config_param());
  thread_sleep_for(delay);
  #ifdef __SCREEN__
  m_screen.initializeDisplay(); // Initialize the OLED display
  thread_sleep_for(delay);
  m_screen.setBuffer(0x0000, splash_window[0], 0x0080);
  m_screen.setBuffer(0x0080, splash_window[1], 0x0080);
  m_screen.setBuffer(0x0100, splash_window[2], 0x0080);
  m_screen.setBuffer(0x0180, splash_window[3], 0x0080);
  m_screen.updateDisplayAll();
  //m_screen.Horizontal_scroll_on();
  #endif
  #ifdef __LCD_AQM0802__
  m_lcd.initialize();
  thread_sleep_for(delay);
  m_lcd.clear();
  m_lcd.setCursor(0, 0);
  m_lcd.writeText("MAX32625");
  m_lcd.setCursor(0, 1);
  m_lcd.writeText("AE-AQM08");
  #endif

  thread_sleep_for(1500);
  m_gyro2.tx_CHIP_ID();
  thread_sleep_for(delay);

  unsigned char addrs[5];
  memset(addrs, 0, sizeof(addrs));
  int cnt_addrs = 0;
  for (uint8_t address = 0x08; address <= 0x77; address++) {
    if (is_device_connected(address << 1)) {
      cnt_addrs++;
      addrs[cnt_addrs - 1] = address;
      if (cnt_addrs == 5)
        break;
    }
    rgb((address >> 5) & 0x01, (address >> 6) & 0x01, (address >> 7) & 0x01)
        .apply();
    thread_sleep_for((uint32_t)1);
  }

  static bool f_changed = true;
  while (true) {
    static display disp;
    static int job_10ms_cnt = 0;
    static int job_100mS_cnt = 0;
    static int job_1s_cnt = 0;
    static int job_10s_cnt = 0;
    job_10ms_cnt++;
    job_100mS_cnt++;
    job_1s_cnt++;
    job_10s_cnt++;
    if (job_10ms_cnt >= 1000)
      job_10ms_cnt = 0;
    if (job_100mS_cnt >= 10)
      job_100mS_cnt = 0;
    if (job_1s_cnt >= 100)
      job_1s_cnt = 0;
    if (job_10s_cnt >= 1000)
      job_10s_cnt = 0;

    // Input
    m_gyro2.tx_GET_ACCGYRO();
    static int tact_drive = 0;
    static int key_up = 0;
    static int key_down = 0;
    static int key_left = 0;
    static int key_right = 0;
    static int enc_mtr = 0;
    static int enc_tyre = 0;
    static int sw_f_is_Right = 0; // 1: Right, 0: Left
    static float current_mtr = 0;
    static float v_bat = 0;
    static unsigned short adc_config = 0;
    static int load_cell = 0;
    tact_drive = (DIN_TACT.read() == 0);
    key_up = (DIN_KEY_U.read() == 0);
    key_down = (DIN_KEY_D.read() == 0);
    key_left = (DIN_KEY_L.read() == 0);
    key_right = (DIN_KEY_R.read() == 0);
    key_up = 0; // Temporary Disabled as new display does not work (2026.01.04)
    key_down = 0; // Temporary Disabled as new display does not work (2026.01.04)
    static int tact_drive_prev = 0;
    static int key_up_prev = 0;
    static int key_down_prev = 0;
    static int key_left_prev = 0;
    static int key_right_prev = 0;
    const int key_up_rise = (key_up && !key_up_prev);
    const int key_down_rise = (key_down && !key_down_prev);
    const int key_left_rise = (key_left && !key_left_prev);
    const int key_right_rise = (key_right && !key_right_prev);
    if (key_up_rise && false) {  // 2026.01.28 disabled up/down button
      disp.select_up();
      f_changed = true;
    }
    if (key_down_rise && false) {  // 2026.01.28 disabled up/down button
      disp.select_down();
      f_changed = true;
    }
    if (key_left_rise) {
      disp.param_down();
      f_changed = true;
    }
    if (key_right_rise) {
      disp.param_up();
      f_changed = true;
    }

    tact_drive_prev = tact_drive;
    key_up_prev = key_up;
    key_down_prev = key_down;
    key_left_prev = key_left;
    key_right_prev = key_right;
    // thread_sleep_for((uint32_t)10);
    enc_mtr = enc_2phase(ENC_MTR1.read() == 0, ENC_MTR2.read() == 0);
    enc_tyre = enc_1phase(ENC_TYRE.read() == 0);
    if (job_100mS_cnt == 0)
      sw_f_is_Right = (m_mcp23017.read_gpio_bit(0) == 0);
    static const float k_ain = 4.096f / 32768.f;
    if (job_100mS_cnt == 1) {
      const float ain01 = (float)m_ads1015.getAIN0_1() * k_ain;
      static const float k_current_mtr = (float)(1.0 / 0.025);
      current_mtr = ain01 * k_current_mtr;
    }
    if (job_100mS_cnt == 2) {
      const float ain2 = (float)m_ads1015.getAIN2() * k_ain;
      static const double r13 = 13000.0;
      static const double r14 = 220.0;
      static const float k_v_bat = (float)((r13 + r14) / r14 / 4.1);
      v_bat = MAX((ain2 - 1.23f) * k_v_bat, 0);
    }
    if (job_100mS_cnt == 3)
      adc_config = (unsigned short)m_ads1015.getConfig();
    if (job_100mS_cnt == 4)
      load_cell = m_nau7802.read();

    float volume = (float)disp.gain / 100;
    float rate_gain = (float)disp.response / 10;
    float gain0 = (float)tact_drive * volume;
    static float gain = 0;
    gain = ratelimitter(gain0, gain, rate_gain * dt, -rate_gain * dt);

    // State Estimation
    static const float q1 = 0.0f / (180.0f / M_PI); // 0 deg tilt
    static const float cosq1 = cos(q1);
    static const float sinq1 = sin(q1);
    const float sign_LR = (sw_f_is_Right ? -1.0f : 1.0f);
    const float new_accx = m_accx;
    const float new_accy = sign_LR * (cosq1 * m_accy + sinq1 * m_accz);
    const float new_accz = sign_LR * (cosq1 * m_accz - sinq1 * m_accy);
    const float new_gyrox = m_gyrox;
    const float new_gyroy = sign_LR * (cosq1 * m_gyroy + sinq1 * m_gyroz);
    const float new_gyroz = sign_LR * (cosq1 * m_gyroz - sinq1 * m_gyroy);
    float tilt_observe = atan2(new_accx, new_accz) * (180.0f / M_PI);
    tilt_observe += sw_f_is_Right ? -90.0 : 90.0;                     
    static float tilt_kalman = tilt_observe;
    const float tilt_priest = tilt_kalman - new_gyroy * dt;
    float tilt_delta = tilt_observe - tilt_priest;
    fitin360(tilt_delta);
    tilt_kalman = 0.01 * tilt_delta + tilt_priest;
    fitin360(tilt_kalman);
    tilt_kalman = LIM(tilt_kalman, -90.0f, 90.0f);
    const float tau = 0.01f;
    static float tilt_lpf1 = tilt_kalman;
    tilt_lpf1 = (dt * tilt_kalman + tau * tilt_lpf1) / (tau + dt);
    static float tilt_lpf2 = tilt_lpf1;
    tilt_lpf2 = (dt * tilt_lpf1 + tau * tilt_lpf2) / (tau + dt);
    float tilt_machine = - tilt_lpf2 -
                         (float)disp.neutral; // Negative Sign needed for RH machine 20250113
    fitin360(tilt_machine);

    const float _thrust = gain * LIM(tilt_machine / 30.0f, -1.0f, 1.0f);
    const float tau2 = 0.01f;
    static float thrust_lpf1 = _thrust;
    thrust_lpf1 = (dt * _thrust + tau2 * thrust_lpf1) / (tau2 + dt);
    static float thrust_lpf2 = thrust_lpf1;
    thrust_lpf2 = (dt * thrust_lpf1 + tau2 * thrust_lpf2) / (tau2 + dt);
    key_up_prev = key_up;
    key_down_prev = key_down;
    //const float duly_left = (1 + _thrust * sign_LR * 0.90) / 2;
    //const float duly_right = (1 - _thrust * sign_LR * 0.90) / 2;
    const float duly_left = (1 - thrust_lpf2 * 0.90) / 2;
    const float duly_right = (1 + thrust_lpf2 * 0.90) / 2;

    rgb::get_jet_color((thrust_lpf2 + 1.0f) * 3.0f).apply();

    if (key_up)
      rgb(1, 0, 0).apply();
    else if (key_down)
      rgb(0, 1, 0).apply();
    else if (key_left)
      rgb(0, 0, 1).apply();
    else if (key_right)
      rgb(1, 1, 0).apply();
    else if (tact_drive)
      rgb(1, 0, 1).apply();
    else if (sw_f_is_Right)
      rgb(0, 1, 1).apply();
    else
      rgb(0, 0, 0).apply();

    pulse_out_left = duly_left;
    pulse_out_right = duly_right;

    if (job_100mS_cnt == 5)
      m_mcp23017.write_gpio_bit(1, 1);

    if (f_changed && job_100mS_cnt == 6) {
      f_changed = false;
      disp.battery = v_bat;
      disp.accx = new_accx;
      disp.accy = new_accy;
      disp.accz = new_accz;
      disp.gyrox = m_gyrox;
      disp.gyroy = m_gyroy;
      disp.gyroz = m_gyroz;
      #ifdef __SCREEN__
      m_screen.clearDisplay();
      disp.update_display(m_screen);
      int ret = m_screen.updateDisplayAll();
      if (ret) {
          pulse_out_left=0.0f;
          pulse_out_right=0.0f;
          return 0;
      }
      #endif
    }
    if (job_1s_cnt == 7 || job_1s_cnt == 57) {
      {
        std::string expression = "======================================\r\n";
        m_debug2.add_uart_data((unsigned char *)expression.c_str(),
                               expression.length());
      }
      static const float k_acc = 1000.f / 9.8f;
      send_message("ACCEL",
                   {m_accx * k_acc, m_accy * k_acc, m_accz * k_acc}, " (mG)",
                   true);
      send_message("GYRO ", {m_gyrox, m_gyroy, m_gyroz}, " (deg/s)", true);
      send_message("ACCL^",
                   {new_accx * k_acc, new_accy * k_acc, new_accz * k_acc},
                   " (mG)", true);
      send_message("TILT ", {tilt_observe, tilt_kalman, tilt_machine}, " (deg)",
                   true);
      send_message("GAIN ", {(float)disp.gain}, " (%)", true);
      send_message("NTRL ", {(float)disp.neutral}, " (deg)", true);
      send_message("RPNS ", {(float)disp.response}, " (%)", true);
      send_message("THRST",
                   {_thrust * 100, duly_left * 100, duly_right * 100}, " (%)",
                   true);
      send_message("CHIP_ID", {(float)m_chip_id}, "", true);
      {
        std::string expression = "I2C_ADDRESS: ";
        for (int j = 0; j < 5; j++) {
          if (j != 0)
            expression += ", ";
          char buffer[5] = {0};
          sprintf(buffer, "0x%02X", addrs[j]);
          expression += std::string(buffer);
        }
        expression += "\r\n";
        m_debug2.add_uart_data((unsigned char *)expression.c_str(),
                               expression.length());
      }
      send_message("BATTERY", {(float)v_bat * 1000}, " (mV)", true);
      send_message("CUR_MTR", {(float)current_mtr * 1000}, " (mA)", true);
      send_message("ENCodER", {(float)enc_mtr, (float)enc_tyre}, "", true);
      send_message("LOADCEL",
                   {(float)load_cell / 8388608.f * 1000}, " (m)", true);
      unsigned char zero[] = {0};
      m_debug2.add_uart_data(zero, 1);
    }

    thread_sleep_for((uint32_t)(dt * 1000));
  }
}
