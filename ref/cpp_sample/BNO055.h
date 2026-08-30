
#include "mbed.h"

class BNO055 {
private:
  int i;
  UnbufferedSerial mbno055_uart; // UART for bno055
  int m_uart_data_len;
  unsigned char m_uart_data[200];
  void (*m_bno055_rx_gyro)(float, float, float, float, float, float);
  void (*m_bno055_rx_chip_id)(unsigned char);

public:
  // Constructor
  BNO055(const PinName _tx, const PinName _rx)
      : i(0), mbno055_uart(_tx, _rx), m_uart_data_len(0), m_uart_data{}, m_bno055_rx_gyro(NULL), m_bno055_rx_chip_id(NULL) {
    mbno055_uart.baud(115200);
    mbno055_uart.format(
        /* bits */ 8,
        /* parity */ SerialBase::None,
        /* stop bit */ 1);
    mbno055_uart.attach(callback(this, &BNO055::bno055_rx), SerialBase::RxIrq);
  };

  void attach_rx_callback(void (*_bno055_rx_gyro)(float, float, float, float, float, float), void (*_bno055_rx_chip_id)(unsigned char)) {
    m_bno055_rx_gyro = _bno055_rx_gyro;
    m_bno055_rx_chip_id = _bno055_rx_chip_id;
  }

  void set_baudrate(int _baudrate) { mbno055_uart.baud(_baudrate); }


  void tx_CR() {
    unsigned char uart_data[] = {0x0D}; // <CR>
    add_uart_data(uart_data, sizeof(uart_data));
  }
  void tx_CHIP_ID() {
    unsigned char uart_data[] = {0xAA, 0x01, 0x00, 0x01};
    add_uart_data(uart_data, sizeof(uart_data));
  }
  void tx_OPR_MODE() {
    unsigned char uart_data[] = {0xAA, 0x00, 0x3D, 0x01, 0x05}; // change to "ACCGYRO" mode
    add_uart_data(uart_data, sizeof(uart_data));
  }
  void tx_OPR_MODE2() {
    char three = 3;
    mbno055_uart.write(&three, 1);
  }
  void tx_GET_ACCGYRO() {
    unsigned char uart_data[] = {0xAA, 0x01, 0x08, 0x12};
    add_uart_data(uart_data, sizeof(uart_data));
  }

  void memsft(unsigned char _data[], int _data_len, int _shift_to_left) {
    unsigned char buf[200];
    if (_shift_to_left > 0) {
      memcpy(buf, _data + _shift_to_left, _data_len - _shift_to_left);
      memset(_data, 0, _data_len);
      memcpy(_data, buf, _data_len - _shift_to_left);
    } else if (_shift_to_left < 0) {
      memcpy(buf, _data, _data_len + _shift_to_left);
      memset(_data, 0, _data_len);
      memcpy(_data - _shift_to_left, buf, _data_len + _shift_to_left);
    }
  }
  unsigned char cue_uartdata(unsigned char _data[], int &_data_len) {
    int i;
    for (i = 0; i < _data_len; i++) {
      if (_data[i] == 0xBB)
        break;
    }
    if (i < _data_len) {
      memsft(_data, _data_len, i);
      _data_len -= i;
      return true;
    } else {
      _data_len = 0;
      return false;
    }
  }
  unsigned char extract_packet(unsigned char _data[], int &_data_len) {
    unsigned char f_cue = cue_uartdata(_data, _data_len);
    if (!f_cue)
      return false;
    if (_data_len < 3)
      return false;
    int packet_len = (int)_data[1] + 2;
    if (_data_len < packet_len)
      return false;

    parse_packet(_data, packet_len);
    memsft(_data, _data_len, packet_len);
    _data_len -= packet_len;

    return true;
  }
  void parse_packet(unsigned char _packet[], int _packet_len) {
    if (_packet_len == 20 && _packet[8] == 0 && _packet[9] == 0 &&
        _packet[10] == 0 && _packet[11] == 0 && _packet[12] == 0 &&
        _packet[13] == 0) {
      int16_t temp[9];
      memcpy(temp, _packet + 2, 18);
      float accx = ((float)temp[0]) * 0.001f * 9.801f;
      float accy = ((float)temp[1]) * 0.001f * 9.801f;
      float accz = ((float)temp[2]) * 0.001f * 9.801f;
      float gyrox = ((float)temp[6]) / 16.0f / 180.0f * 3.14156f;
      float gyroy = ((float)temp[7]) / 16.0f / 180.0f * 3.14156f;
      float gyroz = ((float)temp[8]) / 16.0f / 180.0f * 3.14156f;
      (*m_bno055_rx_gyro)(accx, accy, accz, gyrox, gyroy, gyroz);
    } else if (_packet_len == 3 && _packet[1] == 1) {
      (*m_bno055_rx_chip_id)(_packet[2]);
    }
  }
  void bno055_rx() {
    static int rx_data_len = 0;
    static unsigned char rx_data[200];
    while (mbno055_uart.readable()) {
      rx_data_len++;
      mbno055_uart.read(rx_data + rx_data_len - 1, 1);
    }

    while (extract_packet(rx_data, rx_data_len))
      ;
  }

private:
  void pc_tx() {
    if (i >= m_uart_data_len - 1) {
      mbno055_uart.attach(NULL, SerialBase::TxIrq);
      mbno055_uart.write(m_uart_data + i, 1);
      m_uart_data_len = 0;
      i = 0;
    } else {
      mbno055_uart.write(m_uart_data + i, 1);
      i++;
    }
  }

  // UART Output
  void add_uart_data(unsigned char _uart_data[], const int _uart_data_len) {
    const unsigned char f_no_data = (m_uart_data_len == 0);
    memcpy(m_uart_data + m_uart_data_len, _uart_data, _uart_data_len);
    m_uart_data_len += _uart_data_len;
    if (f_no_data) {
      mbno055_uart.attach(callback(this, &BNO055::pc_tx), SerialBase::TxIrq);
      pc_tx();
    }
  }
};
