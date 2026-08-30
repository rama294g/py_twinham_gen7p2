
#ifndef _UART2_H_
#define _UART2_H_
#include "mbed.h"

class UART2 {
private:
  int m_uart_data_len{0};
  unsigned char m_uart_data[1024]{0};
  int m_idx{0};

  UnbufferedSerial m_debug2;

public:
  UART2(const PinName _tx, const PinName _rx) : m_debug2(_tx, _rx){};

  void init() {
    m_debug2.baud(9600);
    m_debug2.format(8, SerialBase::None, 1);
    m_debug2.attach(callback(this, &UART2::pc_rx), SerialBase::RxIrq);
  }

  void pc_tx() {
    if (m_idx >= m_uart_data_len - 1) {
      m_debug2.attach(NULL, SerialBase::TxIrq);
      m_debug2.write(m_uart_data + m_idx, 1);
      m_uart_data_len = 0;
      m_idx = 0;
    } else {
      m_debug2.write(m_uart_data + m_idx, 1);
      m_idx++;
    }
  }
  void add_uart_data(unsigned char _uart_data[], const int _uart_data_len) {
    const unsigned char f_no_data = (m_uart_data_len == 0);
    memcpy(m_uart_data + m_uart_data_len, _uart_data, _uart_data_len);
    m_uart_data_len += _uart_data_len;
    if (f_no_data) {
      m_debug2.attach(callback(this, &UART2::pc_tx), SerialBase::TxIrq);
      pc_tx();
    }
  }
  void pc_rx() {
    char buf[1];
    while (m_debug2.readable()) {
      m_debug2.read(buf, sizeof(buf));
    }
  }
};
#endif