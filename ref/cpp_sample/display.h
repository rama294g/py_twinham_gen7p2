
#include "SSD1305.h"

class display {
public:
  int gain{40};
  int neutral{15};
  int response{10};
  float battery{0};
  float accx{0};
  float accy{0};
  float accz{0};
  float gyrox{0};
  float gyroy{0};
  float gyroz{0};
  float tilt{0};
  float thrust{0};
  float current{0};
  float encoder1{0};
  float encoder2{0};
  float loadcell{0};

  int m_line{0};
  int m_selected_line{0};

  struct ret_aaaa {
    char buf[16]{0};
    int len{0};
    ret_aaaa(){};
    ret_aaaa(const char *_c, const int _len) {
      memcpy(buf, _c, _len);
      len = _len;
    }
  };
  static ret_aaaa qqqq(const float _v, const int _d1, const int _d2) {
    const bool f = (_v >= 0);
    const float w = fabs(_v);
    int x = (int)floor(w);
    float y = w - (float)x;
    char a[10];
    for (int i = 0; i < 10; i++) {
      a[i] = '0' + x % 10;
      x /= 10;
    }
    char b[10];
    for (int i = 0; i < 10; i++) {
      y *= 10;
      int z = (int)floor(y);
      y -= z;
      b[i] = '0' + z;
    }
    ret_aaaa ret;
    int pos = 0;
    ret.buf[pos++] = (f ? '+' : '-');
    for (int i = 0; i < _d1; i++) {
      ret.buf[pos++] = a[_d1 - 1 - i];
    }
    ret.buf[pos++] = '.';
    for (int i = 0; i < _d2; i++) {
      ret.buf[pos++] = b[i];
    }
    ret.len = pos;
    return ret;
  }
  static ret_aaaa get_bbbb(const ret_aaaa &_s1, const float _v, const int _d1,
                           const int _d2, const ret_aaaa &_s2) {
    ret_aaaa ret;
    int i = 0;
    memcpy(ret.buf + i, _s1.buf, _s1.len);
    i += _s1.len;
    ret_aaaa number = qqqq(_v, _d1, _d2);
    memcpy(ret.buf + i, number.buf, number.len);
    i += number.len;
    memcpy(ret.buf + i, _s2.buf, _s2.len);
    i += _s2.len;
    ret.len = i;
    return ret;
  }
  ret_aaaa get_aaaa(const int _line) const {
    ret_aaaa ret;
    switch (_line) {
    case 0: {
      sprintf(ret.buf, "GAIN: %7d %%", gain);
      ret.len = 16;
      return ret;
    }
    case 1: {
      sprintf(ret.buf, "NEUTRAL: %2d deg", neutral);
      ret.len = 15;
      return ret;
    }
    case 2: {
      sprintf(ret.buf, "RESPONSE: %3d %%", response);
      ret.len = 15;
      return ret;
    }
    case 3: {
      int soc_index =
          (int)round((battery - 14.0f) * 5.0f); // 0% at 7.0v, 100% at 8.0v
      if (soc_index < 0)
        soc_index = 0;
      if (soc_index > 10)
        soc_index = 10;
      sprintf(ret.buf, "BAT  ");
      for (int i = 0; i < 10; i++) {
        ret.buf[i + 4] = (i < soc_index ? 0x81 : 0x80);
      }
      ret.len = 14;
      return ret;
    }
    case 4: {
      ret = get_bbbb(ret_aaaa("ACCX:", 5), accx / 9.801f, 1, 3,
                     ret_aaaa(" G", 2));
      return ret;
    }
    case 5: {
      ret = get_bbbb(ret_aaaa("ACCY:", 5), accy / 9.801f, 1, 3,
                     ret_aaaa(" G", 2));
      return ret;
    }
    case 6: {
      ret = get_bbbb(ret_aaaa("ACCZ:", 5), accz / 9.801f, 1, 3,
                     ret_aaaa(" G", 2));
      return ret;
    }
    case 7: {
      sprintf(ret.buf, "GYROX:%+4d deg/s", (int)(gyrox));
      ret.len = 14;
      return ret;
    }
    case 8: {
      sprintf(ret.buf, "GYROY:%+4d deg/s", (int)(gyroy));
      ret.len = 14;
      return ret;
    }
    case 9: {
      sprintf(ret.buf, "GYROZ:%+4d deg/s", (int)(gyroz));
      ret.len = 14;
      return ret;
    }
    }
    return ret;
  }
  int increment[9]{10, 5, 5, 0, 0, 0, 0, 0, 0};
  int min_value[9]{10, 0, 5, 0, 0, 0, 0, 0, 0};
  int max_value[9]{100, 50, 40, 0, 0, 0, 0, 0, 0};
  int *ptr[9]{&gain,   &neutral, &response, nullptr, nullptr,
              nullptr, nullptr,  nullptr,   nullptr};

  void param_up() {
    switch (m_selected_line) {
    case 0:
    case 1:
    case 2:
      int &tgt = *ptr[m_selected_line];
      tgt += increment[m_selected_line];
      if (tgt > max_value[m_selected_line])
        tgt = max_value[m_selected_line];
      break;
    }
  }
  void param_down() {
    switch (m_selected_line) {
    case 0:
    case 1:
    case 2:
      int &tgt = *ptr[m_selected_line];
      tgt -= increment[m_selected_line];
      if (tgt < min_value[m_selected_line])
        tgt = min_value[m_selected_line];
      break;
    }
  }

  void select_up() {
    m_selected_line--;
    if (m_selected_line < 0)
      m_selected_line = 0;
    if (m_line > m_selected_line)
      m_line = m_selected_line;
  }
  void select_down() {
    m_selected_line++;
    if (m_selected_line > 9)
      m_selected_line = 9;
    if (m_line < m_selected_line - 3)
      m_line = m_selected_line - 3;
  }

  void update_display(SSD1305 &_screen) {
    for (int i = 0; i < 4; i++) {
      ret_aaaa a0 = get_aaaa(m_line + i);
      _screen.setString(0, i, a0.buf, a0.len, m_line + i == m_selected_line);
    }
  };
};
