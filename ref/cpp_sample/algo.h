
#include "mbed.h"
#include <math.h>

// Macros
#define M_PI 3.141592653589793
#define LIM(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(a) ((a) > 0 ? (a) : (-a))

float ratelimitter(float current, float prev, float posirate, float negarate) {
  float gap = current - prev;
  gap = LIM(gap, negarate, posirate);
  return gap + prev;
}

void fitin360(float &v) { v -= round(v / 360) * 360; }

int enc_1phase(const bool _d) {
  static bool d_prev = _d;
  static int cnt = 0;

  if ((!d_prev) && (_d)) { // d: up
      cnt++;
  } else if ((d_prev) && (!_d)) { // d: down
      cnt++;
  } 

  d_prev = _d;

  return cnt;
}
int enc_2phase(const bool _d, const bool _q) {
  static bool d_prev = _d;
  static bool q_prev = _q;
  static int cnt = 0;

  if ((!d_prev) && (_d)) { // d: up
    if (_q)
      cnt--;
    else
      cnt++;
  } else if ((d_prev) && (!_d)) { // d: down
    if (_q)
      cnt++;
    else
      cnt--;
  } else if ((!q_prev) && (_q)) { // q: up
    if (_d)
      cnt++;
    else
      cnt--;
  } else if ((q_prev) && (!_q)) { // q: down
    if (_d)
      cnt--;
    else
      cnt++;
  }

  d_prev = _d;
  q_prev = _q;

  return cnt;
}

struct rgb {
  static DigitalOut *rLED, *gLED, *bLED;
  float r;
  float g;
  float b;
  rgb() : r(0), g(0), b(0){};
  rgb(const float _r, const float _g, const float _b) : r(_r), g(_g), b(_b){};
  static rgb get_jet_color(const float _h_org) {
    float _h = _h_org - floor(_h_org / 6) * 6;
    if (_h < 1)
      return rgb(_h - 0, 0, 1);
    if (_h < 2)
      return rgb(1, 0, 2 - _h);
    if (_h < 3)
      return rgb(1, _h - 2, 0);
    if (_h < 4)
      return rgb(4 - _h, 1, 0);
    if (_h < 5)
      return rgb(0, 1, _h - 4);
    if (_h < 6)
      return rgb(0, 6 - _h, 1);
    return rgb(0, 0, 0);
  }
  void apply() {
    *rLED = r > 0.5 ? LED_ON : LED_OFF;
    *gLED = g > 0.5 ? LED_ON : LED_OFF;
    *bLED = b > 0.5 ? LED_ON : LED_OFF;
  }
};

DigitalOut *rgb::rLED = NULL;
DigitalOut *rgb::gLED = NULL;
DigitalOut *rgb::bLED = NULL;