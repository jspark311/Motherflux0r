#include <CppPotpourri.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SensorFilter.h>
#include <SX8634.h>
#include "uApp.h"
#include "Motherflux0r.h"


extern Adafruit_SSD1331 display;

/* Sensor representations... */
extern SX8634* touch;



uAppTouchTest::uAppTouchTest() : uApp("Touch Test") {
}


uAppTouchTest::~uAppTouchTest() {
}



int8_t uAppTouchTest::refresh() {
  int8_t ret = 0;
  _stopwatch.markStart();
  if (uApp::drawnApp() != uApp::appActive()) {
    redraw_app_window();
    display.setTextSize(0);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0, 12);
    display.print("Mode:");
    display.setCursor(0, 28);
    display.print("Slider:");
  }
  switch (_process_user_input()) {
    case 2:
    case 1:
    case 0:
      _redraw_window();
    default:
      break;
  }
  _stopwatch.markStop();
  return ret;
}



int8_t uAppTouchTest::_process_user_input() {
  int8_t ret = 0;

  if (_slider_current != _slider_pending) {
    uint16_t sval = 60 - _slider_pending;
    //display.fillRect(0, 45, 28, 8, BLACK);
    display.setTextSize(0);
    display.setTextColor(BLUE, BLACK);
    display.setCursor(0, 20);
    display.print(touch->getModeStr(touch->operationalMode()));
    display.setCursor(0, 36);
    display.print(sval);

    uint8_t slider_pix = 2 + (((1+sval) / 61.0f) * (display.width()-5));
    display.fillRect(0, 55, display.width()-1, 7, BLACK);
    display.drawRoundRect(0, 54, display.width(), 9, 3, GREEN);
    display.fillRect(slider_pix-1, 55, 3, 7, RED);

    _slider_current = _slider_pending;
    ret++;
  }
  if (_buttons_current != _buttons_pending) {
    const uint8_t x_coords[] = {37, 57, 77, 77, 57, 37};
    const uint8_t y_coords[] = {14, 14, 14, 34, 34, 34};
    uint16_t diff = _buttons_current ^ _buttons_pending;
    if (diff & 0x0001) {   // Interpret a cancel press at full slider right as a return to APP_SELECT.
      if (_slider_current == 0) {
        uApp::setAppActive(AppID::APP_SELECT);
      }
    }

    for (uint8_t i = 0; i < 6; i++) {
      if (touch->buttonPressed(i)) {
        display.fillRoundRect(x_coords[i], y_coords[i], 18, 18, 4, RED);
      }
      else {
        display.fillRect(x_coords[i], y_coords[i], 18, 18, BLACK);
        display.drawRoundRect(x_coords[i], y_coords[i], 18, 18, 4, GREEN);
      }
    }
    display.setCursor(0, 44);
    display.fillRect(0, 44, 28, 8, BLACK);
    display.setTextColor(RED, BLACK);
    display.print(_buttons_pending, HEX);
    _buttons_current = _buttons_pending;
    ret++;
  }
  return ret;
}


/*
* Draws the touch test app.
*/
void uAppTouchTest::_redraw_window() {
}
