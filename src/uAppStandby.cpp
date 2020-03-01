#include <CppPotpourri.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SensorFilter.h>
#include "uApp.h"
#include "Motherflux0r.h"

    // TODO: Put things into reset states.
    // TODO: Power off non-essential rails.
    // TODO: Scale back the CPU clock.
    // TODO: Set wake sources.

extern Adafruit_SSD1331 display;


uAppStandby::uAppStandby() : uApp("Standby") {}


uAppStandby::~uAppStandby() {}



int8_t uAppStandby::refresh() {
  int8_t ret = 0;
  _stopwatch.markStart();
  if (uApp::drawnApp() != uApp::appActive()) {
    display.fillScreen(BLACK);
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



int8_t uAppStandby::_process_user_input() {
  int8_t ret = 0;

  if (_slider_current != _slider_pending) {
    _slider_current = _slider_pending;
    ret++;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    if (_buttons_pending == 0x0028) {
      // Interpret a cancel press as a return to APP_SELECT.
      uApp::setAppActive(AppID::APP_SELECT);
    }
    _buttons_current = _buttons_pending;
    ret++;
  }
  return ret;
}



/*
* Draws the tricorder app.
*/
void uAppStandby::_redraw_window() {
}
