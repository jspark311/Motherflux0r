#include <CppPotpourri.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SensorFilter.h>
#include "uApp.h"
#include "Motherflux0r.h"


extern Adafruit_SSD1331 display;


uAppConfigurator::uAppConfigurator() : uApp("Configurator") {}

uAppConfigurator::~uAppConfigurator() {}



int8_t uAppConfigurator::refresh() {
  int8_t ret = 0;
  _stopwatch.markStart();
  if (uApp::drawnApp() != uApp::appActive()) {
    redraw_app_window();
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



int8_t uAppConfigurator::_process_user_input() {
  int8_t ret = 0;

  if (_slider_current != _slider_pending) {
    redraw_app_window();
    _slider_current = _slider_pending;
    ret++;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    if (diff & 0x0001) {   // Interpret a cancel press as a return to APP_SELECT.
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
void uAppConfigurator::_redraw_window() {
  if (_slider_pending <= 7) {
  }
  else if (_slider_current <= 15) {
  }
  else if (_slider_current <= 22) {
  }
  else if (_slider_current <= 30) {
  }
  else if (_slider_current <= 37) {
  }
  else if (_slider_current <= 45) {
  }
  else if (_slider_current <= 52) {
  }
  else {
  }
}
