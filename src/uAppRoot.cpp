#include <CppPotpourri.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SensorFilter.h>
#include "uApp.h"
#include "Motherflux0r.h"


extern Adafruit_SSD1331 display;



uAppRoot::uAppRoot() : uApp("Root menu") {}


uAppRoot::~uAppRoot() {}



int8_t uAppRoot::refresh() {
  int8_t ret = 0;
  _stopwatch.markStart();
  if (uApp::drawnApp() != uApp::appActive()) {
    redraw_app_window();
    _redraw_window();
  }
  switch (_process_user_input()) {
    case 2:
    case 1:
      _redraw_window();
    case 0:
    default:
      break;
  }
  _stopwatch.markStop();
  return ret;
}



int8_t uAppRoot::_process_user_input() {
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
    if (diff & 0x0004) {   // Button 3
      if (_buttons_pending & 0x0004) {   // Interpret an ok press as an app selection.
        uApp::setAppActive(app_page);
      }
    }
    _buttons_current = _buttons_pending;
    ret++;
  }
  return ret;
}



/*
* Draws the tricorder app.
*/
void uAppRoot::_redraw_window() {
  const uint8_t ICON_SIZE   = 32;
  const uint8_t TEXT_OFFSET = 56;

  display.fillRect(0, 11, display.width()-1, display.height()-12, BLACK);
  display.setCursor(0, TEXT_OFFSET);
  display.setTextColor(WHITE);
  display.print("Fxn: ");
  display.setTextColor(MAGENTA, BLACK);
  if (_slider_current <= 7) {
    render_button_icon(ICON_LIGHT, 0, 11, 0xFFFF);
    display.print("Touch Diag");
    app_page = AppID::TOUCH_TEST;
  }
  else if (_slider_current <= 15) {
    render_button_icon(ICON_UVI, 0, 11, 0xFFFF);
    display.print("Settings");
    app_page = AppID::CONFIGURATOR;
  }
  else if (_slider_current <= 22) {
    render_button_icon(ICON_RH, 0, 11, 0xFFFF);
    display.print("Data MGMT");
    app_page = AppID::DATA_MGMT;
  }
  else if (_slider_current <= 30) {
    render_button_icon(ICON_MAGNET, 0, 11, 0xFFFF);
    display.print("Synth Box");
    app_page = AppID::SYNTH_BOX;
  }
  else if (_slider_current <= 37) {
    render_button_icon(ICON_SOUND, 0, 11, 0xFFFF);
    display.print("Comms");
    app_page = AppID::COMMS_TEST;
  }
  else if (_slider_current <= 45) {
    render_button_icon(ICON_THERMO, 0, 11, 0xFFFF);
    display.print("Meta");
    app_page = AppID::META;
  }
  else if (_slider_current <= 52) {
    render_button_icon(ICON_GPS, 0, 11, 0xFFFF);
    display.print("I2C Scanner");
    app_page = AppID::I2C_SCANNER;
  }
  else {
    render_button_icon(ICON_IMU, 0, 11, 0xFFFF);
    display.print("Tricorder");
    app_page = AppID::TRICORDER;
  }
}
