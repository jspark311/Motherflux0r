#include <CppPotpourri.h>
#include <SensorFilter.h>
#include "uApp.h"
#include "Motherflux0r.h"

extern SSD13xx display;


uAppRoot::uAppRoot() : uApp("Root menu", (Image*) &display) {}

uAppRoot::~uAppRoot() {}


/*******************************************************************************
* Lifecycle callbacks from the super class
*******************************************************************************/
/**
* Called by superclass on activation of lifecycle. This function should prepare
*   the class as if it were freshly instantiated. Redraw will not happen until
*   the tick following this function returning 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppRoot::_lc_on_preinit() {
  int8_t ret = 1;
  redraw_app_window();
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppRoot::_lc_on_active() {
  int8_t ret = 0;
  _redraw_window();
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppRoot::_lc_on_teardown() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppRoot::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppRoot::_process_user_input() {
  int8_t ret = 0;

  if (_slider_current != _slider_pending) {
    _slider_current = _slider_pending;
    ret = 1;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    ret = 1;
    if (diff & 0x0001) {   // Interpret a cancel press as a command to enter standby.
      uApp::setAppActive(AppID::HOT_STANDBY);
      ret = -1;
    }
    if (diff & 0x0004) {   // Button 3
      if (_buttons_pending & 0x0004) {   // Interpret an ok press as an app selection.
        uApp::setAppActive(app_page);
        ret = -1;
      }
    }
    _buttons_current = _buttons_pending;
  }
  return ret;
}



/*
* Draws the app.
*/
void uAppRoot::_redraw_window() {
  const uint8_t ICON_SIZE   = 32;
  const uint8_t TEXT_OFFSET = 56;

  FB->fillRect(0, 11, FB->x()-1, FB->y()-12, BLACK);
  FB->setCursor(0, TEXT_OFFSET);
  FB->setTextColor(WHITE);
  FB->writeString("Fxn: ");
  FB->setTextColor(MAGENTA, BLACK);
  if (_slider_current <= 7) {
    render_button_icon(ICON_LIGHT, 0, 11, 0xFFFF);
    FB->writeString("Touch Diag  ");
    app_page = AppID::TOUCH_TEST;
  }
  else if (_slider_current <= 15) {
    render_button_icon(ICON_UVI, 0, 11, 0xFFFF);
    FB->writeString("Settings    ");
    app_page = AppID::CONFIGURATOR;
  }
  else if (_slider_current <= 22) {
    render_button_icon(ICON_RH, 0, 11, 0xFFFF);
    FB->writeString("Data MGMT   ");
    app_page = AppID::DATA_MGMT;
  }
  else if (_slider_current <= 30) {
    render_button_icon(ICON_MAGNET, 0, 11, 0xFFFF);
    FB->writeString("Synth Box   ");
    app_page = AppID::SYNTH_BOX;
  }
  else if (_slider_current <= 37) {
    render_button_icon(ICON_SOUND, 0, 11, 0xFFFF);
    FB->writeString("Comms       ");
    app_page = AppID::COMMS_TEST;
  }
  else if (_slider_current <= 45) {
    render_button_icon(ICON_THERMO, 0, 11, 0xFFFF);
    FB->writeString("Meta        ");
    app_page = AppID::META;
  }
  else if (_slider_current <= 52) {
    render_button_icon(ICON_GPS, 0, 11, 0xFFFF);
    FB->writeString("GPS Tools   ");
    app_page = AppID::LOCATOR;
  }
  else {
    render_button_icon(ICON_IMU, 0, 11, 0xFFFF);
    FB->writeString("Tricorder   ");
    app_page = AppID::TRICORDER;
  }
}
