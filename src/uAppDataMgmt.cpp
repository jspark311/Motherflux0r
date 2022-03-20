#include "Motherflux0r.h"
#include "uApp.h"

    // TODO: Put things into reset states.
    // TODO: Power off non-essential rails.
    // TODO: Scale back the CPU clock.
    // TODO: Set wake sources.


uAppDataMgmt::uAppDataMgmt() : uApp("Data Mgmt", (Image*) &display) {}

uAppDataMgmt::~uAppDataMgmt() {}



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
int8_t uAppDataMgmt::_lc_on_preinit() {
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
int8_t uAppDataMgmt::_lc_on_active() {
  int8_t ret = 0;
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppDataMgmt::_lc_on_teardown() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppDataMgmt::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppDataMgmt::_process_user_input() {
  int8_t ret = 0;

  if (_slider_current != _slider_pending) {
    _slider_current = _slider_pending;
    ret = 1;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    ret = 1;
    if (_buttons_pending == 0x0028) {
      // Interpret a cancel press as a return to APP_SELECT.
      uApp::setAppActive(AppID::APP_SELECT);
      ret = -1;
    }
    _buttons_current = _buttons_pending;
  }
  return ret;
}



/*
* Draws the app.
*/
void uAppDataMgmt::_redraw_window() {
  FB->fill(0);
}
