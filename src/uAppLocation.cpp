#include <CppPotpourri.h>
#include <SensorFilter.h>
#include <GPSWrapper.h>
#include "SensorGlue.h"
#include "uApp.h"
#include "Motherflux0r.h"

extern WakeLock* wakelock_mag;
extern WakeLock* wakelock_baro;
extern WakeLock* wakelock_gps;


uAppLocation::uAppLocation() : uApp("Locator", (Image*) &display) {}

uAppLocation::~uAppLocation() {}



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
int8_t uAppLocation::_lc_on_preinit() {
  int8_t ret = 1;
  FB->fill(BLACK);
  redraw_app_window();
  if (nullptr != wakelock_gps) {   wakelock_gps->acquire();   }
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppLocation::_lc_on_active() {
  int8_t ret = 0;
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppLocation::_lc_on_teardown() {
  int8_t ret = 1;
  if (nullptr != wakelock_gps) {   wakelock_gps->release();   }
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppLocation::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppLocation::_process_user_input() {
  int8_t ret = 1;
  if (_slider_current != _slider_pending) {
    FB->fill(BLACK);
    FB->setTextSize(0);
    FB->setCursor(0, 0);
    if (_slider_pending <= 7) {
      FB->writeString("Current location");
    }
    else if (_slider_pending <= 15) {
      FB->writeString("GPS stats");
    }
    else if (_slider_pending <= 22) {
    }
    else if (_slider_pending <= 30) {
    }
    else if (_slider_pending <= 37) {
    }
    else if (_slider_pending <= 45) {
    }
    else if (_slider_pending <= 52) {
    }
    else {
    }
    _slider_current = _slider_pending;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    if (diff & 0x0001) {   // Interpret a cancel press as a return to APP_SELECT.
      uApp::setAppActive(AppID::APP_SELECT);
      if (nullptr != wakelock_mag) {   wakelock_mag->release();   }
      ret = -1;
    }
    if (diff & 0x0002) {   // Cluttered display toggle.
      if (_buttons_pending & 0x0002) {
        _cluttered_display(!_cluttered_display());
      }
    }
    if (diff & 0x0008) {   // Text of actual value.
      if (_buttons_pending & 0x0008) {
        _render_text_value(!_render_text_value());
      }
    }
    if (diff & 0x0010) {   // Button 5
      if (_buttons_pending & 0x0010) {
        _render_lock_range(!_render_lock_range());
      }
    }
    _buttons_current = _buttons_pending;
  }
  return ret;
}



/*
* Draws the tricorder app.
*/
void uAppLocation::_redraw_window() {
  StringBuilder tmp_val_str;
  if (_slider_current <= 7) {
    FB->setTextColor(WHITE, BLACK);
    FB->setCursor(0, 11);
    tmp_val_str.concatf(
      "%.6f\n%.6f\n%.6f\nALT:   %u\nSpeed: %u",
      current_location.lat,
      current_location.lon,
      current_location.mag_bearing,
      current_location.altitude,
      current_location.speed
    );
    FB->writeString(tmp_val_str.string());
  }
  else if (_slider_current <= 15) {
    FB->setTextColor(WHITE, BLACK);
    FB->setCursor(0, 11);
    tmp_val_str.concatf(
      "%u\nSATS:  %u\nPrsd: %u\nRejt: %u",
      current_location.timestamp,
      current_location.sat_count,
      gps.sentencesParsed(),
      gps.sentencesRejected()
    );
    FB->writeString(tmp_val_str.string());
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
