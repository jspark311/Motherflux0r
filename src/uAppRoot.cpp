#include "Motherflux0r.h"
#include <CppPotpourri.h>
#include "uApp.h"



typedef struct {
  const char*   title;
  const uint8_t icon_id;
  const AppID   app_id;
  const uint8_t modal_id;
} uapp_root_selection;


const uapp_root_selection SELECTIONS[] = {
  { .title    = (char*) "Shutdown    ",
    .icon_id  = ICON_LIGHT,
    .app_id   = AppID::SUSPEND,
    .modal_id = UAPP_MODAL_NONE },

  /* Tricorder group */
  { .title    = (char*) "Magnetometer",
    .icon_id  = ICON_MAGNET,
    .app_id   = AppID::MAGNETOMETER,
    .modal_id = UAPP_MODAL_NONE },
  { .title    = (char*) "Inertial    ",
    .icon_id  = ICON_IMU,
    .app_id   = AppID::TRICORDER,
    .modal_id = UAPP_MODAL_TRICORDER_IMU },
  { .title    = (char*) "GPS Diag    ",
    .icon_id  = ICON_GPS,
    .app_id   = AppID::TRICORDER,
    .modal_id = UAPP_MODAL_TRICORDER_GPS },
  { .title    = (char*) "Thermograph ",
    .icon_id  = ICON_THERMO,
    .app_id   = AppID::TRICORDER,
    .modal_id = UAPP_MODAL_TRICORDER_THERMOPILE },
  { .title    = (char*) "Photometry  ",
    .icon_id  = ICON_LIGHT,
    .app_id   = AppID::TRICORDER,
    .modal_id = UAPP_MODAL_TRICORDER_LIGHT },
  { .title    = (char*) "Atmospheric ",
    .icon_id  = ICON_RH,
    .app_id   = AppID::TRICORDER,
    .modal_id = UAPP_MODAL_TRICORDER_ATMO },
  { .title    = (char*) "Synth Box   ",
    .icon_id  = ICON_SOUND,
    .app_id   = AppID::SYNTH_BOX,
    .modal_id = UAPP_MODAL_NONE },
  { .title    = (char*) "Comms       ",
    .icon_id  = ICON_SOUND,
    .app_id   = AppID::COMMS_TEST,
    .modal_id = UAPP_MODAL_NONE },
  { .title    = (char*) "Meta        ",
    .icon_id  = ICON_BATTERY,
    .app_id   = AppID::META,
    .modal_id = UAPP_MODAL_NONE },
  { .title    = (char*) "GPS Tools   ",
    .icon_id  = ICON_GPS,
    .app_id   = AppID::LOCATOR,
    .modal_id = UAPP_MODAL_NONE },
  { .title    = (char*) "Data MGMT   ",
    .icon_id  = ICON_RH,
    .app_id   = AppID::DATA_MGMT,
    .modal_id = UAPP_MODAL_NONE },
  { .title    = (char*) "Settings    ",
    .icon_id  = ICON_UVI,
    .app_id   = AppID::CONFIGURATOR,
    .modal_id = UAPP_MODAL_NONE },
  { .title    = (char*) "Touch Diag  ",
    .icon_id  = ICON_LIGHT,
    .app_id   = AppID::TOUCH_TEST,
    .modal_id = UAPP_MODAL_NONE }
};

const uint MAX_SEL_INDICIES = sizeof(SELECTIONS) / sizeof(uapp_root_selection);


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
  if (_modal_id >= MAX_SEL_INDICIES) {
    _modal_id = UAPP_MODAL_NONE;
  }
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
    _modal_id = MAX_SEL_INDICIES - (MAX_SEL_INDICIES * ((float) _slider_pending/60.0));
    _modal_id = strict_min(_modal_id, (MAX_SEL_INDICIES-1));
    _slider_current = _slider_pending;
    ret = 1;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    ret = 1;

    if (diff & 0x0001) {   // Cancel
      // Nullify the deepest tree specifier. If none are defined, interpret a
      //   cancel press as a command to enter standby.
      if (_buttons_pending & 0x0001) {
        if (UAPP_MODAL_NONE != _modal_id) {
          _modal_id = UAPP_MODAL_NONE;
          ret = 1;
        }
        else {
          uApp::setAppActive(AppID::HOT_STANDBY);
          ret = -1;
        }
      }
    }

    if (diff & 0x0002) {   // Up
      if (_buttons_pending & 0x0002) {
      }
    }

    if (diff & 0x0004) {   // Accept
      if (_buttons_pending & 0x0004) {   // Interpret an ok press as an app selection.
        uApp::setAppActive(SELECTIONS[_modal_id].app_id, SELECTIONS[_modal_id].modal_id);
        ret = -1;
      }
    }

    if (diff & 0x0008) {   // Right
      if (_buttons_pending & 0x0008) {   // Next selection
        _modal_id++;
        if (_modal_id >= MAX_SEL_INDICIES) {
          _modal_id = 0;
        }
        ret = 1;
      }
    }
    if (diff & 0x0010) {   // Down
      if (_buttons_pending & 0x0010) {
      }
    }
    if (diff & 0x0020) {   // Left
      if (_buttons_pending & 0x0020) {   // Previous selection.
        if (0 == _modal_id) {
          _modal_id = MAX_SEL_INDICIES;
        }
        _modal_id--;
        ret = 1;
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
  StringBuilder tmp_val_str;
  FB->fillRect(0, 11, FB->x()-1, FB->y()-12, BLACK);
  render_button_icon(SELECTIONS[_modal_id].icon_id, 0, 11, 0xFFFF);

  FB->setCursor(36, 12);
  FB->setTextColor(WHITE, BLACK);
  FB->writeString("Fxn ");
  FB->setTextColor(MAGENTA, BLACK);
  tmp_val_str.concatf("%u/%2u", (_modal_id+1), MAX_SEL_INDICIES);
  FB->writeString((char*) tmp_val_str.string());
  FB->setCursor(0, 36);
  FB->writeString(SELECTIONS[_modal_id].title);

  const uint8_t slider_pix = 2 + (((1+_modal_id) / (float) (MAX_SEL_INDICIES+1)) * (FB->x()-5));
  FB->fillRect(0, 55, FB->x()-1, 7, 0x0000);
  FB->drawRoundRect(0, 54, FB->x(), 9, 3, 0xFFFF);
  FB->fillRect(slider_pix-1, 55, 3, 7, MAGENTA);
}
