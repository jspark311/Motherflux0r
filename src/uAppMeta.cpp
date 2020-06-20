#include <CppPotpourri.h>
#include <SensorFilter.h>
#include "VEML6075.h"
#include "ICM20948.h"
#include "BME280.h"
#include "AMG88xx.h"
#include "DRV425.h"
#include "TSL2561.h"
#include "TMP102.h"
#include "uApp.h"
#include "Motherflux0r.h"


extern SSD13xx display;

extern SensorFilter<float> graph_array_frame_rate;
extern SensorFilter<float> graph_array_cpu_time;
extern StopWatch stopwatch_main_loop_time;
//extern StopWatch stopwatch_main_loop_time;

extern uint32_t boot_time;
uint32_t disp_update_rate = 0;


uAppMeta::uAppMeta() : uApp("Meta", (Image*) &display) {}

uAppMeta::~uAppMeta() {}


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
int8_t uAppMeta::_lc_on_preinit() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppMeta::_lc_on_active() {
  int8_t ret = 0;
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppMeta::_lc_on_teardown() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppMeta::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppMeta::_process_user_input() {
  int8_t ret = 1;

  if (_slider_current != _slider_pending) {
    if (_slider_pending <= 7) {
      // Basic firmware stuff
      FB->setCursor(0, 11);
      FB->setTextSize(0);
      FB->setTextColor(WHITE);
      FB->writeString("Firmware ");
      FB->setTextColor(CYAN);
      FB->writeString(TEST_PROG_VERSION "\n");
      FB->setTextColor(CYAN);
      FB->writeString(__DATE__ "\n");
      FB->writeString(__TIME__ "\n");
      FB->setTextColor(WHITE);
      FB->writeString("Serial: \n");
      FB->writeString("Uptime: \n");
      FB->writeString("FPS:    ");
    }
    else if (_slider_pending <= 22) {
      FB->setCursor(0, 11);
      FB->setTextSize(0);
      FB->setTextColor(WHITE);
      FB->writeString("ML Worst\n");
      FB->writeString("ML Best\n");
      FB->writeString("ML Mean\n");
      FB->writeString("ML Last\n");
    }
    else if (_slider_current <= 30) {
    }
    _slider_current = _slider_pending;
    ret = 1;
  }
  if (0 == ret) {
    if (_slider_pending <= 22) {
      ret = 1;
    }
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    ret = 1;
    if (_buttons_pending == 0x0001) {
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
void uAppMeta::_redraw_window() {
  StringBuilder tmp_val_str;
  if (_slider_pending <= 7) {
    FB->setTextColor(CYAN, BLACK);
    FB->setCursor(47, 35);
    FB->writeString(Serial ? "USB" : "None");
    FB->setCursor(47, 43);
    tmp_val_str.concat((uint) (millis() - boot_time));
    FB->writeString(&tmp_val_str);
    FB->setCursor(47, 51);
    tmp_val_str.clear();
    tmp_val_str.concat((uint) disp_update_rate);
    FB->writeString(&tmp_val_str);
  }
  else if (_slider_current <= 15) {
    if (graph_array_frame_rate.dirty()) {
      draw_graph_obj(
        0, 10, 96, 45, CYAN,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_frame_rate
      );
      FB->setTextSize(0);
      FB->setCursor(0, 56);
      FB->setTextColor(WHITE);
      FB->writeString("Framerate:  ");
      FB->setTextColor(CYAN, BLACK);
      tmp_val_str.clear();
      tmp_val_str.concat((uint) graph_array_frame_rate.value());
      FB->writeString(&tmp_val_str);
    }
  }
  else if (_slider_current <= 22) {
    FB->setTextColor(CYAN, BLACK);
    FB->setCursor(52, 11);
    tmp_val_str.concat((uint) stopwatch_main_loop_time.worstTime());
    FB->writeString(&tmp_val_str);
    FB->setCursor(52, 19);
    tmp_val_str.clear();
    tmp_val_str.concat((uint) stopwatch_main_loop_time.bestTime());
    FB->writeString(&tmp_val_str);
    FB->setCursor(52, 27);
    tmp_val_str.clear();
    tmp_val_str.concat((uint) stopwatch_main_loop_time.meanTime());
    FB->writeString(&tmp_val_str);
    FB->setCursor(52, 35);
    tmp_val_str.clear();
    tmp_val_str.concat((uint) stopwatch_main_loop_time.lastTime());
    FB->writeString(&tmp_val_str);
  }
  else if (_slider_current <= 30) {
    StringBuilder disp_str;
    if ((_last_i2c_scan + 600) <= millis()) {
      if (_button_pressed_up()) {
        uApp::redraw_app_window("I2C Probe Wire", 0, 0);
        for (uint8_t addr = 0; addr < 0x80; addr++) {
          delay(5);
          Wire.beginTransmission(addr);
          if (0 == Wire.endTransmission()) {
            disp_str.concatf("0x%02x ", addr);
            FB->setPixel(addr & 0x1F, 11 + (addr >> 5), CYAN);
          }
        }
        if (disp_str.length() > 0) {
          FB->setCursor(0, 20);
          FB->writeString(&disp_str);
        }
        _last_i2c_scan = millis();
      }
      else if (_button_pressed_dn()) {
        uApp::redraw_app_window("I2C Probe Wire1", 0, 0);
        for (uint8_t addr = 0; addr < 0x80; addr++) {
          delay(5);
          Wire1.beginTransmission(addr);
          if (0 == Wire1.endTransmission()) {
            disp_str.concatf("0x%02x ", addr);
            FB->setPixel(addr & 0x1F, 11 + (addr >> 5), CYAN);
          }
        }
        if (disp_str.length() > 0) {
          FB->setCursor(0, 20);
          FB->writeString(&disp_str);
        }
        _last_i2c_scan = millis();
      }
    }
  }
  else if (_slider_current <= 37) {
  }
  else if (_slider_current <= 45) {
  }
  else if (_slider_current <= 52) {
  }
  else {
    // CPU load metrics
    if (graph_array_cpu_time.dirty()) {
      draw_graph_obj(
        0, 10, 96, 45, 0xFE00,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_cpu_time
      );
      FB->setTextSize(0);
      FB->setCursor(0, 56);
      FB->setTextColor(WHITE);
      FB->writeString("Load:  ");
      FB->setTextColor(GREEN, BLACK);
      tmp_val_str.concat(graph_array_cpu_time.value());
      FB->writeString(&tmp_val_str);
    }
  }
}
