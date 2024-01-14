#include <CppPotpourri.h>
#include <SensorFilter.h>
#include "ICM20948.h"
#include "DRV425.h"
#include "uApp.h"
#include "Motherflux0r.h"

extern SensorFilter<float> graph_array_frame_rate;
extern SensorFilter<float> graph_array_cpu_time;
extern StopWatch stopwatch_main_loop_time;
//extern StopWatch stopwatch_main_loop_time;

extern ManuvrPMU pmu;

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
  redraw_app_window();
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
    FB->fill(BLACK);
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
      ImageGraph<float> graph(96, 45);
      graph.fg_color            = 0xFFFFFFFF;

      const uint32_t  DATA_SIZE_FPS = graph_array_frame_rate.windowSize();
      const uint32_t  LAST_SIDX_FPS = graph_array_frame_rate.lastIndex();
      const uint32_t  DATA_IDX_FPS  = (1 + LAST_SIDX_FPS + strict_abs_delta(DATA_SIZE_FPS, (uint32_t) 96)) % DATA_SIZE_FPS;
      const float*    F_MEM_PTR_FPS = graph_array_frame_rate.memPtr();
      float tmp_data_fps[DATA_SIZE_FPS];
      for (uint32_t i = 0; i < DATA_SIZE_FPS; i++) {
        tmp_data_fps[i] = *(F_MEM_PTR_FPS + ((i + LAST_SIDX_FPS) % DATA_SIZE_FPS));
      }

      graph.trace0.color        = CYAN;
      graph.trace0.dataset      = tmp_data_fps;
      graph.trace0.data_len     = DATA_SIZE_FPS;
      graph.trace0.enabled      = true;
      graph.trace0.autoscale_x  = false;
      graph.trace0.autoscale_y  = true;
      graph.trace0.show_x_range = false;
      graph.trace0.show_y_range = _cluttered_display();
      graph.trace0.show_value   = _render_text_value();
      graph.trace0.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
      graph.trace0.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.
      graph.trace0.offset_x     = DATA_IDX_FPS;

      const uint32_t  DATA_SIZE_CPU = graph_array_cpu_time.windowSize();
      const uint32_t  LAST_SIDX_CPU = graph_array_cpu_time.lastIndex();
      const uint32_t  DATA_IDX_CPU  = (1 + LAST_SIDX_CPU + strict_abs_delta(DATA_SIZE_CPU, (uint32_t) 96)) % DATA_SIZE_CPU;
      const float*    F_MEM_PTR_CPU = graph_array_cpu_time.memPtr();
      float tmp_data_cpu[DATA_SIZE_CPU];
      for (uint32_t i = 0; i < DATA_SIZE_CPU; i++) {
        tmp_data_cpu[i] = *(F_MEM_PTR_CPU + ((i + LAST_SIDX_CPU) % DATA_SIZE_CPU));
      }

      graph.trace1.color        = CYAN;
      graph.trace1.dataset      = tmp_data_cpu;
      graph.trace1.data_len     = DATA_SIZE_CPU;
      graph.trace1.enabled      = true;
      graph.trace1.autoscale_x  = false;
      graph.trace1.autoscale_y  = true;
      graph.trace1.show_x_range = false;
      graph.trace1.show_y_range = _cluttered_display();
      graph.trace1.show_value   = _render_text_value();
      graph.trace1.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
      graph.trace1.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.
      graph.trace1.offset_x     = DATA_IDX_CPU;

      graph.drawGraph(FB, 0, 10);

      FB->setTextSize(0);
      FB->setCursor(0, 56);
      FB->setTextColor(CYAN, BLACK);
      FB->writeString("FPS ");
      FB->setTextColor(0xFE, BLACK);
      FB->writeString("CPU");
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
    StringBuilder disp_str;
    disp_str.concatf("Battery: %.2fV  ", 0.0);
    FB->setCursor(0, 0);
    FB->setTextSize(0);
    FB->setTextColor(WHITE, BLACK);
    FB->writeString(&disp_str);
    FB->writeString(pmu.getChargeStateString());
    disp_str.clear();
    FB->setCursor(0, 10);
  }
  else if (_slider_current <= 45) {
  }
  else if (_slider_current <= 52) {
  }
  else {
    // CPU load metrics
  }
}
