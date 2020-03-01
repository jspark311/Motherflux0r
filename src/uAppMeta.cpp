#include <CppPotpourri.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
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


extern Adafruit_SSD1331 display;

extern SensorFilter<float> graph_array_frame_rate;
extern SensorFilter<float> graph_array_cpu_time;
extern StopWatch stopwatch_main_loop_time;
//extern StopWatch stopwatch_main_loop_time;

extern uint32_t boot_time;
extern uint32_t disp_update_rate;


uAppMeta::uAppMeta() : uApp("Meta") {}


uAppMeta::~uAppMeta() {}



int8_t uAppMeta::refresh() {
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



int8_t uAppMeta::_process_user_input() {
  int8_t ret = 0;

  if (_slider_current != _slider_pending) {
    redraw_app_window();
    if (_slider_pending <= 7) {
      // Basic firmware stuff
      display.setCursor(0, 11);
      display.setTextSize(0);
      display.setTextColor(WHITE);
      display.print("Firmware ");
      display.setTextColor(CYAN);
      display.println(TEST_PROG_VERSION);
      display.setTextColor(CYAN);
      display.println(__DATE__);
      display.println(__TIME__);
      display.setTextColor(WHITE);
      display.println("Serial: ");
      display.println("Uptime: ");
      display.print("FPS:    ");
    }
    else if (_slider_pending <= 22) {
      display.setCursor(0, 11);
      display.setTextSize(0);
      display.setTextColor(WHITE);
      display.println("ML Worst");
      display.println("ML Best");
      display.println("ML Mean");
      display.println("ML Last");
    }
    _slider_current = _slider_pending;
    ret++;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    if (diff & 0x0001) {   // Interpret a cancel press as a return to APP_SELECT.
      uApp::setAppActive(AppID::APP_SELECT);
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
    ret++;
  }
  return ret;
}



/*
* Draws the tricorder app.
*/
void uAppMeta::_redraw_window() {
  if (_slider_pending <= 7) {
    display.setTextColor(CYAN, BLACK);
    display.setCursor(47, 35);
    display.print(Serial ? "USB" : "None");
    display.setCursor(47, 43);
    display.print(millis() - boot_time);
    display.setCursor(47, 51);
    display.print(disp_update_rate);
  }
  else if (_slider_current <= 15) {
    if (graph_array_frame_rate.dirty()) {
      draw_graph_obj(
        0, 10, 96, 45, CYAN,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_frame_rate
      );
      display.setTextSize(0);
      display.setCursor(0, 56);
      display.setTextColor(WHITE);
      display.print("Framerate:  ");
      display.setTextColor(CYAN, BLACK);
      display.print(graph_array_frame_rate.value());
    }
  }
  else if (_slider_current <= 22) {
    display.setTextColor(CYAN, BLACK);
    display.setCursor(52, 11);
    display.print(stopwatch_main_loop_time.worstTime());
    display.setCursor(52, 19);
    display.print(stopwatch_main_loop_time.bestTime());
    display.setCursor(52, 27);
    display.print(stopwatch_main_loop_time.meanTime());
    display.setCursor(52, 35);
    display.print(stopwatch_main_loop_time.lastTime());
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
    // CPU load metrics
    if (graph_array_cpu_time.dirty()) {
      draw_graph_obj(
        0, 10, 96, 45, 0xFE00,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_cpu_time
      );
      display.setTextSize(0);
      display.setCursor(0, 56);
      display.setTextColor(WHITE);
      display.print("Load:  ");
      display.setTextColor(GREEN, BLACK);
      display.print(graph_array_cpu_time.value());
    }
  }
}
