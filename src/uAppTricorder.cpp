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

/* Sensor representations... */
extern DRV425 magneto;
extern TMP102 tmp102;
extern GridEYE grideye;
extern VEML6075 uv;
extern ICM_20948_SPI imu;
extern TSL2561 tsl2561;
extern BME280I2C baro;


extern SensorFilter<float> graph_array_pressure;
extern SensorFilter<float> graph_array_humidity;
extern SensorFilter<float> graph_array_air_temp;
extern SensorFilter<float> graph_array_psu_temp;
extern SensorFilter<float> graph_array_uva;
extern SensorFilter<float> graph_array_uvb;
extern SensorFilter<float> graph_array_uvi;
extern SensorFilter<float> graph_array_ana_light;
extern SensorFilter<float> graph_array_visible;
extern SensorFilter<float> graph_array_therm_mean;
extern SensorFilter<float> graph_array_therm_frame;
extern SensorFilter<float> graph_array_mag_confidence;



void redraw_tricorder_window();

// Thermopile constants
const float THERM_TEMP_MAX = 150.0;
const float THERM_TEMP_MIN = 0.0;

static float    therm_midpoint_lock = 0.0;    // GidEye
static DataVis current_data_vis = DataVis::TEXT;



uAppTricorder::uAppTricorder() : uApp("Tricorder") {
}


uAppTricorder::~uAppTricorder(){
}



int8_t uAppTricorder::refresh() {
  int8_t ret = 0;
  _stopwatch.markStart();
  if (uApp::drawnApp() != uApp::appActive()) {
    uApp::redraw_app_window("Tricorder", 0, 0);
  }
  switch (_process_user_input()) {
    case 2:
    case 1:
    case 0:
      _redraw_tricorder_window();
    default:
      break;
  }
  _stopwatch.markStop();
  return ret;
}



int8_t uAppTricorder::_process_user_input() {
  int8_t ret = 0;

  if (_slider_current != _slider_pending) {
    if (_slider_pending <= 45) {
      uApp::redraw_app_window("Tricorder", 0, 0);
    }
    else {
      //display.fillRect(0, 11, display.width()-1, display.height()-12, BLACK);
      display.fillScreen(BLACK);
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
void uAppTricorder::_redraw_tricorder_window() {
  if (_slider_pending <= 7) {
    // Baro
    if (graph_array_humidity.dirty()) {
      float altitude  = baro.Altitude(baro.pres());
      float dew_point = baro.DewPoint(baro.temp(), baro.hum());
      float sea_level = baro.EquivalentSeaLevelPressure(altitude, baro.temp(), baro.pres());
      draw_graph_obj(
        0, 10, 96, 37, 0x03E0,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_humidity
      );
      display.setTextSize(0);
      display.setCursor(0, 48);
      display.setTextColor(WHITE);
      display.print("Alt: ");
      display.setTextColor(GREEN, BLACK);
      display.print(altitude);
      display.println("m");
      display.setTextColor(WHITE);
      display.print("Humidity: ");
      display.setTextColor(0x03E0, BLACK);
      display.print(graph_array_humidity.value());
      display.println("%");
    }
  }
  else if (_slider_pending <= 15) {
    // Baro
    if (graph_array_air_temp.dirty()) {
      draw_graph_obj(
        0, 10, 48, 37, 0x83D0,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_air_temp
      );
    }
    if (graph_array_pressure.dirty()) {
      draw_graph_obj(
        48, 10, 48, 37, 0xFE00,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_pressure
      );
    }
    display.setTextSize(0);
    display.setCursor(0, 48);
    display.setTextColor(WHITE);
    display.print("Pres: ");
    display.setTextColor(0xFE00, BLACK);
    display.print(graph_array_pressure.value());
    display.println("Pa");
    display.setTextColor(WHITE);
    display.print("Air Temp: ");
    display.setTextColor(0x83D0, BLACK);
    display.print(graph_array_air_temp.value());
    display.println("C");
  }
  else if (_slider_pending <= 22) {
    // TSL2561
    if (graph_array_visible.dirty()) {
      draw_graph_obj(
        0, 10, 96, 45, 0xF100,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_visible
      );
      display.setTextSize(0);
      display.setCursor(0, 56);
      display.setTextColor(WHITE);
      display.print("Lux:  ");
      display.setTextColor(0xF100, BLACK);
      display.print(graph_array_visible.value());
    }
  }
  else if (_slider_pending <= 30) {
    // Analog light sensor
    graph_array_ana_light.feedFilter(analogRead(ANA_LIGHT_PIN) / 1024.0);
    draw_graph_obj(
      0, 10, 96, 45, 0xFE00,
      true, _cluttered_display(), _render_text_value(),
      &graph_array_ana_light
    );
    display.setTextSize(0);
    display.setCursor(0, 56);
    display.setTextColor(WHITE);
    display.print("Light:  ");
    display.setTextColor(GREEN, BLACK);
    display.print(graph_array_ana_light.value());
  }
  else if (_slider_pending <= 37) {
    if (_render_lock_range()) {
      // UVI
      if (graph_array_uvi.dirty()) {
        draw_graph_obj(
          0, 10, 96, 45, 0xF81F,
          true, _cluttered_display(), _render_text_value(),
          &graph_array_uvi
        );
        display.setTextSize(0);
        display.setCursor(0, 56);
        display.setTextColor(WHITE);
        display.print("UVI:  ");
        display.setTextColor(0xF81F, BLACK);
        display.print(graph_array_uvi.value());
      }
    }
    else {
      if (graph_array_uva.dirty()) {
        draw_graph_obj(
          0, 10, 48, 45, 0x781F,
          true, _cluttered_display(), _render_text_value(),
          &graph_array_uva
        );
      }
      if (graph_array_uvb.dirty()) {
        draw_graph_obj(
          48, 10, 48, 45, 0xF80F,
          true, _cluttered_display(), _render_text_value(),
          &graph_array_uvb
        );
      }
      display.setTextSize(0);
      display.setCursor(0, 56);
      display.setTextColor(WHITE);
      display.print("UVA/B: ");
      display.setTextColor(0x781F, BLACK);
      display.print(graph_array_uva.value());
      display.setTextColor(WHITE, BLACK);
      display.print(" / ");
      display.setTextColor(0xF80F, BLACK);
      display.print(graph_array_uvb.value());
    }
  }
  else if (_slider_pending <= 45) {
    // IMU
    display.setCursor(0, 11);
    display.setTextColor(YELLOW, BLACK);
    display.print("IMU");
    StringBuilder temp0;
    temp0.concatf("uT<%.2f, %.2f, %.2f>", imu.accX(), imu.accY(), imu.accZ());
    display.setTextColor(WHITE, BLACK);
    display.println((char*) temp0.string());
    //imu.gyrX();
    //imu.gyrY();
    //imu.gyrZ();
    //imu.magX();
    //imu.magY();
    //imu.magZ();
    //imu.temp();
  }
  else if (_slider_pending <= 52) {
    display.setCursor(0, 0);
    display.setTextColor(0xFC00, BLACK);
    display.print("Magnetometer");
    if (!_render_lock_range()) {
      display.setTextColor(WHITE, BLACK);
      Vector3f64* mag_vect = magneto.getFieldVector();
      float bearing_north = 0.0;
      float bearing_mag = 0.0;
      magneto.getBearing(HeadingType::TRUE_NORTH, &bearing_north);
      magneto.getBearing(HeadingType::MAGNETIC_NORTH, &bearing_mag);
      draw_compass(0, 11, 44, 44, false, _render_text_value(), bearing_mag, bearing_north);
      display.setCursor(0, 57);
      display.print(mag_vect->length());
      display.print(" uT");
    }
    else {
      draw_graph_obj(
        0, 10, 96, 45, 0xF81F,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_mag_confidence
      );
      display.setTextSize(0);
      display.setCursor(0, 56);
      display.setTextColor(WHITE);
      display.print("Confidence: ");
      display.setTextColor(0xFC00, BLACK);
      display.print(graph_array_mag_confidence.value());
    }
  }
  else {    // Thermopile
    if (graph_array_therm_mean.dirty()) {
      const uint8_t PIXEL_SIZE  = 4;
      const uint8_t TEXT_OFFSET = (PIXEL_SIZE*8)+5;
      const float TEMP_RANGE = THERM_TEMP_MAX - THERM_TEMP_MIN;
      const float BINSIZE_T  = TEMP_RANGE / (PIXEL_SIZE * 8);  // Space of display gives scale size.
      float* therm_pixels = graph_array_therm_frame.memPtr();
      float  therm_field_min = graph_array_therm_frame.minValue();
      float  therm_field_max = graph_array_therm_frame.maxValue();
      bool lock_range_to_current = _cluttered_display();
      bool lock_range_to_absolute = _render_lock_range();
      if (!(lock_range_to_absolute | lock_range_to_current)) {
        therm_midpoint_lock = therm_field_max - ((therm_field_max - therm_field_min) / 2);
      }
      else if (lock_range_to_absolute) {
        therm_field_max = THERM_TEMP_MAX;
        therm_field_min = THERM_TEMP_MIN;
      }
      else {  // Lock to current range. Use the filtered mean so that it drifts.
        therm_field_max = graph_array_therm_mean.value() + therm_midpoint_lock;
        therm_field_min = graph_array_therm_mean.value() - therm_midpoint_lock;
      }
      const float MIDPOINT_T = lock_range_to_absolute ? (TEMP_RANGE / 2.0) : therm_midpoint_lock;

      for (uint8_t i = 0; i < 64; i++) {
        uint x = (i & 0x07) * PIXEL_SIZE;
        uint y = (i >> 3) * PIXEL_SIZE;
        float pix_deviation = abs(MIDPOINT_T - therm_pixels[i]);
        uint8_t pix_intensity = BINSIZE_T * (pix_deviation / (therm_field_max - MIDPOINT_T));
        uint16_t color = (therm_pixels[i] <= MIDPOINT_T) ? pix_intensity : (pix_intensity << 11);
        display.fillRect(x, y, PIXEL_SIZE, PIXEL_SIZE, color);
      }
      display.setTextSize(0);
      display.setCursor(TEXT_OFFSET, 0);
      display.setTextColor(RED, BLACK);
      display.print(therm_field_max);

      display.setCursor(TEXT_OFFSET, 12);
      display.setTextColor(WHITE, BLACK);
      display.print("ABS (C)");

      display.setCursor(TEXT_OFFSET, (PIXEL_SIZE*8)-7);
      display.setTextColor(BLUE, BLACK);
      display.print(therm_field_min);

      display.setCursor(0, TEXT_OFFSET);
      display.setTextColor(WHITE);
      display.print("Mean:  ");
      display.setTextColor(RED, BLACK);
      display.println(graph_array_therm_mean.value());

      display.setTextColor(WHITE);
      display.print("Range: ");
      display.setTextColor(RED, BLACK);
      display.println(therm_field_max - therm_field_min);

      display.setTextColor(WHITE);
      display.print("STDEV: ");
      display.setTextColor(RED, BLACK);
      display.println(graph_array_therm_frame.stdevValue());
    }
  }
}
