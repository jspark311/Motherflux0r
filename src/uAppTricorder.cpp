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



int8_t uAppTricorder::_process_user_input() {
  int8_t ret = 0;

  if (_slider_current != _slider_pending) {
    if (_slider_pending <= 7) {
      display.fillScreen(BLACK);
      display.setTextSize(0);
      display.setCursor(0, 0);
      display.setTextColor(0x03E0, BLACK);
      display.print("Pressure (Pa)");
    }
    else if (_slider_pending <= 15) {
      display.fillScreen(BLACK);
      display.setTextSize(0);
      display.setCursor(0, 0);
      display.setTextColor(0x3EE3, BLACK);
      display.print("RelH%");
      display.setTextColor(WHITE, BLACK);
      display.print(" / ");
      display.setTextColor(0x83D0, BLACK);
      display.print("Temp  ");
    }
    else if (_slider_pending <= 22) {
      redraw_app_window();
    }
    else if (_slider_pending <= 30) {
      display.fillScreen(BLACK);
      display.setTextSize(0);
      display.setCursor(0, 0);
      display.setTextColor(0xF710, BLACK);
      display.print("ANA");
      display.setTextColor(WHITE, BLACK);
      display.print(" / ");
      display.setTextColor(0xF140, BLACK);
      display.print("Lux");
      display.setTextColor(WHITE, BLACK);
      display.print(" / ");
      display.setTextColor(0xF81F, BLACK);
      display.print("UVI");
    }
    else if (_slider_pending <= 37) {
      display.fillScreen(BLACK);
      display.setTextSize(0);
      display.setCursor(0, 0);
      display.setTextColor(0x781F, BLACK);
      display.print("UVa");
      display.setTextColor(WHITE, BLACK);
      display.print(" / ");
      display.setTextColor(0xF80F, BLACK);
      display.print("UVb       ");
    }
    else if (_slider_pending <= 45) {
      redraw_app_window();
    }
    else if (_slider_pending <= 52) {
      display.fillScreen(BLACK);
      display.setCursor(0, 0);
      display.setTextColor(0x071F, BLACK);
      display.print("Magnetometer  ");
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
void uAppTricorder::_redraw_window() {
  if (_slider_current <= 7) {
    // Baro
    if (graph_array_humidity.dirty()) {
      float altitude  = baro.Altitude(baro.pres());
      float dew_point = baro.DewPoint(baro.temp(), baro.hum());
      float sea_level = baro.EquivalentSeaLevelPressure(altitude, baro.temp(), baro.pres());
      draw_graph_obj(
        0, 10, 96, 37, 0x03E0,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_pressure
      );
      display.setTextSize(0);
      display.setCursor(0, 48);
      display.setTextColor(WHITE, BLACK);
      display.print("Alt: ");
      display.setTextColor(GREEN, BLACK);
      display.print(altitude);
      display.println("m");
      display.setTextColor(WHITE, BLACK);
      display.print("Dew Point: ");
      display.setTextColor(0x03E0, BLACK);
      display.print(dew_point);
      display.println("C");
    }
  }
  else if (_slider_current <= 15) {
    // Baro
    if (graph_array_air_temp.dirty()) {
      draw_graph_obj(
        0, 10, 96, 54, 0x83D0, 0x3EE3,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_air_temp, &graph_array_humidity
      );
    }
  }
  else if (_slider_current <= 22) {
  }
  else if (_slider_current <= 30) {
    // Light
    graph_array_ana_light.feedFilter(analogRead(ANA_LIGHT_PIN) / 1024.0);
    draw_graph_obj(
      0, 10, 96, 54, 0xF710, 0xF140, 0xF81F,
      true, _cluttered_display(), _render_text_value(),
      &graph_array_ana_light, &graph_array_visible, &graph_array_uvi
    );
  }
  else if (_slider_current <= 37) {
    if (graph_array_uva.dirty()) {
      draw_graph_obj(
        0, 10, 96, 54, 0x781F, 0xF80F,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_uva, &graph_array_uvb
      );
    }
  }
  else if (_slider_current <= 45) {
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
  else if (_slider_current <= 52) {
    if (graph_array_mag_confidence.dirty()) {
      display.setTextColor(WHITE, BLACK);
      Vector3f64* mag_vect = magneto.getFieldVector();
      float bearing_north = 0.0;
      float bearing_mag = 0.0;
      magneto.getBearing(HeadingType::TRUE_NORTH, &bearing_north);
      magneto.getBearing(HeadingType::MAGNETIC_NORTH, &bearing_mag);
      draw_compass(0, 10, 45, 45, false, _render_text_value(), bearing_mag, bearing_north);
      display.setCursor(0, 57);
      display.print(mag_vect->length());
      display.print(" uT");
      draw_graph_obj(
        46, 10, 50, 55, 0x071F,
        true, false, _render_text_value(),
        &graph_array_mag_confidence
      );
    }
  }
  else {    // Thermopile
    if (graph_array_therm_mean.dirty()) {
      const uint8_t FIELD_SIZE = 32;
      const uint8_t TEXT_OFFSET = FIELD_SIZE+5;
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

      draw_data_square_field(0, 0, 32, 32, &therm_field_min, &therm_field_max, 0, &graph_array_therm_frame);

      display.setTextSize(0);
      display.setCursor(TEXT_OFFSET, 0);
      display.setTextColor(RED, BLACK);
      display.print(therm_field_max);

      display.setCursor(TEXT_OFFSET, 12);
      display.setTextColor(WHITE, BLACK);
      display.print("ABS (C)");

      display.setCursor(TEXT_OFFSET, FIELD_SIZE-7);
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
