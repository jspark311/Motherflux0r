/**
* This uApp is carved up according to sensor.
*/

#include <CppPotpourri.h>
#include <SensorFilter.h>
#include <GPSWrapper.h>
#include "SensorGlue.h"
#include "uApp.h"
#include "Motherflux0r.h"

extern WakeLock* wakelock_tof;
extern WakeLock* wakelock_mag;
extern WakeLock* wakelock_lux;
extern WakeLock* wakelock_imu;
extern WakeLock* wakelock_grideye;
extern WakeLock* wakelock_uv;
extern WakeLock* wakelock_baro;
extern WakeLock* wakelock_gps;


// Thermopile constants
const float THERM_TEMP_MAX = 150.0;
const float THERM_TEMP_MIN = 0.0;

static float    therm_midpoint_lock = 0.0;    // GidEye
static DataVis current_data_vis = DataVis::TEXT;


uAppTricorder::uAppTricorder() : uApp("Tricorder", (Image*) &display) {}

uAppTricorder::~uAppTricorder() {}



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
int8_t uAppTricorder::_lc_on_preinit() {
  int8_t ret = 1;
  FB->fill(BLACK);
  redraw_app_window();
  if (nullptr != wakelock_tof) {   wakelock_tof->acquire();   }
  if (nullptr != wakelock_mag) {   wakelock_mag->acquire();   }
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppTricorder::_lc_on_active() {
  int8_t ret = 0;
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppTricorder::_lc_on_teardown() {
  int8_t ret = 1;
  if (nullptr != wakelock_tof) {   wakelock_tof->release();   }
  if (nullptr != wakelock_mag) {   wakelock_mag->release();   }
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppTricorder::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppTricorder::_process_user_input() {
  int8_t ret = 1;

  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    if (diff & 0x0001) {   // Cancel
      // Nullify the deepest tree specifier. If none are defined, interpret a
      //   cancel press as a command to return to APP_SELECT.
      if (_buttons_pending & 0x0001) {
        if (UAPP_MODAL_NONE != _modal_id) {
          _modal_id = UAPP_MODAL_NONE;
          ret = 1;
        }
        else {
          if (nullptr != wakelock_tof) {   wakelock_tof->release();   }
          if (nullptr != wakelock_mag) {   wakelock_mag->release();   }
          uApp::setAppActive(AppID::APP_SELECT);
          ret = -1;
        }
      }
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
  }

  if (UAPP_MODAL_NONE != _modal_id) {
    switch (_modal_id) {
      case UAPP_MODAL_TRICORDER_MAG:         ret = _pui_magnetometer();   break;
      case UAPP_MODAL_TRICORDER_IMU:         ret = _pui_imu();            break;
      case UAPP_MODAL_TRICORDER_GPS:         ret = _pui_gps();            break;
      case UAPP_MODAL_TRICORDER_THERMOPILE:  ret = _pui_thermal_field();  break;
      case UAPP_MODAL_TRICORDER_LIGHT:       ret = _pui_photometry();     break;
      case UAPP_MODAL_TRICORDER_ATMO:        ret = _pui_baro();           break;
      case UAPP_MODAL_TRICORDER_RANGING:     ret = _pui_tof();            break;
      default:  break;
    }
  }
  else {
    if (_slider_current != _slider_pending) {
      FB->fill(BLACK);
      FB->setTextSize(0);
      FB->setCursor(0, 0);
      if (_slider_pending <= 7) {
        _modal_id = UAPP_MODAL_TRICORDER_ATMO;
        if (_render_lock_range()) {
          FB->setTextColor(0x03E0, BLACK);
          FB->writeString("Pressure (Pa)");
        }
        else {
          FB->setTextColor(0x3EE3, BLACK);
          FB->writeString("RelH%");
          FB->setTextColor(WHITE, BLACK);
          FB->writeString(" / ");
          FB->setTextColor(0x83D0, BLACK);
          FB->writeString("Temp  ");
        }
      }
      else if (_slider_pending <= 15) {
        _modal_id = UAPP_MODAL_TRICORDER_IMU;
        FB->setTextColor(YELLOW, BLACK);
        FB->writeString("IMU");
      }
      else if (_slider_pending <= 22) {
        _modal_id = UAPP_MODAL_TRICORDER_RANGING;
        FB->setTextColor(0x8235, BLACK);
        FB->writeString("Distance (mm)");
      }
      else if (_slider_pending <= 30) {
        _modal_id = UAPP_MODAL_TRICORDER_LIGHT;
        if (_render_lock_range()) {
          FB->setTextColor(YELLOW, BLACK);
          FB->writeString("ANA");
          FB->setTextColor(WHITE, BLACK);
          FB->writeString(" / ");
          FB->setTextColor(0xF140, BLACK);
          FB->writeString("Lux");
          FB->setTextColor(WHITE, BLACK);
          FB->writeString(" / ");
          FB->setTextColor(RED, BLACK);
          FB->writeString("IR");
        }
        else {
          FB->setTextColor(0x791F, BLACK);
          FB->writeString("UVa");
          FB->setTextColor(WHITE, BLACK);
          FB->writeString(" / ");
          FB->setTextColor(0xF80F, BLACK);
          FB->writeString("UVb");
          FB->setTextColor(WHITE, BLACK);
          FB->writeString(" / ");
          FB->setTextColor(MAGENTA, BLACK);
          FB->writeString("UVI");
        }
      }
      else if (_slider_pending <= 37) {
        FB->setTextColor(WHITE, BLACK);
        FB->writeString("Batt ");
        FB->setTextColor(COLOR_BATT_VOLTAGE, BLACK);
        FB->writeString("V");
        FB->setTextColor(WHITE, BLACK);
        FB->writeString(" / ");
        FB->setTextColor(COLOR_BATT_CURRENT, BLACK);
        FB->writeString("I");
      }
      else if (_slider_pending <= 45) {
        _modal_id = UAPP_MODAL_TRICORDER_GPS;
        redraw_app_window();
      }
      else if (_slider_pending <= 52) {
        _modal_id = UAPP_MODAL_TRICORDER_THERMOPILE;
      }
      else {
        _modal_id = UAPP_MODAL_TRICORDER_MAG;
      }
      _slider_current = _slider_pending;
    }
  }

  if (_buttons_current != _buttons_pending) _buttons_current = _buttons_pending;
  if (_slider_current != _slider_pending)   _slider_current  = _slider_pending;
  return ret;
}



/*
* Draws the tricorder app.
*/
void uAppTricorder::_redraw_window() {
  StringBuilder tmp_val_str;
  switch (_modal_id) {
    case UAPP_MODAL_TRICORDER_MAG:         _render_magnetometer();   return;
    case UAPP_MODAL_TRICORDER_IMU:         _render_imu();            return;
    case UAPP_MODAL_TRICORDER_GPS:         _render_gps();            return;
    case UAPP_MODAL_TRICORDER_THERMOPILE:  _render_thermal_field();  return;
    case UAPP_MODAL_TRICORDER_LIGHT:       _render_photometry();     return;
    case UAPP_MODAL_TRICORDER_ATMO:        _render_baro();           return;
    case UAPP_MODAL_TRICORDER_RANGING:     _render_tof();            return;
    case UAPP_MODAL_NONE:
    default:  break;
  }

  // if (graph_array_batt_voltage.dirty() && graph_array_batt_current.dirty()) {
  //   draw_graph_obj(
  //     0, 10, 96, 37, COLOR_BATT_VOLTAGE, COLOR_BATT_CURRENT,
  //     true, _cluttered_display(), _render_text_value(),
  //     &graph_array_batt_voltage, &graph_array_batt_current
  //   );
  // }
}


int8_t uAppTricorder::_pui_magnetometer() {
  int8_t ret = 1;
  if (_slider_current != _slider_pending) {
    if (_slider_pending <= 7) {
    }
    else if (_slider_pending <= 15) {
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
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;
    // bool up_pressed   = (_buttons_current & 0x0002);
    // bool down_pressed = (_buttons_current & 0x0010);
    // _button_pressed_up(!down_pressed && up_pressed);
    // _button_pressed_dn(down_pressed && !up_pressed);
    if (diff & 0x0002) {   // Up
      if (_buttons_pending & 0x0002) {
        _cluttered_display(!_cluttered_display());
      }
    }
    if (diff & 0x0004) {   // Accept
      if (_buttons_pending & 0x0004) {   // Interpret an ok press as an app selection.
      }
    }
    if (diff & 0x0008) {   // Right
      if (_buttons_pending & 0x0008) {   // Text of actual value.
        _render_text_value(!_render_text_value());
      }
    }
    if (diff & 0x0010) {   // Down
      if (_buttons_pending & 0x0010) {
        _render_lock_range(!_render_lock_range());
      }
    }
    if (diff & 0x0020) {   // Left
      if (_buttons_pending & 0x0020) {   // Previous selection.
      }
    }
    _buttons_current = _buttons_pending;
    ret = 1;
  }
  return ret;
}


int8_t uAppTricorder::_pui_thermal_field() {
  int8_t ret = 1;
  return ret;
}


int8_t uAppTricorder::_pui_imu() {
  int8_t ret = 1;
  return ret;
}


int8_t uAppTricorder::_pui_baro() {
  int8_t ret = 1;
  return ret;
}


int8_t uAppTricorder::_pui_gps() {
  int8_t ret = 1;
  return ret;
}


int8_t uAppTricorder::_pui_photometry() {
  int8_t ret = 1;
  return ret;
}


int8_t uAppTricorder::_pui_tof() {
  int8_t ret = 1;
  return ret;
}




void uAppTricorder::_render_magnetometer() {
  const uint8_t TOP_MARGIN     = 10;
  const uint8_t ELEMENT_MARGIN = 1;
  const uint8_t COMPASS_SIZE   = 34;
  const uint8_t GRAPH_HEIGHT   = 53;
  const uint8_t GRAPH_WIDTH    = _cluttered_display() ? (96-(COMPASS_SIZE+ELEMENT_MARGIN)) : 96;
  const uint8_t GRAPH_H_OFFSET = 96 - GRAPH_WIDTH;
  StringBuilder tmp_val_str;

  if (_cluttered_display()) {
    if (compass.dataReady()) {
      float bearing_north = 0.0;
      float bearing_mag = 0.0;
      compass.getBearing(HeadingType::TRUE_NORTH, &bearing_north);
      compass.getBearing(HeadingType::MAGNETIC_NORTH, &bearing_mag);
      draw_compass(0, TOP_MARGIN, COMPASS_SIZE, COMPASS_SIZE, false, _render_text_value(), bearing_mag, bearing_north);
      FB->setTextColor(WHITE, BLACK);
      FB->setCursor(0, 56);
      tmp_val_str.clear();
      tmp_val_str.concatf("%.1ff  ", bearing_mag);
      FB->writeString(&tmp_val_str);
    }
  }
  if (graph_array_mag_strength_z.dirty()) {
    Vector3f* mag_vect_ptr = mag_vect.getData();
    FB->setTextColor(WHITE, BLACK);
    FB->setCursor(0, 0);
    tmp_val_str.clear();
    tmp_val_str.concatf("%.3f uT   ", mag_vect_ptr->length());
    FB->writeString(&tmp_val_str);
    draw_graph_obj(
      GRAPH_H_OFFSET, TOP_MARGIN, GRAPH_WIDTH, GRAPH_HEIGHT,
      COLOR_X_AXIS, COLOR_Y_AXIS, COLOR_Z_AXIS,
      true, false, _render_text_value(),
      &graph_array_mag_strength_x, &graph_array_mag_strength_y, &graph_array_mag_strength_z
    );
  }
}


void uAppTricorder::_render_thermal_field() {
  StringBuilder tmp_val_str;
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
      uint16_t color = ((uint16_t) pix_intensity) << ((therm_pixels[i] <= MIDPOINT_T) ? 8 : 3);
      //if (therm_pixels[i] < MIDPOINT_T) {
      //}
      //else if (therm_pixels[i] > MIDPOINT_T) {
      //}
      FB->fillRect(x, y, PIXEL_SIZE, PIXEL_SIZE, color);
    }

    FB->setTextSize(0);
    FB->setCursor(TEXT_OFFSET, 0);
    FB->setTextColor(RED, BLACK);
    tmp_val_str.concatf("%.2f", therm_field_max);
    FB->writeString(&tmp_val_str);
    tmp_val_str.clear();

    FB->setCursor(TEXT_OFFSET, 12);
    FB->setTextColor(WHITE, BLACK);
    FB->writeString("ABS (C)");

    FB->setCursor(TEXT_OFFSET, (PIXEL_SIZE*8)-7);
    FB->setTextColor(BLUE, BLACK);
    tmp_val_str.concatf("%.2f", therm_field_min);
    FB->writeString(&tmp_val_str);
    tmp_val_str.clear();

    if (!_render_text_value()) {
      FB->setCursor(0, TEXT_OFFSET);
      FB->setTextColor(WHITE);
      FB->writeString("Mean:  ");
      FB->setTextColor(RED, BLACK);
      tmp_val_str.concatf("%.2f\n", graph_array_therm_mean.value());
      FB->writeString(&tmp_val_str);
      tmp_val_str.clear();

      FB->setTextColor(WHITE);
      FB->writeString("Range: ");
      FB->setTextColor(RED, BLACK);
      tmp_val_str.concatf("%.2f\n", (therm_field_max - therm_field_min));
      FB->writeString(&tmp_val_str);
      tmp_val_str.clear();

      FB->setTextColor(WHITE);
      FB->writeString("STDEV: ");
      FB->setTextColor(RED, BLACK);
      tmp_val_str.concatf("%.2f", graph_array_therm_frame.stdev());
      FB->writeString(&tmp_val_str);
      tmp_val_str.clear();
    }
    else {
      float pix_deviation = abs(MIDPOINT_T - graph_array_therm_mean.value());
      uint8_t pix_intensity = BINSIZE_T * (pix_deviation / (therm_field_max - MIDPOINT_T));
      uint16_t color = ((uint16_t) pix_intensity) << ((graph_array_therm_mean.value() <= MIDPOINT_T) ? 8 : 3);
      draw_graph_obj(
        0, TEXT_OFFSET, 95, 63-TEXT_OFFSET, color,
        true, true, true,
        &graph_array_therm_mean
      );
    }
  }
}


void uAppTricorder::_render_imu() {
  StringBuilder tmp_val_str;
  //StringBuilder temp0;
  //temp0.concatf("uT<%.2f, %.2f, %.2f>", imu.accX(), imu.accY(), imu.accZ());
  //FB->setTextColor(WHITE, BLACK);
  //FB->writeString(&temp0);
  //imu.gyrX();
  //imu.gyrY();
  //imu.gyrZ();
  //imu.magX();
  //imu.magY();
  //imu.magZ();
  //imu.temp();
}


void uAppTricorder::_render_baro() {
  StringBuilder tmp_val_str;
  if (_render_lock_range()) {
    if (graph_array_humidity.dirty()) {
      float altitude  = baro.Altitude(baro.pres());
      float dew_point = baro.DewPoint(baro.temp(), baro.hum());
      float sea_level = baro.EquivalentSeaLevelPressure(altitude, baro.temp(), baro.pres());
      draw_graph_obj(
        0, 10, 96, 36, 0x03E0,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_pressure
      );
      FB->setTextSize(0);
      FB->setCursor(0, 47);
      FB->setTextColor(WHITE, BLACK);
      FB->writeString("Alt: ");
      FB->setTextColor(GREEN, BLACK);
      tmp_val_str.clear();
      tmp_val_str.concatf("%.3f ft", altitude);
      FB->writeString(&tmp_val_str);
      FB->setTextColor(WHITE, BLACK);
      FB->writeString("Dew Point: ");
      FB->setTextColor(0x03E0, BLACK);
      tmp_val_str.clear();
      tmp_val_str.concatf("%.2fC", dew_point);
      FB->writeString(&tmp_val_str);
    }
  }
  else {
    if (graph_array_air_temp.dirty()) {
      draw_graph_obj(
        0, 10, 96, 37, 0x83D0, 0x3EE3,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_air_temp, &graph_array_humidity
      );
    }
  }
}


void uAppTricorder::_render_gps() {
  StringBuilder tmp_val_str;
}


void uAppTricorder::_render_photometry() {
  StringBuilder tmp_val_str;
  if (_render_lock_range()) {
    graph_array_ana_light.feedFilter(analogRead(ANA_LIGHT_PIN) / 1024.0);
    draw_graph_obj(
      0, 10, 96, 53, YELLOW, 0xF140, RED,
      true, _cluttered_display(), _render_text_value(),
      &graph_array_ana_light, &graph_array_visible, &graph_array_broad_ir
    );
  }
  else {
    if (graph_array_uva.dirty()) {
      draw_graph_obj(
        0, 10, 96, 53, 0x791F, 0xF80F, MAGENTA,
        true, _cluttered_display(), _render_text_value(),
        &graph_array_uva, &graph_array_uvb, &graph_array_uvi
      );
    }
  }
}


void uAppTricorder::_render_tof() {
  StringBuilder tmp_val_str;
  if (graph_array_time_of_flight.dirty()) {
    draw_graph_obj(
      0, 10, 96, 53, 0x8235,
      true, _cluttered_display(), _render_text_value(),
      &graph_array_time_of_flight
    );
  }
}
