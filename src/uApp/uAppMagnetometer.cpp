/**
* This uApp is carved up according to sensor.
*/
#include "../Motherflux0r.h"
#include "../uApp.h"

#define UAPP_SUBMODAL_TRICORDER_MAG_GRAPH  0x00
#define UAPP_SUBMODAL_TRICORDER_COMPASS    0x01
#define UAPP_SUBMODAL_TRICORDER_MAG_STATS  0x02
#define UAPP_SUBMODAL_TRICORDER_MAG_VECT   0x03
#define UAPP_SUBMODAL_TRICORDER_MAG_CONF   0x04


uAppMagnetometer::uAppMagnetometer() :
  uApp("Magnetometer", (Image*) &display),
  _graph(display.x(), 53) {}

uAppMagnetometer::~uAppMagnetometer() {}


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
int8_t uAppMagnetometer::_lc_on_preinit() {
  int8_t ret = 1;
  FB->fill(BLACK);

  _graph.fg_color = 0xFFFFFFFF;

  _graph.trace0.color        = COLOR_X_AXIS;
  _graph.trace0.autoscale_x  = false;
  _graph.trace0.autoscale_y  = true;
  _graph.trace0.show_x_range = false;
  _graph.trace0.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  _graph.trace0.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.

  _graph.trace1.color        = COLOR_Y_AXIS;
  _graph.trace1.autoscale_x  = false;
  _graph.trace1.autoscale_y  = true;
  _graph.trace1.show_x_range = false;
  _graph.trace1.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  _graph.trace1.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.

  _graph.trace2.color        = COLOR_Z_AXIS;
  _graph.trace2.autoscale_x  = false;
  _graph.trace2.autoscale_y  = true;
  _graph.trace2.show_x_range = false;
  _graph.trace2.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  _graph.trace2.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.

  redraw_app_window();
  //if (nullptr != wakelock_mag) {   wakelock_mag->acquire();   }
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppMagnetometer::_lc_on_active() {
  int8_t ret = 0;
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppMagnetometer::_lc_on_teardown() {
  int8_t ret = 1;
  //if (nullptr != wakelock_mag) {   wakelock_mag->release();   }
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppMagnetometer::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppMagnetometer::_process_user_input() {
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
          //if (nullptr != wakelock_mag) {   wakelock_mag->release();   }
          uApp::setAppActive(AppID::APP_SELECT);
          ret = -1;
        }
      }
      _buttons_current |= 0x0001;
    }
  }

  if (_slider_current != _slider_pending) {
    FB->fill(BLACK);
    if (_slider_pending <= 7) {
      _modal_id = UAPP_SUBMODAL_TRICORDER_MAG_GRAPH;
    }
    else if (_slider_pending <= 15) {
      _modal_id = UAPP_SUBMODAL_TRICORDER_COMPASS;
    }
    else if (_slider_pending <= 22) {
      _modal_id = UAPP_SUBMODAL_TRICORDER_MAG_STATS;
    }
    else if (_slider_pending <= 30) {
      _modal_id = UAPP_SUBMODAL_TRICORDER_MAG_CONF;
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
    FB->fill(BLACK);
    uint16_t diff = _buttons_current ^ _buttons_pending;
    // bool up_pressed   = (_buttons_current & 0x0002);
    // bool down_pressed = (_buttons_current & 0x0010);
    // _button_pressed_up(!down_pressed && up_pressed);
    // _button_pressed_dn(down_pressed && !up_pressed);
    if (diff & 0x0002) {   // Up
      if (_buttons_pending & 0x0002) {
        switch (_modal_id) {
          case UAPP_SUBMODAL_TRICORDER_MAG_CONF:
            mag_adc.setOversamplingRatio((MCP356xOversamplingRatio) ((((uint8_t) mag_adc.getOversamplingRatio()) + 1) % 16));
            break;
          case UAPP_SUBMODAL_TRICORDER_MAG_STATS:
          case UAPP_SUBMODAL_TRICORDER_MAG_GRAPH:
          case UAPP_SUBMODAL_TRICORDER_COMPASS:
          case UAPP_SUBMODAL_TRICORDER_MAG_VECT:
          default:
            _cluttered_display(!_cluttered_display());
            break;
        }
      }
    }
    if (diff & 0x0004) {   // Accept
      if (_buttons_pending & 0x0004) {
        switch (_modal_id) {
          case UAPP_SUBMODAL_TRICORDER_MAG_CONF:
            switch (magneto.bandwidth()) {
              case DRV425Bandwidth::BW0:   magneto.bandwidth(DRV425Bandwidth::BW1);  break;
              case DRV425Bandwidth::BW1:   magneto.bandwidth(DRV425Bandwidth::BW0);  break;
            }
            break;
          case UAPP_SUBMODAL_TRICORDER_MAG_STATS:
          case UAPP_SUBMODAL_TRICORDER_MAG_GRAPH:
          case UAPP_SUBMODAL_TRICORDER_COMPASS:
          case UAPP_SUBMODAL_TRICORDER_MAG_VECT:
          default:
            break;
        }
      }
    }
    if (diff & 0x0008) {   // Right
      if (_buttons_pending & 0x0008) {   // Text of actual value.
        switch (_modal_id) {
          case UAPP_SUBMODAL_TRICORDER_MAG_CONF:
            mag_filter.windowSize(strict_min((uint32_t) 360, mag_filter.windowSize() + 60));
            break;
          case UAPP_SUBMODAL_TRICORDER_MAG_STATS:
          case UAPP_SUBMODAL_TRICORDER_MAG_GRAPH:
          case UAPP_SUBMODAL_TRICORDER_COMPASS:
          case UAPP_SUBMODAL_TRICORDER_MAG_VECT:
          default:
            _render_text_value(!_render_text_value());
            break;
        }
      }
    }
    if (diff & 0x0010) {   // Down
      if (_buttons_pending & 0x0010) {
        switch (_modal_id) {
          case UAPP_SUBMODAL_TRICORDER_MAG_CONF:
            mag_adc.setOversamplingRatio((MCP356xOversamplingRatio) ((((uint8_t) mag_adc.getOversamplingRatio()) - 1) % 16));
            break;
          case UAPP_SUBMODAL_TRICORDER_MAG_STATS:
          case UAPP_SUBMODAL_TRICORDER_MAG_GRAPH:
          case UAPP_SUBMODAL_TRICORDER_COMPASS:
          case UAPP_SUBMODAL_TRICORDER_MAG_VECT:
          default:
            _render_lock_range(!_render_lock_range());
            break;
        }
      }
    }
    if (diff & 0x0020) {   // Left
      if (_buttons_pending & 0x0020) {   // Previous selection.
        switch (_modal_id) {
          case UAPP_SUBMODAL_TRICORDER_MAG_CONF:
            mag_filter.windowSize((uint32_t) strict_max((int32_t) 0, (int32_t) mag_filter.windowSize() - 60));
            break;
          case UAPP_SUBMODAL_TRICORDER_MAG_GRAPH:
          case UAPP_SUBMODAL_TRICORDER_COMPASS:
          case UAPP_SUBMODAL_TRICORDER_MAG_STATS:
          case UAPP_SUBMODAL_TRICORDER_MAG_VECT:
          default:
            break;
        }
      }
    }
    _buttons_current = _buttons_pending;
  }

  if (_slider_current != _slider_pending)   _slider_current  = _slider_pending;
  return ret;
}


/*******************************************************************************
* Render functions
*******************************************************************************/

void uAppMagnetometer::_render_mag_unready() {
  StringBuilder tmp_str;
  FB->setTextColor(WHITE, BLACK);
  FB->setCursor(0, 0);
  FB->setTextSize(0);
  FB->writeString("Mag not ready\n");
  FB->setCursor(0, 12);

  FB->setTextColor(GREY, BLACK);
  FB->writeString("Power:       ");
  if (magneto.power()) {  FB->setTextColor(GREEN,  BLACK);  FB->writeString(" On\n");   }
  else {                  FB->setTextColor(0x03E0, BLACK);  FB->writeString("Off\n");  }

  FB->setTextColor(GREY, BLACK);
  FB->writeString("GPIO Init:   ");
  if (sx1503.initialized()) {  FB->setTextColor(GREEN,  BLACK);  FB->writeString("Yes\n");   }
  else {                       FB->setTextColor(0x03E0, BLACK);  FB->writeString(" No\n");  }

  FB->setTextColor(GREY, BLACK);
  FB->writeString("ADC: ");
  uint16_t color = 0x03E0;
  switch (mag_adc.currentState()) {
    case MCP356xState::POST_INIT:
    case MCP356xState::CLK_MEASURE:
    case MCP356xState::CALIBRATION:
    case MCP356xState::USR_CONF:
      color = YELLOW;
      break;
    case MCP356xState::IDLE:
    case MCP356xState::READING:
      color = GREEN;
      break;
    default:
      break;
  }
  FB->setTextColor(color,  BLACK);
  tmp_str.concatf("%11s\n", MCP356x::stateStr(mag_adc.currentState()));
  FB->writeString((char*) tmp_str.string());
  tmp_str.clear();

  FB->setTextColor(GREY, BLACK);
  FB->writeString("Filter:  ");
  if (mag_filter.windowFull()) {
    color = GREEN;
    tmp_str.concat("  Ready\n");
  }
  else {
    color = (0 < mag_filter.totalSamples()) ? YELLOW : GREY;
    tmp_str.concatf(
      "%d/%d\n",
      mag_filter.totalSamples(),
      mag_filter.windowSize()
    );
  }
  FB->setTextColor(color, BLACK);
  FB->writeString((char*) tmp_str.string());
  tmp_str.clear();
}




int8_t uAppMagnetometer::_rerender_graph() {
  const uint16_t TOP_MARGIN     = 10;
  const uint16_t GRAPH_HEIGHT   = 53;
  //const uint8_t GRAPH_WIDTH    = _cluttered_display() ? (96-(COMPASS_SIZE+ELEMENT_MARGIN)) : 96;
  const uint16_t GRAPH_WIDTH    = sizeof(glbl_graph_data_1) / sizeof(glbl_graph_data_1[0]);
  const uint16_t GRAPH_H_OFFSET = FB->x() - GRAPH_WIDTH;
  Vector3f* mag_vect_ptr = mag_vect.getData();
  FB->setTextColor(WHITE, BLACK);
  FB->setCursor(0, 0);
  StringBuilder tmp_val_str;
  tmp_val_str.concatf("%.4f uT   ", mag_vect_ptr->length());
  FB->writeString(&tmp_val_str);

  const uint32_t  DATA_SIZE = mag_filter.windowSize();
  const uint32_t  LAST_SIDX = mag_filter.lastIndex();
  const Vector3f* F_MEM_PTR = mag_filter.memPtr();

  for (uint32_t i = 0; i < GRAPH_WIDTH; i++) {
    Vector3f* vect = (F_MEM_PTR + (((LAST_SIDX + DATA_SIZE) - (GRAPH_WIDTH - i)) % DATA_SIZE));
    glbl_graph_data_1[i] = vect->x;
    glbl_graph_data_2[i] = vect->y;
    glbl_graph_data_3[i] = vect->z;
  }


  _graph.trace0.data_len     = GRAPH_WIDTH;
  _graph.trace1.data_len     = GRAPH_WIDTH;
  _graph.trace2.data_len     = GRAPH_WIDTH;

  _graph.trace0.offset_x     = 0;
  _graph.trace1.offset_x     = 0;
  _graph.trace2.offset_x     = 0;

  _graph.trace0.show_y_range = _cluttered_display();
  _graph.trace1.show_y_range = _cluttered_display();
  _graph.trace2.show_y_range = _cluttered_display();

  _graph.trace0.show_value   = _render_text_value();
  _graph.trace1.show_value   = _render_text_value();
  _graph.trace2.show_value   = _render_text_value();

  _graph.trace0.dataset      = glbl_graph_data_1;
  _graph.trace1.dataset      = glbl_graph_data_2;
  _graph.trace2.dataset      = glbl_graph_data_3;

  _graph.trace0.enabled      = true;
  _graph.trace1.enabled      = true;
  _graph.trace2.enabled      = true;

  _graph.drawGraph(FB, 0, TOP_MARGIN);
  return 0;
}


/*
* uApp redraw function
*/
void uAppMagnetometer::_redraw_window() {
  const uint8_t TOP_MARGIN     = 10;
  const uint8_t ELEMENT_MARGIN = 1;
  const uint8_t COMPASS_SIZE   = 34;
  const uint8_t GRAPH_HEIGHT   = 53;
  StringBuilder tmp_val_str;
  FB->setTextSize(0);
  UIGfxWrapper gfx(FB);

  switch (_modal_id) {
    case UAPP_SUBMODAL_TRICORDER_MAG_STATS:
      if (compass.dataReady()) {
        char* title_txt = (char*) "No data       ";
        double x_val = 0.0;
        double y_val = 0.0;
        double z_val = 0.0;
        double vect_magnitude = 0.0;
        if (_cluttered_display()) {
          title_txt = (char*) "stdev: ";
          x_val = graph_array_mag_strength_x.stdev();
          y_val = graph_array_mag_strength_y.stdev();
          z_val = graph_array_mag_strength_z.stdev();
        }
        else {
          title_txt = (char*) "RMS:   ";
          x_val = graph_array_mag_strength_x.rms();
          y_val = graph_array_mag_strength_y.rms();
          z_val = graph_array_mag_strength_z.rms();
        }
        vect_magnitude = sqrt((x_val*x_val) + (y_val * y_val) + (z_val * z_val));
        FB->setCursor(0, 0);
        FB->setTextColor(WHITE, BLACK);
        FB->writeString(title_txt);
        FB->setTextColor(GREY, BLACK);
        tmp_val_str.concatf("%.5f", vect_magnitude);
          FB->writeString((char*) tmp_val_str.string());
          tmp_val_str.clear();

        FB->setCursor(0, 14);
        FB->setTextColor(WHITE, BLACK);
        FB->writeString(" X: ");
        tmp_val_str.concatf("%.5f\n", x_val);
          FB->setTextColor(COLOR_X_AXIS, BLACK);
          FB->writeString((char*) tmp_val_str.string());
          tmp_val_str.clear();
        FB->setTextColor(WHITE, BLACK);
        FB->writeString(" Y: ");
        tmp_val_str.concatf("%.5f\n", y_val);
          FB->setTextColor(COLOR_Y_AXIS, BLACK);
          FB->writeString((char*) tmp_val_str.string());
          tmp_val_str.clear();
        FB->setTextColor(WHITE, BLACK);
        FB->writeString(" Z: ");
        tmp_val_str.concatf("%.5f\n", z_val);
          FB->setTextColor(COLOR_Z_AXIS, BLACK);
          FB->writeString((char*) tmp_val_str.string());
          tmp_val_str.clear();
        FB->setTextColor(WHITE, BLACK);
        FB->writeString("Temp: ");
        tmp_val_str.concatf("%.2f\n", mag_adc.getTemperature());
          FB->setTextColor(GREY, BLACK);
          FB->writeString((char*) tmp_val_str.string());
          tmp_val_str.clear();
      }
      break;

    case UAPP_SUBMODAL_TRICORDER_COMPASS:
      if (compass.dataReady()) {
        float bearing_north = 0.0;
        float bearing_mag = 0.0;
        compass.getBearing(HeadingType::TRUE_NORTH, &bearing_north);
        compass.getBearing(HeadingType::MAGNETIC_NORTH, &bearing_mag);
        gfx.drawCompass(0, TOP_MARGIN, COMPASS_SIZE, COMPASS_SIZE, false, _render_text_value(), bearing_mag, bearing_north);
        FB->setTextColor(WHITE, BLACK);
        FB->setCursor(36, 14);
        FB->writeString("Mag:");
        FB->setCursor(41, 22);
        tmp_val_str.concatf("%.2f  ", bearing_mag);
          FB->setTextColor(GREEN, BLACK);
          FB->writeString(&tmp_val_str);
          tmp_val_str.clear();
        FB->setTextColor(WHITE, BLACK);
        FB->setCursor(36, 30);
        FB->writeString("Pole:");
        FB->setCursor(41, 38);
        tmp_val_str.concatf("%.2f  ", bearing_north);
          FB->setTextColor(GREEN, BLACK);
          FB->writeString(&tmp_val_str);
          tmp_val_str.clear();
        FB->setTextColor(WHITE, BLACK);
        FB->setCursor(36, 46);
        FB->writeString("Length:");
        FB->setCursor(41, 54);
        tmp_val_str.concatf("%.3f  ", mag_vect.getData()->length());
          FB->setTextColor(GREEN, BLACK);
          FB->writeString(&tmp_val_str);
          tmp_val_str.clear();
        if (_cluttered_display()) {
        }
      }
      break;

    case UAPP_SUBMODAL_TRICORDER_MAG_GRAPH:
      if (mag_filter.windowFull()) {
        _rerender_graph();
      }
      else {
        _render_mag_unready();
      }
      break;

    case UAPP_SUBMODAL_TRICORDER_MAG_CONF:
      if (true) {
        FB->setCursor(0, 0);
        FB->setTextColor(WHITE, BLACK);
        FB->writeString("Mag settings: ");

        FB->setCursor(0, 14);
        FB->setTextColor(WHITE, BLACK);
        FB->writeString("Bandwidth:  ");
        FB->setTextColor(CYAN, BLACK);
        FB->writeString((DRV425Bandwidth::BW0 == magneto.bandwidth()) ? "BW_0" : "BW_1");

        static const uint32_t OSR_RATIO_VALUES[] = {
          32,      64,   128,   256,   512,  1024,  2048,  4096,
          8192, 16384, 20480, 24576, 40960, 49152, 81920, 98304
        };
        FB->setTextColor(WHITE, BLACK);
        FB->writeString("\nOvrsampl: ");
        tmp_val_str.concatf("%u\n", OSR_RATIO_VALUES[(uint8_t) mag_adc.getOversamplingRatio()]);
        FB->setTextColor(CYAN, BLACK);
        FB->writeString((char*) tmp_val_str.string());
        tmp_val_str.clear();

        FB->setTextColor(WHITE, BLACK);
        FB->writeString("F depth:  ");
        tmp_val_str.concatf("%u\n", mag_filter.windowSize());
        FB->setTextColor(CYAN, BLACK);
        FB->writeString((char*) tmp_val_str.string());
        tmp_val_str.clear();
      }
      break;

    default:
      redraw_app_window();
      _render_mag_unready();
      break;
  }
}
