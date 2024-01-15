#include "Motherflux0r.h"
#include <inttypes.h>
#include <stdint.h>
#include <math.h>


static uint32_t off_time_vib      = 0;      // millis() when vibrator should be disabled.
static uint32_t off_time_led_r    = 0;      // millis() when LED_R should be disabled.
static uint32_t off_time_led_g    = 0;      // millis() when LED_G should be disabled.
static uint32_t off_time_led_b    = 0;      // millis() when LED_B should be disabled.


/*******************************************************************************
* The program has a set of configurations that it defines and loads at runtime.
* This defines everything required to handle that conf fluidly and safely.
*******************************************************************************/
const EnumDef<CalConfKey> CAL_CONF_KEY_LIST[] = {
  { CalConfKey::CAL_MAG_HARD_IRON,          "mag_hard_iron",          0, (uint8_t) TCode::VECT_3_FLOAT   },
  { CalConfKey::CAL_MAG_SOFT_IRON,          "mag_soft_iron",          0, (uint8_t) TCode::VECT_3_FLOAT   },
  { CalConfKey::CAL_MAG_ORIENTATION,        "mag_orientation",        0, (uint8_t) TCode::VECT_3_FLOAT   },
  { CalConfKey::CAL_IMU_ORIENTATION,        "imu_orientation",        0, (uint8_t) TCode::VECT_3_FLOAT   },
  { CalConfKey::CAL_THERMAL_THRESHOLD_LOW,  "thermal_threshold_low",  0, (uint8_t) TCode::INT8           },
  { CalConfKey::CAL_THERMAL_THRESHOLD_HIGH, "thermal_threshold_high", 0, (uint8_t) TCode::INT8           },
  { CalConfKey::CAL_BATTERY_THRESHOLD_LOW,  "battery_threshold_low",  0, (uint8_t) TCode::FLOAT          },
  { CalConfKey::CAL_BATTERY_THRESHOLD_HIGH, "battery_threshold_high", 0, (uint8_t) TCode::FLOAT          },
  { CalConfKey::CAL_DATE,                   "date",                   0, (uint8_t) TCode::UINT64         },
  { CalConfKey::INVALID, "INVALID", (ENUM_FLAG_MASK_INVALID_CATCHALL), (uint8_t) TCode::NONE}
};

const EnumDef<UsrConfKey> USR_CONF_KEY_LIST[] = {
  { UsrConfKey::USR_UNIT_LOCKED,          "unit_locked",           0, (uint8_t) TCode::BOOLEAN  },
  { UsrConfKey::USR_VIB_LEVEL,            "vib_level",             0, (uint8_t) TCode::UINT8    },
  { UsrConfKey::USR_TIMEOUT_IDLE,         "timeout_idle",          0, (uint8_t) TCode::UINT32   },
  { UsrConfKey::USR_TIMEOUT_SHUTDOWN,     "timeout_shutdown",      0, (uint8_t) TCode::UINT32   },
  { UsrConfKey::USR_BLUETOOTH_ENABLED,    "bluetooth_enabled",     0, (uint8_t) TCode::INT8     },
  { UsrConfKey::USR_UNITS,                "units",                 0, (uint8_t) TCode::UINT8    },
  { UsrConfKey::USR_VOLUME,               "volume",                0, (uint8_t) TCode::UINT8    },
  { UsrConfKey::USR_DISP_BRIGHTNESS,      "disp_brightness",       0, (uint8_t) TCode::UINT8    },
  { UsrConfKey::USR_LONGPRESS_THRESHOLD,  "longpress_threshold",   0, (uint8_t) TCode::UINT32   },
  { UsrConfKey::USR_APP_POLLING_PERIOD,   "app_polling_period",    0, (uint8_t) TCode::UINT32   },
  { UsrConfKey::USR_TYPEMATIC_PERIOD,     "typematic_period",      0, (uint8_t) TCode::UINT32   },
  { UsrConfKey::USR_SERIAL_BAUD_RATE,     "serial_baud_rate",      0, (uint8_t) TCode::UINT32   },
  { UsrConfKey::USR_MAG_OVERSAMPLE,       "mag_oversample",        0, (uint8_t) TCode::UINT16   },
  { UsrConfKey::USR_MAG_FILTER_DEPTH,     "mag_filter_depth",      0, (uint8_t) TCode::UINT16   },
  { UsrConfKey::USR_MAG_BANDWIDTH,        "mag_bandwidth",         0, (uint8_t) TCode::UINT32   },
  { UsrConfKey::INVALID, "INVALID", (ENUM_FLAG_MASK_INVALID_CATCHALL), (uint8_t) TCode::NONE}
};

const EnumDefList<CalConfKey> CAL_CONF_LIST(
  CAL_CONF_KEY_LIST, (sizeof(CAL_CONF_KEY_LIST) / sizeof(CAL_CONF_KEY_LIST[0])),
  "CalConfKey"
);
const EnumDefList<UsrConfKey> USR_CONF_LIST(
  USR_CONF_KEY_LIST, (sizeof(USR_CONF_KEY_LIST) / sizeof(USR_CONF_KEY_LIST[0])),
  "UsrConfKey"  // Doesn't _need_ to be the enum name...
);

ConfRecordValidation<CalConfKey> cal_conf(0, &CAL_CONF_LIST);
ConfRecordValidation<UsrConfKey> usr_conf(0, &USR_CONF_LIST);



/*******************************************************************************
* LED and vibrator control
* Only have enable functions since disable is done by timer in the main loop.
*******************************************************************************/
void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity = 3500) {
  uint32_t* millis_ptr = nullptr;
  switch (idx) {
    case LED_R_PIN:
      analogWrite(LED_R_PIN, intensity);
      millis_ptr = &off_time_led_r;
      break;
    case LED_G_PIN:
      analogWrite(LED_G_PIN, intensity);
      millis_ptr = &off_time_led_g;
      break;
    case LED_B_PIN:
      analogWrite(LED_B_PIN, intensity);
      millis_ptr = &off_time_led_b;
      break;
    default:
      return;
  }
  *millis_ptr = millis() + duration;
}


void vibrateOn(uint32_t duration, uint16_t intensity = 4095) {
  analogWrite(VIBRATOR_PIN, intensity);
  off_time_vib = millis() + duration;
}


void timeoutCheckVibLED() {
  uint32_t millis_now = millis();
  if (millis_now >= off_time_led_r) {   pinMode(LED_R_PIN, GPIOMode::INPUT);     }
  if (millis_now >= off_time_led_g) {   pinMode(LED_G_PIN, GPIOMode::INPUT);     }
  if (millis_now >= off_time_led_b) {   pinMode(LED_B_PIN, GPIOMode::INPUT);     }
  if (millis_now >= off_time_vib) {     pinMode(VIBRATOR_PIN, GPIOMode::INPUT);  }
}



/*******************************************************************************
* Enum support functions
*******************************************************************************/

const char* const getSensorIDString(SensorID e) {
  switch (e) {
    case SensorID::BARO:          return "BARO";
    case SensorID::MAGNETOMETER:  return "MAGNETOMETER";
    case SensorID::IMU:           return "IMU";
    case SensorID::LIGHT:         return "LIGHT";
    case SensorID::MIC:           return "MIC";
    case SensorID::UV:            return "UV";
    case SensorID::GPS:           return "GPS";
    case SensorID::THERMOPILE:    return "THERMOPILE";
    //case SensorID::PSU_TEMP:      return "PSU_TEMP";
    case SensorID::TOF:           return "ToF";
    case SensorID::BATT_VOLTAGE:  return "BATT_VOLTAGE";
    case SensorID::LUX:           return "LUX";
  }
  return "UNKNOWN";
}


void listAllSensors(StringBuilder* output) {
  for (uint8_t i = 0; i < 11; i++) {
    output->concatf("%2u: %s\n", i, getSensorIDString((SensorID) i));
  }
}



/*
* Taken from:
* https://forum.pjrc.com/threads/32677-Is-there-a-logarithmic-function-for-FFT-bin-selection-for-any-given-of-bands
*/
float FindE(int bands, int bins) {
  float increment=0.1, eTest, n;
  int b, count, d;

  for (eTest = 1; eTest < bins; eTest += increment) {     // Find E through brute force calculations
    count = 0;
    for (b = 0; b < bands; b++) {                         // Calculate full log values
      n = pow(eTest, b);
      d = int(n + 0.5);
      count += d;
    }
    if (count > bins) {     // We calculated over our last bin
      eTest -= increment;   // Revert back to previous calculation increment
      increment /= 10.0;    // Get a finer detailed calculation & increment a decimal point lower
    }
    else
      if (count == bins)    // We found the correct E
        return eTest;       // Return calculated E
    if (increment < 0.0000001)        // Ran out of calculations. Return previous E. Last bin will be lower than (bins-1)
      return (eTest - increment);
  }
  return 0;                 // Return error 0
}

/*
* Taken from:
* https://forum.pjrc.com/threads/32677-Is-there-a-logarithmic-function-for-FFT-bin-selection-for-any-given-of-bands
*/
void printFFTBins(StringBuilder* output) {
  float e, n;
  int b, bands, bins, count=0, d;

  bands = 96;                             // Frequency bands; (Adjust to desired value)
  bins = 256;                             // FFT bins; (Adjust to desired value)

  e = FindE(bands, bins);                 // Find calculated E value
  if (e) {                                // If a value was returned continue
    output->concatf("E = %4.4f\n", e);    // Print calculated E value
    for (b = 0; b < bands; b++) {         // Test and print the bins from the calculated E
      n = pow(e, b);
      d = int(n + 0.5);

      output->concatf("%4d ", count);     // Print low bin
      count += d - 1;
      output->concatf("%4d\n", count);    // Print high bin
      ++count;
    }
  }
  else
    output->concatf("Error\n\n");         // Error, something happened
}


/*******************************************************************************
* Display helper routines
*******************************************************************************/
/*
* Render a basic icon to the display.
*/
void render_button_icon(uint8_t sym, int x, int y, uint16_t color) {
  const uint8_t ICON_SIZE = 7;
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  switch (sym) {
    case BUTTON_UP:
      x0 = x + (ICON_SIZE >> 1);
      y0 = y;
      x1 = x;
      y1 = y + ICON_SIZE;
      x2 = x + ICON_SIZE;
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case BUTTON_DOWN:
      x0 = x;
      y0 = y;
      x1 = x + ICON_SIZE;
      y1 = y;
      x2 = x + (ICON_SIZE >> 1);
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case BUTTON_LEFT:
      x0 = x + ICON_SIZE;
      y0 = y;
      x1 = x;
      y1 = y + (ICON_SIZE >> 1);
      x2 = x + ICON_SIZE;
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case BUTTON_RIGHT:
      x0 = x;
      y0 = y;
      x1 = x + ICON_SIZE;
      y1 = y + (ICON_SIZE >> 1);
      x2 = x;
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
		case ICON_CANCEL:
		case ICON_ACCEPT:
		case ICON_THERMO:
		case ICON_IMU:
		case ICON_GPS:
		case ICON_LIGHT:
		case ICON_UVI:
		case ICON_SOUND:
		case ICON_RH:
		case ICON_MIC:
		case ICON_MAGNET:
		case ICON_BATTERY:
      {
        uint16_t* iptr = bitmapPointer(sym);
        const uint16_t EXTENT_X = *iptr++;
        const uint16_t EXTENT_Y = *iptr++;
        //display.setAddrWindow(x, y, EXTENT_X, EXTENT_Y);
        for (uint8_t h = 0; h < EXTENT_Y; h++) {
          for (uint8_t w = 0; w < EXTENT_X; w++) {
            display.setPixel(x+w, y+h, *iptr++);
          }
        }
        //display.endWrite();
      }
      break;
  }
}



void draw_graph_obj(Image* FB,
  PixUInt x, PixUInt y, PixUInt w, PixUInt h, uint32_t color,
  bool opt1, bool opt2, bool opt3,
  SensorFilter<float>* filter
)
{
  ImageGraph<float> graph(w, h);
  graph.fg_color = 0xFFFFFFFF;

  const uint32_t  DATA_SIZE = filter->windowSize();
  const uint32_t  LAST_SIDX = filter->lastIndex();
  const uint32_t  DATA_IDX  = (1 + LAST_SIDX + strict_abs_delta(DATA_SIZE, (uint32_t) w)) % DATA_SIZE;
  const float*    F_MEM_PTR = filter->memPtr();
  float tmp_data[DATA_SIZE];
  for (uint32_t i = 0; i < DATA_SIZE; i++) {
    tmp_data[i] = *(F_MEM_PTR + ((i + LAST_SIDX) % DATA_SIZE));
  }

  graph.trace0.color        = color;
  graph.trace0.dataset      = tmp_data;
  graph.trace0.data_len     = DATA_SIZE;
  graph.trace0.enabled      = true;
  graph.trace0.autoscale_x  = false;
  graph.trace0.autoscale_y  = true;
  graph.trace0.show_x_range = false;
  graph.trace0.show_y_range = opt1;
  graph.trace0.show_value   = opt2;
  graph.trace0.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  graph.trace0.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.
  graph.trace0.offset_x     = DATA_IDX;

  graph.drawGraph(FB, x, y);
}


void draw_graph_obj(Image* FB,
  PixUInt x, PixUInt y, PixUInt w, PixUInt h, uint32_t color1, uint32_t color2, uint32_t color3,
  bool opt1, bool opt2, bool opt3,
  SensorFilter<float>* filter1, SensorFilter<float>* filter2, SensorFilter<float>* filter3
)
{
  ImageGraph<float> graph(w, h);
  graph.fg_color = 0xFFFFFFFF;

  const uint32_t  DATA_SIZE_1 = filter1->windowSize();
  const uint32_t  LAST_SIDX_1 = filter1->lastIndex();
  const uint32_t  DATA_IDX_1  = (1 + LAST_SIDX_1 + strict_abs_delta(DATA_SIZE_1, (uint32_t) w)) % DATA_SIZE_1;
  const float*    F_MEM_PTR_1 = filter1->memPtr();
  float tmp_data_1[DATA_SIZE_1];
  for (uint32_t i = 0; i < DATA_SIZE_1; i++) {
    tmp_data_1[i] = *(F_MEM_PTR_1 + ((i + LAST_SIDX_1) % DATA_SIZE_1));
  }

  const uint32_t  DATA_SIZE_2 = filter2->windowSize();
  const uint32_t  LAST_SIDX_2 = filter2->lastIndex();
  const uint32_t  DATA_IDX_2  = (1 + LAST_SIDX_2 + strict_abs_delta(DATA_SIZE_2, (uint32_t) w)) % DATA_SIZE_2;
  const float*    F_MEM_PTR_2 = filter2->memPtr();
  float tmp_data_2[DATA_SIZE_2];
  for (uint32_t i = 0; i < DATA_SIZE_2; i++) {
    tmp_data_2[i] = *(F_MEM_PTR_2 + ((i + LAST_SIDX_2) % DATA_SIZE_2));
  }

  const uint32_t  DATA_SIZE_3 = filter3->windowSize();
  const uint32_t  LAST_SIDX_3 = filter3->lastIndex();
  const uint32_t  DATA_IDX_3  = (1 + LAST_SIDX_3 + strict_abs_delta(DATA_SIZE_3, (uint32_t) w)) % DATA_SIZE_3;
  const float*    F_MEM_PTR_3 = filter3->memPtr();
  float tmp_data_3[DATA_SIZE_3];
  for (uint32_t i = 0; i < DATA_SIZE_3; i++) {
    tmp_data_3[i] = *(F_MEM_PTR_3 + ((i + LAST_SIDX_3) % DATA_SIZE_3));
  }

  graph.trace0.color        = color1;
  graph.trace0.dataset      = tmp_data_1;
  graph.trace0.data_len     = DATA_SIZE_1;
  graph.trace0.enabled      = true;
  graph.trace0.autoscale_x  = false;
  graph.trace0.autoscale_y  = true;
  graph.trace0.show_x_range = false;
  graph.trace0.show_y_range = opt1;
  graph.trace0.show_value   = opt2;
  graph.trace0.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  graph.trace0.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.
  graph.trace0.offset_x     = DATA_IDX_1;

  graph.trace1.color        = color2;
  graph.trace1.dataset      = tmp_data_2;
  graph.trace1.data_len     = DATA_SIZE_2;
  graph.trace1.enabled      = true;
  graph.trace1.autoscale_x  = false;
  graph.trace1.autoscale_y  = true;
  graph.trace1.show_x_range = false;
  graph.trace1.show_y_range = opt1;
  graph.trace1.show_value   = opt2;
  graph.trace1.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  graph.trace1.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.
  graph.trace1.offset_x     = DATA_IDX_2;

  graph.trace2.color        = color3;
  graph.trace2.dataset      = tmp_data_3;
  graph.trace2.data_len     = DATA_SIZE_3;
  graph.trace2.enabled      = true;
  graph.trace2.autoscale_x  = false;
  graph.trace2.autoscale_y  = true;
  graph.trace2.show_x_range = false;
  graph.trace2.show_y_range = opt1;
  graph.trace2.show_value   = opt2;
  graph.trace2.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  graph.trace2.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.
  graph.trace2.offset_x     = DATA_IDX_3;

  graph.drawGraph(FB, x, y);
}


const DRV425Config DRV425_CONFIG;

/*******************************************************************************
* Checklist for moving out of the boot-up phase of the runtime.
*******************************************************************************/
const StepSequenceList CHECKLIST_BOOT[] = {
  { .FLAG         = CHKLST_BOOT_BUS_SPI0,
    .LABEL        = "INIT_SPI0",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_BUS_I2C0,
    .LABEL        = "INIT_I2C0",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return (i2c0.busOnline() ? 1:0);  }
  },
  { .FLAG         = CHKLST_BOOT_BUS_I2C1,
    .LABEL        = "INIT_I2C1",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return (i2c1.busOnline() ? 1:0);  }
  },
  { .FLAG         = CHKLST_BOOT_TOUCH,
    .LABEL        = "INIT_TOUCH",
    .DEP_MASK     = (CHKLST_BOOT_BUS_I2C0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DISPLAY,
    .LABEL        = "INIT_DISPLAY",
    .DEP_MASK     = (CHKLST_BOOT_BUS_SPI0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },

  { .FLAG         = CHKLST_BOOT_MAG_GPIO,
    .LABEL        = "INIT_MAG_GPIO",
    .DEP_MASK     = (CHKLST_BOOT_BUS_I2C1),
    .DISPATCH_FXN = []() { return (0 == sx1503.init()   ? 1 : 0);  },
    .POLL_FXN     = []() { return (sx1503.initialized() ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_BOOT_MAG_ADC,
    .LABEL        = "MAG_ADC",
    .DEP_MASK     = (CHKLST_BOOT_MAG_GPIO),
    .DISPATCH_FXN = []() {
      if (!magneto.power()) return 0;
      return (0 == mag_adc.init() ? 1 : 0);
    },
    .POLL_FXN     = []() { return (mag_adc.adcFound()  ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_BOOT_AUDIO_STACK,
    .LABEL        = "AUDIO_STACK",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() {
      sineL.amplitude(1.0);
      sineL.frequency(440);
      sineL.phase(0);
      sineR.amplitude(0.8);
      sineR.frequency(660);
      sineR.phase(0);
      mixerFFT.gain(0, mix_queueL_to_fft);
      mixerFFT.gain(1, mix_queueR_to_fft);
      mixerFFT.gain(2, mix_noise_to_fft);
      mixerFFT.gain(3, 0.0);
      mixerL.gain(0, mix_queue_to_line);
      mixerL.gain(1, mix_noise_to_line);
      mixerL.gain(2, mix_synth_to_line);
      mixerL.gain(3, 0.0);
      mixerR.gain(0, mix_queue_to_line);
      mixerR.gain(1, mix_noise_to_line);
      mixerR.gain(2, mix_synth_to_line);
      mixerR.gain(3, 0.0);
      pinkNoise.amplitude(volume_pink_noise);
      ampL.gain(volume_left_output);
      ampR.gain(volume_right_output);
      return 1;
    },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_INIT_BARO,
    .LABEL        = "INIT_BARO",
    .DEP_MASK     = (CHKLST_BOOT_BUS_I2C1),
    .DISPATCH_FXN = []() { return (0 == baro.init()   ? 1 : 0);  },
    .POLL_FXN     = []() { return (baro.initialized() ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_BOOT_INIT_UV,
    .LABEL        = "INIT_UV",
    .DEP_MASK     = (CHKLST_BOOT_BUS_I2C1),
    .DISPATCH_FXN = []() { return (0 == uv.init()   ? 1 : 0);  },
    .POLL_FXN     = []() {
      if (uv.initialized()) {
        return ((VEML6075Err::SUCCESS == uv.setIntegrationTime(VEML6075IntTime::IT_100MS)) ? 1 : -1);
      }
      return 0;
    }
  },
  { .FLAG         = CHKLST_BOOT_INIT_GRIDEYE,
    .LABEL        = "INIT_GRIDEYE",
    .DEP_MASK     = (CHKLST_BOOT_BUS_I2C1),
    .DISPATCH_FXN = []() { return (0 == grideye.init()   ? 1 : 0);  },
    .POLL_FXN     = []() { return (grideye.initialized() ? 1 : 0);  }
  },
};

AsyncSequencer checklist_boot(CHECKLIST_BOOT, (sizeof(CHECKLIST_BOOT) / sizeof(CHECKLIST_BOOT[0])));


/*******************************************************************************
* Checklist for turning things on and off.
*******************************************************************************/
const StepSequenceList CHECKLIST_CYCLIC[] = {
};

AsyncSequencer checklist_cyclic(CHECKLIST_CYCLIC, (sizeof(CHECKLIST_CYCLIC) / sizeof(CHECKLIST_CYCLIC[0])));
