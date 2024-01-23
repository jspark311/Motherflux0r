#include <inttypes.h>
#include <stdint.h>
#include <math.h>

#include "Motherflux0r.h"
#include "uApp.h"

/* Console handler prototypes */
int callback_help(StringBuilder*, StringBuilder*);
int callback_console_tools(StringBuilder*, StringBuilder*);
int callback_touch_tools(StringBuilder*, StringBuilder*);
int callback_link_tools(StringBuilder*, StringBuilder*);
int callback_display_test(StringBuilder*, StringBuilder*);
int callback_spi_debug(StringBuilder*, StringBuilder*);
int callback_i2c_tools(StringBuilder*, StringBuilder*);
int callback_logger_tools(StringBuilder*, StringBuilder*);
int callback_led_test(StringBuilder*, StringBuilder*);
int callback_vibrator_test(StringBuilder*, StringBuilder*);
int callback_aout_mix(StringBuilder*, StringBuilder*);
int callback_fft_mix(StringBuilder*, StringBuilder*);
int callback_synth_set(StringBuilder*, StringBuilder*);
int callback_sensor_tools(StringBuilder*, StringBuilder*);
int callback_magnetometer_fxns(StringBuilder*, StringBuilder*);
int callback_sensor_filter_info(StringBuilder*, StringBuilder*);
int callback_meta_filter_info(StringBuilder*, StringBuilder*);
int callback_sensor_filter_set_strat(StringBuilder*, StringBuilder*);
int callback_meta_filter_set_strat(StringBuilder*, StringBuilder*);
int callback_active_app(StringBuilder*, StringBuilder*);
int callback_print_app_profiler(StringBuilder*, StringBuilder*);
int callback_audio_volume(StringBuilder*, StringBuilder*);
int callback_conf_tools(StringBuilder*, StringBuilder*);
int callback_pmu_tools(StringBuilder*, StringBuilder*);
int callback_checklist_dump(StringBuilder*, StringBuilder*);

void cb_button(int button, bool pressed);
void cb_slider(int slider, int value);
void cb_longpress(int button, uint32_t duration);

void callback_adc_value(uint8_t chan, double voltage);
int8_t battery_state_callback(ChargeState);
int8_t location_callback(LocationFrame*);

extern UARTOpts comm_unit_uart_opts;
extern UARTOpts gps_uart_opts;
extern UARTOpts usb_comm_opts;

extern uint32_t boot_time;     // millis() at boot.
extern uint32_t config_time;   // millis() at end of setup().

extern uAppBoot app_boot;
extern C3PScheduledLambda schedule_ui;

static uint32_t off_time_vib      = 0;      // millis() when vibrator should be disabled.
static uint32_t off_time_led_r    = 0;      // millis() when LED_R should be disabled.
static uint32_t off_time_led_g    = 0;      // millis() when LED_G should be disabled.
static uint32_t off_time_led_b    = 0;      // millis() when LED_B should be disabled.

const char* const CONSOLE_PROMPT_STR = "Motherflux0r # ";

float glbl_graph_data_1[96];
float glbl_graph_data_2[96];
float glbl_graph_data_3[96];


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
void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity) {
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


void vibrateOn(uint32_t duration, uint16_t intensity) {
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

  const uint32_t  DATA_SIZE_2 = filter2->windowSize();
  const uint32_t  LAST_SIDX_2 = filter2->lastIndex();
  const uint32_t  DATA_IDX_2  = (1 + LAST_SIDX_2 + strict_abs_delta(DATA_SIZE_2, (uint32_t) w)) % DATA_SIZE_2;
  const float*    F_MEM_PTR_2 = filter2->memPtr();

  const uint32_t  DATA_SIZE_3 = filter3->windowSize();
  const uint32_t  LAST_SIDX_3 = filter3->lastIndex();
  const uint32_t  DATA_IDX_3  = (1 + LAST_SIDX_3 + strict_abs_delta(DATA_SIZE_3, (uint32_t) w)) % DATA_SIZE_3;
  const float*    F_MEM_PTR_3 = filter3->memPtr();
  for (uint32_t i = 0; i < sizeof(glbl_graph_data_1); i++) {
    glbl_graph_data_1[i] = *(F_MEM_PTR_1 + ((i + LAST_SIDX_1) % DATA_SIZE_1));
    glbl_graph_data_2[i] = *(F_MEM_PTR_2 + ((i + LAST_SIDX_2) % DATA_SIZE_2));
    glbl_graph_data_3[i] = *(F_MEM_PTR_3 + ((i + LAST_SIDX_3) % DATA_SIZE_3));
  }

  graph.trace0.color        = color1;
  graph.trace0.dataset      = glbl_graph_data_1;
  graph.trace0.data_len     = w;
  graph.trace0.enabled      = true;
  graph.trace0.autoscale_x  = false;
  graph.trace0.autoscale_y  = true;
  graph.trace0.show_x_range = false;
  graph.trace0.show_y_range = opt1;
  graph.trace0.show_value   = opt2;
  graph.trace0.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  graph.trace0.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.
  graph.trace0.offset_x     = 0;

  graph.trace1.color        = color2;
  graph.trace1.dataset      = glbl_graph_data_2;
  graph.trace1.data_len     = w;
  graph.trace1.enabled      = true;
  graph.trace1.autoscale_x  = false;
  graph.trace1.autoscale_y  = true;
  graph.trace1.show_x_range = false;
  graph.trace1.show_y_range = opt1;
  graph.trace1.show_value   = opt2;
  graph.trace1.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  graph.trace1.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.
  graph.trace1.offset_x     = 0;

  graph.trace2.color        = color3;
  graph.trace2.dataset      = glbl_graph_data_3;
  graph.trace2.data_len     = w;
  graph.trace2.enabled      = true;
  graph.trace2.autoscale_x  = false;
  graph.trace2.autoscale_y  = true;
  graph.trace2.show_x_range = false;
  graph.trace2.show_y_range = opt1;
  graph.trace2.show_value   = opt2;
  graph.trace2.grid_lock_x  = false;   // Default is to allow the grid to scroll with the starting offset.
  graph.trace2.grid_lock_y  = false;   // Default is to allow the grid to scroll with any range shift.
  graph.trace2.offset_x     = 0;

  graph.drawGraph(FB, x, y);
}

uint32_t touch_timeout  = 0;  // TODO: Rework...

/*******************************************************************************
* Checklist for boot and shutdown. These checks happen once on on every runtime.
*******************************************************************************/
const StepSequenceList CHECKLIST_BOOT[] = {
  /* Checklist for booting up *************************************************/
  /* Audio stack, which takes lots of memory and set behind the scenes. */
  { .FLAG         = CHKLST_BOOT_AUDIO_STACK,
    .LABEL        = "Start Audio",
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

  /* Large memory allocations */
  { .FLAG         = CHKLST_BOOT_INIT_BIG_MEM,
    .LABEL        = "BigMem",
    .DEP_MASK     = (CHKLST_BOOT_AUDIO_STACK),
    .DISPATCH_FXN = []() {
      int8_t ret = 1;
      const char* const ALLOC_FAIL_STR = "Failed to allocate memory for";
      // Do not re-attempt this step unless checks are made harder. BigMem
      //   either fails, or not. It could be made non-blocking, but we want
      //   the ability to gracefully retain a console (and other features),
      //   even in the event that memory was somewhat over-budgeted.
      if (0 != init_sensor_memory()) {  // Allocate memory for the filters.
        c3p_log(LOG_LEV_DEBUG, "BOOT_CHKLST", "%s sensors.\n", ALLOC_FAIL_STR);
        ret = -1;
      }
      if (0 != graph_array_cpu_time.init()) {
        c3p_log(LOG_LEV_DEBUG, "BOOT_CHKLST", "%s CPU profiler.\n", ALLOC_FAIL_STR);
        ret = -1;
      }
      if (0 != graph_array_frame_rate.init()) {
        c3p_log(LOG_LEV_DEBUG, "BOOT_CHKLST", "%s UI profiler.\n", ALLOC_FAIL_STR);
        ret = -1;
      }
      if (0 != mag_filter.init()) {
        c3p_log(LOG_LEV_DEBUG, "BOOT_CHKLST", "%s mag_filter.\n", ALLOC_FAIL_STR);
        ret = -1;
      }
      return ret;
    },
    .POLL_FXN     = []() { return 1;  }
  },

  /* BusQueues and GPIO are fairly important. These also run early. */
  { .FLAG         = CHKLST_BOOT_INIT_GPIO,
    .LABEL        = "GPIO init",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() {
      pinMode(IMU_IRQ_PIN,    GPIOMode::INPUT_PULLUP);
      pinMode(DRV425_CS_PIN,  GPIOMode::INPUT); // Wrong
      pinMode(ANA_LIGHT_PIN,  GPIOMode::INPUT);
      pinMode(TOF_IRQ_PIN,    GPIOMode::INPUT);
      pinMode(LED_R_PIN,      GPIOMode::INPUT);
      pinMode(LED_G_PIN,      GPIOMode::INPUT);
      pinMode(LED_B_PIN,      GPIOMode::INPUT);
      pinMode(RADIO_ENABLE_PIN, GPIOMode::OUTPUT);
      setPin(RADIO_ENABLE_PIN, true);
      return 1;
    },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_INIT_SPI0,
    .LABEL        = "Start spi1",
    .DEP_MASK     = (CHKLST_BOOT_INIT_GPIO),  // Uninitialized CS pins must be seized.
    .DISPATCH_FXN = []() { return (0 == spi0.init()   ? 1 : 0);  },
    .POLL_FXN     = []() {
      int8_t ret = (spi0.busOnline() ? 1:0);
      if (1 == ret) {
        display.setBus(&spi0);
        mag_adc.setAdapter(&spi0);
      }
      return ret;
    }
  },
  { .FLAG         = CHKLST_BOOT_INIT_I2C0,
    .LABEL        = "Start i2c0",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return (0 == i2c0.init()   ? 1 : 0);  },
    .POLL_FXN     = []() {
      int8_t ret = (i2c0.busOnline() ? 1:0);
      if (1 == ret) {
        touch->setAdapter(&i2c0);
      }
      return ret;
    }
  },
  { .FLAG         = CHKLST_BOOT_INIT_I2C1,
    .LABEL        = "Start i2c1",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return (0 == i2c1.init()   ? 1 : 0);  },
    .POLL_FXN     = []() {
      int8_t ret = (i2c1.busOnline() ? 1:0);
      if (1 == ret) {
        sx1503.setAdapter(&i2c1);
        tsl2561.setAdapter(&i2c1);
        uv.setAdapter(&i2c1);
        baro.setAdapter(&i2c1);
        grideye.setAdapter(&i2c1);
        // tof.setAdapter(&i2c1);
      }
      return ret;
    }
  },

  /* Touch board discovery and conf */
  { .FLAG         = CHKLST_BOOT_INIT_TOUCH_FOUND,
    .LABEL        = "Start touch",
    .DEP_MASK     = (CHKLST_BOOT_INIT_I2C0 | CHKLST_BOOT_INIT_GPIO),
    .DISPATCH_FXN = []() {
      if ((0 == touch->reset()) ? 1 : 0) {
        touch_timeout = millis();
        return 1;
      }
      return -1;
    },
    .POLL_FXN     = []() {
      if (millis_since(touch_timeout) > 300) {
        return -1;
      }
      return (touch->devFound() ? 1:0);
    }
  },
  { .FLAG         = CHKLST_BOOT_INIT_TOUCH_READY,
    .LABEL        = "Touch ready",
    .DEP_MASK     = (CHKLST_BOOT_INIT_TOUCH_FOUND),
    .DISPATCH_FXN = []() {
      if (touch->deviceReady()) {
        touch->setLongpress(TOUCH_DWELL_LONG_PRESS, 0);   // Set long-press. No rep.
        touch->setButtonFxn(cb_button);
        touch->setSliderFxn(cb_slider);
        touch->setLongpressFxn(cb_longpress);
        return (0 == touch->setMode(SX8634OpMode::ACTIVE) ? 1:-1);
      }
      return 0;
    },
    .POLL_FXN     = []() {
      if (SX8634OpMode::ACTIVE == touch->operationalMode()) {
        return 1;
      }
      else if (false) {
        // TODO: Timeout observation.
      }
      return 0;
    }
  },

  /* Display and UI */
  { .FLAG         = CHKLST_BOOT_INIT_DISPLAY,
    .LABEL        = "Start display",
    .DEP_MASK     = (CHKLST_BOOT_INIT_SPI0),
    .DISPATCH_FXN = []() { return (0 == display.init()   ? 1 : 0);  },
    .POLL_FXN     = []() { return (display.initialized() ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_BOOT_INIT_UI,
    .LABEL        = "Start UI",
    .DEP_MASK     = (CHKLST_BOOT_INIT_DISPLAY),
    .DISPATCH_FXN = []() {
      int8_t ret = 0;
      if (display.enabled()) {
        schedule_ui.enabled(true);
        ret = 1;
      }
      return ret;
    },
    .POLL_FXN     = []() { return (app_boot.firstFrameWritten() ? 1 : 0);  }
  },

  /* Magnetometer GPIO expander and pipeline */
  { .FLAG         = CHKLST_BOOT_INIT_MAG_GPIO,
    .LABEL        = "Start mag GPIO",
    .DEP_MASK     = (CHKLST_BOOT_INIT_I2C1),
    .DISPATCH_FXN = []() { return (0 == sx1503.init()   ? 1 : 0);  },
    .POLL_FXN     = []() {
      if (sx1503.initialized()) {
        // TODO: This is just to prod the compass into returning a complete
        //   dataset. It's bogus until there is an IMU.
        Vector3f gravity(0.0, 0.0, 1.0);
        Vector3f gravity_err(0.002, 0.002, 0.002);
        compass.pushVector(SpatialSense::ACC, &gravity, &gravity_err);   // Set gravity, initially.
        return 1;
      }
      return 0;
    }
  },

  /* Power controller */
  { .FLAG         = CHKLST_BOOT_INIT_PMU_GUAGE,
    .LABEL        = "Start gas guage",
    .DEP_MASK     = (CHKLST_BOOT_INIT_I2C0 | CHKLST_BOOT_INIT_PMU_CHARGER),
    .DISPATCH_FXN = []() { return 1;  },
    //.POLL_FXN     = []() { return (pmu.ltc294x.initComplete() ? 1 : 0);  }
    .POLL_FXN     = []() {
      pmu.attachCallback(battery_state_callback);
      return 1;
    }
  },
  { .FLAG         = CHKLST_BOOT_INIT_PMU_CHARGER,
    .LABEL        = "Start battery",
    .DEP_MASK     = (CHKLST_BOOT_INIT_I2C0 | CHKLST_BOOT_INIT_GPIO),
    .DISPATCH_FXN = []() { return (0 == pmu.init(&i2c0) ? 1 : 0);  },
    //.POLL_FXN     = []() { return (pmu.bq24155.initComplete() ? 1 : 0);  }
    .POLL_FXN     = []() { return 1;  }
  },

  /* Storage and non-volatile configuration */
  { .FLAG         = CHKLST_BOOT_INIT_STORAGE,
    .LABEL        = "Start storage",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_INIT_CONF_LOAD,
    .LABEL        = "Load config",
    .DEP_MASK     = (CHKLST_BOOT_INIT_STORAGE),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },

  /* UART-based faculties */
  { .FLAG         = CHKLST_BOOT_INIT_GPS,
    .LABEL        = "Start GPS",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return (0 == gps_uart.init(&gps_uart_opts) ? 1 : 0);  },
    .POLL_FXN     = []() {
      if (0 == gps.init()) {
        gps.setCallback(location_callback);
        gps_uart.readCallback(&gps);  // Attach the GPS UART to its parser.
        return 1;
      }
      return -1;
    }
  },
  { .FLAG         = CHKLST_BOOT_INIT_COMMS,
    .LABEL        = "Start comm unit",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return (0 == comm_unit_uart.init(&comm_unit_uart_opts) ? 1 : -1);  },
    .POLL_FXN     = []() { return 1;  }
  },

  /* USB serial and console */
  { .FLAG         = CHKLST_BOOT_INIT_USB,
    .LABEL        = "Start USB",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return (0 == console_uart.init(&usb_comm_opts) ? 1 : -1);  },
    .POLL_FXN     = []() {
      if (Serial) {
        while (Serial.available()) { Serial.read(); } // Drain any leading characters...
        console_uart.readCallback(&console);          // ...attach the UART to console...
        console.setOutputTarget(&console_uart);       // ...and console to UART.
        return 1;
      }
      return ((millis_since(boot_time) > 10000) ? -1 : 0);
    }
  },
  { .FLAG         = CHKLST_BOOT_INIT_CONSOLE,
    .LABEL        = "Start console",
    .DEP_MASK     = (CHKLST_BOOT_INIT_USB),
    .DISPATCH_FXN = []() {
      // Push a boot banner to the UART.
      StringBuilder ptc("Motherflux0r ");
      ptc.concat(TEST_PROG_VERSION);
      ptc.concat("\t Build date " __DATE__ " " __TIME__ "\n");
      console_uart.pushBuffer(&ptc);
      // Stall until the boot banner is written out.
      while (!console_uart.flushed()) {   console_uart.poll();  }
      return 1;
    },
    .POLL_FXN     = []() {
      console.defineCommand("help",        '?',  "Prints help to console.", "[<specific command>]", 0, callback_help);
      console.defineCommand("console",     '\0', "Console conf.", "[echo|prompt|force|rxterm|txterm]", 0, callback_console_tools);
      platform.configureConsole(&console);
      console.defineCommand("touch",       '\0', "SX8634 tools", "", 0, callback_touch_tools);
      console.defineCommand("link",        'l',  "Linked device tools.", "", 0, callback_link_tools);
      console.defineCommand("disp",        'd',  "Display test", "", 1, callback_display_test);
      console.defineCommand("spi",         '\0', "SPI debug.", "", 1, callback_spi_debug);
      #if defined(CONFIG_C3P_I2CADAPTER_ENABLE_CONSOLE)
      console.defineCommand("i2c",         '\0', "I2C tools", "Usage: i2c <bus> <action> [addr]", 1, callback_i2c_tools);
      #endif
      console.defineCommand("log",         '\0', "Logger tools", "", 0, callback_logger_tools);
      console.defineCommand("led",         '\0', "LED Test", "", 1, callback_led_test);
      console.defineCommand("vib",         'v',  "Vibrator test", "", 0, callback_vibrator_test);
      console.defineCommand("aout",        '\0', "Mix volumes for the headphones.", "", 4, callback_aout_mix);
      console.defineCommand("fft",         '\0', "Mix volumes for the FFT.", "", 4, callback_fft_mix);
      console.defineCommand("synth",       '\0', "Synth parameters.", "", 2, callback_synth_set);
      console.defineCommand("sensor",      's',  "Sensor tools", "", 0, callback_sensor_tools);
      console.defineCommand("mag",         'M',  "Magnetometer tools", "[info|gpio|adc]", 0, callback_magnetometer_fxns);
      console.defineCommand("sfi",         '\0', "Sensor filter info.", "", 0, callback_sensor_filter_info);
      console.defineCommand("mfi",         '\0', "Meta filter info.", "", 1, callback_meta_filter_info);
      console.defineCommand("sfs",         '\0', "Sensor filter strategy set.", "", 2, callback_sensor_filter_set_strat);
      console.defineCommand("mfs",         '\0', "Meta filter strategy set.", "", 2, callback_meta_filter_set_strat);
      console.defineCommand("app",         'a',  "Select active application.", "", 0, callback_active_app);
      console.defineCommand("prof",        'P',  "Dump application profiler.", "<app | sch>", 1, callback_print_app_profiler);
      console.defineCommand("vol",         '\0', "Audio volume.", "", 0, callback_audio_volume);
      console.defineCommand("conf",        'c',  "Dump/set conf key.", "[usr|cal|pack] [conf_key] [value]", 1, callback_conf_tools);
      console.defineCommand("pmu",         'p',  "PMU tools", "[info|punch|charging|aux|reset|init|refresh|verbosity]", 1, callback_pmu_tools);
      console.defineCommand("hwstate",     '\0', "Hardware state checklists.", "", 0, callback_checklist_dump);
      console.setTXTerminator(LineTerm::CRLF);
      console.setRXTerminator(LineTerm::CR);
      console.emitPrompt(true);
      console.localEcho(true);
      console.printHelpOnFail(true);
      console.setPromptString(CONSOLE_PROMPT_STR);
      console.init();
      return 1;
    }
  },


  /* Checklist for shutting down **********************************************/
  { .FLAG         = CHKLST_BOOT_DEINIT_GPIO,
    .LABEL        = "MCU GPIO safety",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_SPI0,
    .LABEL        = "Stop spi0",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_I2C0,
    .LABEL        = "Stop i2c0",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_I2C1,
    .LABEL        = "Stop i2c1",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_TOUCH,
    .LABEL        = "Stop touch",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_DISPLAY,
    .LABEL        = "Stop display",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_MAG_GPIO,
    .LABEL        = "Stop mag GPIO",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_UI,
    .LABEL        = "Stop UI",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_PMU_SAFE,
    .LABEL        = "PMU safety",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_HANGUP,
    .LABEL        = "Comms hangup",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_UARTS,
    .LABEL        = "UARTs flushed",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_BOOT_DEINIT_CONF_SAVE,
    .LABEL        = "Conf save",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
};

AsyncSequencer checklist_boot(CHECKLIST_BOOT, (sizeof(CHECKLIST_BOOT) / sizeof(CHECKLIST_BOOT[0])));



/*******************************************************************************
* Checklist for turning things on and off.
*******************************************************************************/
const StepSequenceList CHECKLIST_CYCLIC[] = {
  /* Barometer */
  { .FLAG         = CHKLST_CYC_BARO_INIT,
    .LABEL        = "Baro INIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return (0 == baro.init()   ? 1 : 0);  },
    .POLL_FXN     = []() { return (baro.initialized() ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_CYC_BARO_CONF,
    .LABEL        = "Baro CONF",
    .DEP_MASK     = (CHKLST_CYC_BARO_INIT),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_CYC_BARO_DEINIT,
    .LABEL        = "Baro DEINIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },

  /* Ultraviolet */
  { .FLAG         = CHKLST_CYC_UV_INIT,
    .LABEL        = "UV INIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return (0 == uv.init()   ? 1 : 0);  },
    .POLL_FXN     = []() { return (uv.initialized() ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_CYC_UV_CONF,
    .LABEL        = "UV CONF",
    .DEP_MASK     = (CHKLST_CYC_UV_INIT),
    .DISPATCH_FXN = []() { return ((VEML6075Err::SUCCESS == uv.setIntegrationTime(VEML6075IntTime::IT_100MS)) ? 1 : -1);  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_CYC_UV_DEINIT,
    .LABEL        = "UV DEINIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },

  /* Thermocam */
  { .FLAG         = CHKLST_CYC_GRIDEYE_INIT,
    .LABEL        = "Thermocam INIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return (0 == grideye.init()   ? 1 : 0);  },
    .POLL_FXN     = []() { return (grideye.initialized() ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_CYC_GRIDEYE_CONF,
    .LABEL        = "Thermocam CONF",
    .DEP_MASK     = (CHKLST_CYC_GRIDEYE_INIT),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_CYC_GRIDEYE_DEINIT,
    .LABEL        = "Thermocam DEINIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },

  /* Mag ADC */
  { .FLAG         = CHKLST_CYC_MAG_ADC_INIT,
    .LABEL        = "Mag ADC INIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() {
      if (!magneto.power()) return 0;
      mag_adc.verbosity(LOG_LEV_ERROR);
      sleep_ms(10);
      return (0 == mag_adc.init() ? 1 : 0);
    },
    .POLL_FXN     = []() { return (mag_adc.adcFound()  ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_CYC_MAG_ADC_CONF,
    .LABEL        = "Mag ADC CONF",
    .DEP_MASK     = (CHKLST_CYC_MAG_ADC_INIT),
    //.DISPATCH_FXN = []() { return ((0 == mag_adc.init()) ? 1 : 0);  },
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return ((MCP356xMode::CONTINUOUS == mag_adc.readMode()) ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_CYC_MAG_ADC_DEINIT,
    .LABEL        = "Mag ADC DEINIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },

  /*  */
  { .FLAG         = CHKLST_CYC_ANA_INIT,
    .LABEL        = "ANA(?) INIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_CYC_ANA_CONF,
    .LABEL        = "ANA(?) CONF",
    .DEP_MASK     = (CHKLST_CYC_ANA_INIT),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_CYC_ANA_DEINIT,
    .LABEL        = "ANA(?) DEINIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },

  /* Lux and IR */
  { .FLAG         = CHKLST_CYC_LUX_INIT,
    .LABEL        = "Lux/IR INIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return (0 == tsl2561.init()   ? 1 : 0);  },
    .POLL_FXN     = []() { return (tsl2561.initialized() ? 1 : 0);  }
  },
  { .FLAG         = CHKLST_CYC_LUX_CONF,
    .LABEL        = "Lux/IR CONF",
    .DEP_MASK     = (CHKLST_CYC_LUX_INIT),
    .DISPATCH_FXN = []() {
      tsl2561.integrationTime(TSLIntegrationTime::MS_101);
      return 1;
    },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_CYC_LUX_DEINIT,
    .LABEL        = "Lux/IR DEINIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },

  /* Inertials */
  { .FLAG         = CHKLST_CYC_IMU_INIT,
    .LABEL        = "IMU INIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_CYC_IMU_CONF,
    .LABEL        = "IMU CONF",
    .DEP_MASK     = (CHKLST_CYC_IMU_INIT),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_CYC_IMU_DEINIT,
    .LABEL        = "IMU DEINIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },

  /* Time-of-flight depth sensor */
  { .FLAG         = CHKLST_CYC_TOF_INIT,
    .LABEL        = "ToF INIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
    // tof.setTimeout(500);
    // ret_local = (0 == tof.init());
  },
  { .FLAG         = CHKLST_CYC_TOF_CONF,
    .LABEL        = "ToF CONF",
    .DEP_MASK     = (CHKLST_CYC_TOF_INIT),
    // tof.startContinuous(100);
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
  { .FLAG         = CHKLST_CYC_TOF_DEINIT,
    .LABEL        = "ToF DEINIT",
    .DEP_MASK     = (0),
    .DISPATCH_FXN = []() { return 1;  },
    .POLL_FXN     = []() { return 1;  }
  },
};

AsyncSequencer checklist_cyclic(CHECKLIST_CYCLIC, (sizeof(CHECKLIST_CYCLIC) / sizeof(CHECKLIST_CYCLIC[0])));
