#include "Motherflux0r.h"

#include <math.h>
#include <Arduino.h>
#include <StringBuilder.h>
#include <SensorFilter.h>
#include <ParsingConsole.h>
#include <StopWatch.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <SX8634.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include "VEML6075.h"
#include "ICM20948.h"
#include "BME280.h"
#include "AMG88xx.h"
#include "DRV425.h"
#include "TSL2561.h"
#include "TMP102.h"


/*******************************************************************************
* Global constants
*******************************************************************************/

/* Audio related */
static const uint16_t BIN_INDICIES[] = {
  0,   0,   1,   1,   2,   2,   3,   3,   4,   4,   5,   5,   6,   6,   7,   7,
  8,   8,   9,   9,   10,  10,  11,  11,  12,  12,  13,  13,  14,  14,  15,  15,
  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,
  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,
  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,
  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,
  73,  74,  75,  76,  77,  78,  79,  81,  82,  84,  85,  87,  88,  90,  91,  93,
  94,  96,  97,  99,  100, 102, 103, 105, 106, 108, 109, 111, 112, 114, 115, 117,
  118, 120, 121, 123, 124, 126, 127, 129, 130, 132, 133, 136, 137, 140, 141, 144,
  145, 148, 149, 152, 153, 156, 157, 160, 161, 164, 165, 168, 169, 172, 173, 176,
  177, 180, 181, 184, 185, 188, 189, 193, 194, 198, 199, 203, 204, 208, 209, 213,
  214, 218, 219, 223, 224, 228, 229, 233, 234, 238, 239, 243, 244, 249, 250, 255
};

// Thermopile constants
const float THERM_TEMP_MAX = 150.0;
const float THERM_TEMP_MIN = 0.0;


/*******************************************************************************
* Globals
*******************************************************************************/
Adafruit_SSD1331 display = Adafruit_SSD1331(&SPI, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);

/* Audio library... */
AudioSynthWaveformSine   sineL;          //xy=86.00000762939453,296.0000877380371
AudioSynthWaveformSine   sineR;          //xy=86.00000762939453,329.00012397766113
AudioSynthNoisePink      pinkNoise;          //xy=86.0000114440918,364.0000858306885
//AudioInputAnalog         light_adc(ANA_LIGHT_PIN);  //xy=88.00000381469727,389.0000171661377
//AudioInputAnalog         mic_adc(MIC_ANA_PIN);      //xy=89.00000762939453,422.4285945892334
AudioPlayQueue           queueL;         //xy=97.0000228881836,200.71429824829102
AudioPlayQueue           queueR;         //xy=97.0000228881836,236.00013542175293
AudioMixer4              mixerL;         //xy=329.00008392333984,224.71430587768555
AudioMixer4              mixerR;         //xy=330.00008392333984,290.7142753601074
AudioMixer4              mixerFFT;         //xy=338.0000457763672,357.0000801086426
AudioAnalyzeFFT256       fft256_1;       //xy=491.00001525878906,357.00015449523926
AudioAmplifier           ampR;           //xy=468.0000534057617,285.0000114440918
AudioAmplifier           ampL;           //xy=469.0000190734863,233.0000057220459
AudioOutputI2S           i2s_dac;           //xy=495.00001525878906,269.0000629425049

// AudioConnection          patchCord1(mic_adc, 0, mixerFFT, 3);
// AudioConnection          patchCord2(mic_adc, 0, mixerR, 3);
// AudioConnection          patchCord3(mic_adc, 0, mixerL, 3);
AudioConnection          patchCord4(sineL, 0, mixerFFT, 0);
AudioConnection          patchCord5(sineL, 0, mixerL, 1);
AudioConnection          patchCord6(sineR, 0, mixerR, 1);
AudioConnection          patchCord7(pinkNoise, 0, mixerFFT, 1);
// AudioConnection          patchCord8(light_adc, 0, mixerFFT, 2);
// AudioConnection          patchCord9(light_adc, 0, mixerR, 2);
// AudioConnection          patchCord10(light_adc, 0, mixerL, 2);
// AudioConnection          patchCord11(queueL, 0, mixerL, 0);
// AudioConnection          patchCord12(queueR, 0, mixerR, 0);
AudioConnection          patchCord13(mixerL, ampL);
AudioConnection          patchCord14(mixerR, ampR);
AudioConnection          patchCord15(mixerFFT, fft256_1);
AudioConnection          patchCord16(ampR, 0, i2s_dac, 1);
AudioConnection          patchCord17(ampL, 0, i2s_dac, 0);


uint8_t fft_bars_shown[96];


BME280Settings baro_settings(
  0x76,
  BME280OSR::X1,
  BME280OSR::X1,
  BME280OSR::X1,
  BME280Mode::Forced,
  BME280StandbyTime::StandbyTime_1000ms,
  BME280Filter::Off
);

/* Touch board */
static const SX8634Opts _touch_opts(
  SX8634_DEFAULT_I2C_ADDR,  // i2c address
  TOUCH_RESET_PIN,          // Reset pin. Output. Active low.
  TOUCH_IRQ_PIN             // IRQ pin. Input. Active low. Needs pullup.
);
static SX8634* touch = nullptr;


/* Sensor representations... */
DRV425 magneto(DRV425_ADC_IRQ_PIN, DRV425_GPIO_IRQ_PIN, 255, DRV425_CS_PIN);   // No GPIO reset pin.
TMP102 tmp102(0x49, 255);    // No connection to the alert pin.
GridEYE grideye(0x69, AMG8866_IRQ_PIN);
VEML6075 uv;
ICM_20948_SPI imu;
TSL2561 tsl2561(0x39, TSL2561_IRQ_PIN);
BME280I2C baro(baro_settings);

/* Immediate data... */
static Vector3f64 grav;       // Gravity vector from the IMU.
static Vector3f64 acc_vect;   // Acceleration vector from the IMU.
static Vector3f64 gyr_vect;   // Gyroscopic vector from the IMU.
static Vector3f64 mag_vect0;  // Magnetism vector from the IMU.
static Vector3f64 mag_vect1;  // Magnetism vector from the DRV425 complex.
static float    altitude            = 0.0;    // BME280
static float    dew_point           = 0.0;    // BME280
static float    sea_level           = 0.0;    // BME280
static float    therm_midpoint_lock = 0.0;    // GidEye

/* Data buffers for sensors. */
static SensorFilter<float> graph_array_pressure(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_humidity(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_air_temp(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_psu_temp(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_uva(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_uvb(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_uvi(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_ana_light(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_visible(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_therm_mean(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_therm_frame(FilteringStrategy::MOVING_AVG, 64, 0);
static SensorFilter<float> graph_array_mag_confidence(FilteringStrategy::RAW, 96, 0);

/* Profiling data */
static StopWatch stopwatch_main_loop_time;
static StopWatch stopwatch_display;
static StopWatch stopwatch_sensor_baro;
static StopWatch stopwatch_sensor_uv;
static StopWatch stopwatch_sensor_grideye;
static StopWatch stopwatch_sensor_imu;
static StopWatch stopwatch_sensor_lux;
static StopWatch stopwatch_sensor_tmp102;
static StopWatch stopwatch_sensor_mag;
static StopWatch stopwatch_touch_poll;
static StopWatch stopwatch_app_app_select;
static StopWatch stopwatch_app_touch_test;
static StopWatch stopwatch_app_configurator;
static StopWatch stopwatch_app_data_mgmt;
static StopWatch stopwatch_app_synthbox;
static StopWatch stopwatch_app_comms;
static StopWatch stopwatch_app_meta;
static StopWatch stopwatch_app_i2c_scanner;
static StopWatch stopwatch_app_tricorder;
static StopWatch stopwatch_app_standby;
static StopWatch stopwatch_app_suspend;
static SensorFilter<float> graph_array_cpu_time(FilteringStrategy::MOVING_MED, 96, 0);
static SensorFilter<float> graph_array_frame_rate(FilteringStrategy::RAW, 96, 0);


/* Cheeseball async support stuff. */
static uint32_t boot_time         = 0;      // millis() at boot.
static uint32_t config_time       = 0;      // millis() at end of setup().
static uint32_t off_time_vib      = 0;      // millis() when vibrator should be disabled.
static uint32_t off_time_led_r    = 0;      // millis() when LED_R should be disabled.
static uint32_t off_time_led_g    = 0;      // millis() when LED_G should be disabled.
static uint32_t off_time_led_b    = 0;      // millis() when LED_B should be disabled.
static uint32_t last_interaction  = 0;      // millis() when the user last interacted.
static uint32_t disp_update_last  = 0;      // millis() when the display last updated.
static uint32_t disp_update_next  = 0;      // millis() when the display next updates.
static uint32_t disp_update_rate  = 1;      // Update in Hz for the display
static uint32_t disp_selector_off = 0;      // millis() when the display dialog should timeout.

DataVis current_data_vis = DataVis::TEXT;


/* Console junk... */
ParsingConsole console(128);
static const TCode arg_list_0[]       = {TCode::NONE};
static const TCode arg_list_1_str[]   = {TCode::STR,   TCode::NONE};
static const TCode arg_list_1_uint[]  = {TCode::UINT,  TCode::NONE};
static const TCode arg_list_1_float[] = {TCode::FLOAT, TCode::NONE};
static const TCode arg_list_2_uint[]  = {TCode::UINT,  TCode::UINT,  TCode::NONE};
static const TCode arg_list_3_uint[]  = {TCode::UINT,  TCode::UINT,  TCode::UINT,  TCode::NONE};
static const TCode arg_list_4_uuff[]  = {TCode::UINT,  TCode::UINT,  TCode::FLOAT, TCode::FLOAT, TCode::NONE};
static const TCode arg_list_4_float[] = {TCode::FLOAT, TCode::FLOAT, TCode::FLOAT, TCode::FLOAT, TCode::NONE};

/* Application tracking and interrupts... */
static AppID    active_app          = AppID::APP_SELECT;
static AppID    drawn_app           = AppID::META;
static AppID    app_page            = AppID::TRICORDER;
static AppID    app_previous        = AppID::APP_SELECT;
static bool     dirty_button        = false;
static bool     dirty_slider        = false;
static bool     imu_irq_fired       = false;


/*******************************************************************************
* ISRs
*******************************************************************************/
void imu_isr_fxn() {         imu_irq_fired = true;        }



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


/*******************************************************************************
* Display functions
*******************************************************************************/
/*
* Draws the basics of the UI.
* 96x64
*/
void redraw_app_window(const char* title, uint8_t pages, uint8_t active_page) {
  display.fillScreen(BLACK);
  display.setTextSize(0);
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  //display.setTextColor(MAGENTA, BLACK);
  display.print(title);
  //display.drawLine(0, 10, display.width()-1, 10, WHITE);
  //display.drawLine(0, display.height()-1, display.width()-1, display.height()-1, WHITE);
  //display.drawLine(0, 10, 0, display.height()-1, WHITE);
  //display.drawLine(display.width()-1, 10, display.width()-1, display.height()-1, WHITE);
  //display.drawRect(0, 14, display.width()-1, display.height()-15, WHITE);
  display.drawFastHLine(0, 9, display.width(), WHITE);
  //render_button_icon(2, 46, 0, WHITE);
  //render_button_icon(3, 55, 0, WHITE);
  app_previous = drawn_app;
  drawn_app = active_app;
}


/*
* Draws the meta app.
*/
void redraw_meta_window() {
  if (drawn_app != active_app) {
    dirty_slider = true;
  }

  if (dirty_slider) {   // Initial frame changes go here.
    redraw_app_window("Meta", 0, 0);
    if (touch->sliderValue() <= 7) {
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
      display.println("Touch:  ");
      display.println("Uptime: ");
      display.print("FPS:    ");
    }
    else if (touch->sliderValue() <= 22) {
      display.setCursor(0, 11);
      display.setTextSize(0);
      display.setTextColor(WHITE);
      display.println("ML Worst");
      display.println("ML Best");
      display.println("ML Mean");
      display.println("ML Last");
    }
    dirty_slider = false;
  }

  if (touch->sliderValue() <= 7) {
    display.setTextColor(CYAN, BLACK);
    display.setCursor(47, 35);
    SX8634OpMode tmode = touch->operationalMode();
    display.print(touch->getModeStr(tmode));
    display.setCursor(47, 43);
    display.print(millis() - boot_time);
    display.setCursor(47, 51);
    display.print(disp_update_rate);
  }
  else if (touch->sliderValue() <= 15) {
    if (graph_array_frame_rate.dirty()) {
      draw_graph_obj(
        0, 10, 96, 45, CYAN,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
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
  else if (touch->sliderValue() <= 22) {
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
  else if (touch->sliderValue() <= 30) {
  }
  else if (touch->sliderValue() <= 37) {
  }
  else if (touch->sliderValue() <= 45) {
  }
  else if (touch->sliderValue() <= 52) {
  }
  else {
    // CPU load metrics
    if (graph_array_cpu_time.dirty()) {
      draw_graph_obj(
        0, 10, 96, 45, 0xFE00,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
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

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the configurator app.
*/
void redraw_configurator_window() {
  if (drawn_app != active_app) {
    redraw_app_window("Configurator", 0, 0);
    display.setCursor(0, 11);
  }

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Unit will drive while daydreaming.
*/
void redraw_hot_standby_window() {
  if (drawn_app != active_app) {
    display.fillScreen(BLACK);
  }

  if (dirty_button) {
    if (touch->buttonStates() == 0x0028) {
      // Buttons 0 and 2 (and ONLY those buttons) must be
      //   pressed to turn the UI elements back on.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}

/*
* Unit will go to sleep at the wheel.
*/
void redraw_suspended_window() {
  if (drawn_app != active_app) {
    display.fillScreen(BLACK);
    // TODO: Put things into reset states.
    // TODO: Power off non-essential rails.
    // TODO: Scale back the CPU clock.
    // TODO: Set wake sources.
  }

  if (dirty_button) {
    if (touch->buttonStates() == 0x0050) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the data manager app.
*/
void redraw_data_mgmt_window() {
  if (drawn_app != active_app) {
    redraw_app_window("Data Manager", 0, 0);
  }

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the comms app.
*/
void redraw_comms_root_window() {
  if (drawn_app != active_app) {
    redraw_app_window("Comms", 0, 0);
  }

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


static uint32_t last_i2c_scan = 0;

/*
* Draws the I2C app.
*/
void redraw_i2c_probe_window() {
  if (drawn_app != active_app) {
    redraw_app_window("I2C Scanner", 0, 0);
  }
  StringBuilder disp_str;

  if ((last_i2c_scan + 330) <= millis()) {
    if (touch->buttonPressed(4)) {
      redraw_app_window("I2C Probe Wire", 0, 0);
      for (uint8_t addr = 0; addr < 0x80; addr++) {
        Wire.beginTransmission(addr);
        if (0 == Wire.endTransmission()) {
          disp_str.concatf("0x%02x ", addr);
          display.drawPixel(addr & 0x1F, 11 + (addr >> 5), RED);
        }
      }
      if (disp_str.length() > 0) {
        display.setCursor(0, 20);
        display.print((char*) disp_str.string());
      }
      last_i2c_scan = millis();
    }
    else if (touch->buttonPressed(1)) {
      redraw_app_window("I2C Probe Wire1", 0, 0);
      for (uint8_t addr = 0; addr < 0x80; addr++) {
        Wire1.beginTransmission(addr);
        if (0 == Wire1.endTransmission()) {
          disp_str.concatf("0x%02x ", addr);
          display.drawPixel(addr & 0x1F, 11 + (addr >> 5), RED);
        }
      }
      if (disp_str.length() > 0) {
        display.setCursor(0, 20);
        display.print((char*) disp_str.string());
      }
      last_i2c_scan = millis();
    }
  }

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the touch test app.
*/
void redraw_touch_test_window() {
  if (drawn_app != active_app) {
    redraw_app_window("SX8634 diag", 0, 0);
    display.setCursor(0, 37);
    display.setTextColor(WHITE);
    display.print("Slider:");
    dirty_button = true;
    dirty_slider = true;
  }

  if (dirty_button) {
    const uint8_t x_coords[] = {37, 57, 77, 77, 57, 37};
    const uint8_t y_coords[] = {14, 14, 14, 34, 34, 34};
    for (uint8_t i = 0; i < 6; i++) {
      if (touch->buttonPressed(i)) {
        display.fillRoundRect(x_coords[i], y_coords[i], 18, 18, 4, RED);
        if ((0 == i) && (touch->sliderValue() == 0)) {
          // Interpret a cancel press as a return to APP_SELECT.
          active_app = AppID::APP_SELECT;
        }
      }
      else {
        display.fillRect(x_coords[i], y_coords[i], 18, 18, BLACK);
        display.drawRoundRect(x_coords[i], y_coords[i], 18, 18, 4, GREEN);
      }
    }
    display.setCursor(0, 29);
    display.fillRect(0, 29, 28, 8, BLACK);
    display.setTextColor(RED, BLACK);
    display.print(touch->buttonStates(), HEX);
    dirty_button = false;
  }
  if (dirty_slider) {
    uint16_t sval = 60 - touch->sliderValue();
    dirty_slider = false;
    //display.fillRect(0, 45, 28, 8, BLACK);
    display.setTextSize(0);
    display.setCursor(0, 45);
    display.setTextColor(WHITE, BLACK);
    display.print(sval);

    uint8_t slider_pix = 2 + (((1+sval) / 61.0f) * (display.width()-5));
    display.fillRect(0, 55, display.width()-1, 7, BLACK);
    display.drawRoundRect(0, 54, display.width(), 9, 3, GREEN);
    display.fillRect(slider_pix-1, 55, 3, 7, RED);
  }
}


/*
* Draws the tricorder app.
*/
void redraw_tricorder_window() {
  if (drawn_app != active_app) {
    redraw_app_window("Tricorder", 0, 0);
    dirty_slider = true;
  }

  if (dirty_slider) {
    if (touch->sliderValue() <= 45) {
      redraw_app_window("Tricorder", 0, 0);
    }
    else {
      //display.fillRect(0, 11, display.width()-1, display.height()-12, BLACK);
      display.fillScreen(BLACK);
    }
    dirty_slider = false;
  }

  if (touch->sliderValue() <= 7) {
    // Baro
    if (graph_array_humidity.dirty()) {
      draw_graph_obj(
        0, 10, 96, 37, 0x03E0,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
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
  else if (touch->sliderValue() <= 15) {
    // Baro
    if (graph_array_air_temp.dirty()) {
      draw_graph_obj(
        0, 10, 48, 37, 0x83D0,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
        &graph_array_air_temp
      );
    }
    if (graph_array_pressure.dirty()) {
      draw_graph_obj(
        48, 10, 48, 37, 0xFE00,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
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
  else if (touch->sliderValue() <= 22) {
    // TSL2561
    if (graph_array_visible.dirty()) {
      draw_graph_obj(
        0, 10, 96, 45, 0xF100,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
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
  else if (touch->sliderValue() <= 30) {
    // Analog light sensor
    graph_array_ana_light.feedFilter(analogRead(ANA_LIGHT_PIN) / 1024.0);
    draw_graph_obj(
      0, 10, 96, 45, 0xFE00,
      true, touch->buttonPressed(1), touch->buttonPressed(4),
      &graph_array_ana_light
    );
    display.setTextSize(0);
    display.setCursor(0, 56);
    display.setTextColor(WHITE);
    display.print("Light:  ");
    display.setTextColor(GREEN, BLACK);
    display.print(graph_array_ana_light.value());
  }
  else if (touch->sliderValue() <= 37) {
    if (touch->buttonPressed(5)) {
      // UVI
      if (graph_array_uvi.dirty()) {
        draw_graph_obj(
          0, 10, 96, 45, 0xF81F,
          true, touch->buttonPressed(1), touch->buttonPressed(4),
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
          true, touch->buttonPressed(1), touch->buttonPressed(4),
          &graph_array_uva
        );
      }
      if (graph_array_uvb.dirty()) {
        draw_graph_obj(
          48, 10, 48, 45, 0xF80F,
          true, touch->buttonPressed(1), touch->buttonPressed(4),
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
  else if (touch->sliderValue() <= 45) {
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
  else if (touch->sliderValue() <= 52) {
    display.setCursor(0, 0);
    display.setTextColor(0xFC00, BLACK);
    display.print("Magnetometer");
    if (!touch->buttonPressed(5)) {
      display.setTextColor(WHITE, BLACK);
      Vector3f64* mag_vect = magneto.getFieldVector();
      float bearing_north = 0.0;
      float bearing_mag = 0.0;
      magneto.getBearing(HeadingType::TRUE_NORTH, &bearing_north);
      magneto.getBearing(HeadingType::MAGNETIC_NORTH, &bearing_mag);
      draw_compass(0, 11, 44, 44, false, touch->buttonPressed(4), bearing_mag, bearing_north);
      display.setCursor(0, 57);
      display.print(mag_vect->length());
      display.print(" uT");
    }
    else {
      draw_graph_obj(
        0, 10, 96, 45, 0xF81F,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
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
      bool lock_range_to_current = touch->buttonPressed(1);
      bool lock_range_to_absolute = touch->buttonPressed(5);
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

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the app selection window.
*/
void redraw_app_select_window() {
  const uint8_t ICON_SIZE    = 32;
  const uint8_t TEXT_OFFSET = ICON_SIZE+11;

  if (drawn_app != active_app) {
    redraw_app_window("Select Function", 0, 0);
    dirty_slider = true;
  }

  if (dirty_slider) {
    display.fillRect(0, 11, display.width()-1, display.height()-12, BLACK);
    display.setCursor(0, TEXT_OFFSET);
    display.setTextColor(WHITE);
    display.print("Fxn: ");
    display.setTextColor(MAGENTA, BLACK);
    if (touch->sliderValue() <= 7) {
      display.print("Touch Diag");
      app_page = AppID::TOUCH_TEST;
    }
    else if (touch->sliderValue() <= 15) {
      display.print("Settings");
      app_page = AppID::CONFIGURATOR;
    }
    else if (touch->sliderValue() <= 22) {
      display.print("Data MGMT");
      app_page = AppID::DATA_MGMT;
    }
    else if (touch->sliderValue() <= 30) {
      display.print("Synth Box");
      app_page = AppID::SYNTH_BOX;
    }
    else if (touch->sliderValue() <= 37) {
      display.print("Comms");
      app_page = AppID::COMMS_TEST;
    }
    else if (touch->sliderValue() <= 45) {
      display.print("Meta");
      app_page = AppID::META;
    }
    else if (touch->sliderValue() <= 52) {
      display.print("I2C Scanner");
      app_page = AppID::I2C_SCANNER;
    }
    else {
      display.print("Tricorder");
      app_page = AppID::TRICORDER;
    }
    dirty_slider = false;
  }

  if (dirty_button) {
    if (touch->buttonPressed(2)) {
      active_app = app_page;  // This will cause the app to switch.
    }
    else if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a request to doze.
      display.fillScreen(BLACK);
      active_app = AppID::HOT_STANDBY;
    }
    dirty_button = false;
  }
}


/*
* Draws the FFT app.
*/
void redraw_fft_window() {
  const float SCALER_PIX = 64;
  const int SHOWN_DECAY  = 1;

  if (drawn_app != active_app) {
    redraw_app_window("FFT", 0, 0);
    //display.fillScreen(BLACK);
  }

  if (dirty_slider) {
    sineL.frequency(10+300*touch->sliderValue());
    sineR.frequency(610+300*touch->sliderValue());
    dirty_slider = false;
  }

  float fft_bins[96];
  for (uint8_t i = 0; i < 96; i++) {
    fft_bins[i] = fft256_1.read(BIN_INDICIES[i << 1], BIN_INDICIES[(i << 1) + 1]);
  }
  for (uint8_t i = 0; i < 96; i++) {
    uint8_t scaled_val = fft_bins[i] * SCALER_PIX;
    uint y_real  = (display.height()-1) - scaled_val;
    uint h_real  = (display.height()-1) - scaled_val;
    display.drawFastVLine(i, 0, display.height()-1, BLACK);
    if (scaled_val >= fft_bars_shown[i]) {
      fft_bars_shown[i] = scaled_val;
      display.drawFastVLine(i, y_real, h_real, WHITE);
    }
    else {
      if (fft_bars_shown[i] > 0) fft_bars_shown[i] = fft_bars_shown[i] - SHOWN_DECAY;
      uint y_decay = (display.height()-1) - fft_bars_shown[i];
      uint h_decay = y_decay - h_real;
      display.drawFastVLine(i, y_decay, h_decay, WHITE);
      display.drawFastVLine(i, y_real, h_real, GREEN);
    }
  }
  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


void render_button_icon(uint8_t sym, int x, int y, uint16_t color) {
  const uint8_t ICON_SIZE = 7;
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  switch (sym) {
    case 0:
      x0 = x + (ICON_SIZE >> 1);
      y0 = y;
      x1 = x;
      y1 = y + ICON_SIZE;
      x2 = x + ICON_SIZE;
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case 1:
      x0 = x;
      y0 = y;
      x1 = x + ICON_SIZE;
      y1 = y;
      x2 = x + (ICON_SIZE >> 1);
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case 2:
      x0 = x + ICON_SIZE;
      y0 = y;
      x1 = x;
      y1 = y + (ICON_SIZE >> 1);
      x2 = x + ICON_SIZE;
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case 3:
      x0 = x;
      y0 = y;
      x1 = x + ICON_SIZE;
      y1 = y + (ICON_SIZE >> 1);
      x2 = x;
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case 4:
      display.drawBitmap(x, y, bitmapPointer(ICON_ACCEPT), 9, 9, color);
      break;
    case 5:
      //display.drawLine(0, 10, display.width()-1, 10, WHITE);
      display.drawBitmap(x, y, bitmapPointer(ICON_CANCEL), 9, 9, color);
      break;
  }
}



/*
* Called at the frame-rate interval for the display.
*/
void updateDisplay() {
  switch (active_app) {
    case AppID::APP_SELECT:
      stopwatch_app_app_select.markStart();
      redraw_app_select_window();
      stopwatch_app_app_select.markStop();
      break;
    case AppID::TOUCH_TEST:
      stopwatch_app_touch_test.markStart();
      redraw_touch_test_window();
      stopwatch_app_touch_test.markStop();
      break;
    case AppID::CONFIGURATOR:
      stopwatch_app_configurator.markStart();
      redraw_configurator_window();
      stopwatch_app_configurator.markStop();
      break;
    case AppID::DATA_MGMT:
      stopwatch_app_data_mgmt.markStart();
      redraw_data_mgmt_window();
      stopwatch_app_data_mgmt.markStop();
      break;
    case AppID::SYNTH_BOX:
      stopwatch_app_synthbox.markStart();
      redraw_fft_window();
      stopwatch_app_synthbox.markStop();
      break;
    case AppID::COMMS_TEST:
      stopwatch_app_comms.markStart();
      redraw_comms_root_window();
      stopwatch_app_comms.markStop();
      break;
    case AppID::META:
      stopwatch_app_meta.markStart();
      redraw_meta_window();
      stopwatch_app_meta.markStop();
      break;
    case AppID::I2C_SCANNER:
      stopwatch_app_i2c_scanner.markStart();
      redraw_i2c_probe_window();
      stopwatch_app_i2c_scanner.markStop();
      break;
    case AppID::TRICORDER:
      stopwatch_app_tricorder.markStart();
      redraw_tricorder_window();
      stopwatch_app_tricorder.markStop();
      break;
    case AppID::HOT_STANDBY:
      stopwatch_app_standby.markStart();
      redraw_hot_standby_window();
      stopwatch_app_standby.markStop();
      break;
    case AppID::SUSPEND:
      stopwatch_app_suspend.markStart();
      redraw_suspended_window();
      stopwatch_app_suspend.markStop();
      break;
  }
}



/*******************************************************************************
* Sensor service functions
*******************************************************************************/

/*
* Reads the VEML6075 and adds the data to the pile.
*/
int8_t read_uv_sensor() {
  int8_t ret = 0;
  graph_array_uva.feedFilter(uv.uva());
  graph_array_uvb.feedFilter(uv.uvb());
  graph_array_uvi.feedFilter(uv.index());
  return ret;
}


/*
* Reads the BME280 and adds the data to the pile.
*/
int8_t read_baro_sensor() {
  int8_t ret = 0;
  altitude  = baro.Altitude(baro.pres());
  dew_point = baro.DewPoint(baro.temp(), baro.hum());
  sea_level = baro.EquivalentSeaLevelPressure(altitude, baro.temp(), baro.pres());
  graph_array_humidity.feedFilter(baro.hum());
  graph_array_air_temp.feedFilter(baro.temp());
  graph_array_pressure.feedFilter(baro.pres());
  return ret;
}


/*
* Reads the IMU and optionally integrates for orientation.
*/
int8_t read_imu() {
  int8_t ret = 0;
  imu.getAGMT();
  return ret;
}


/*
* Reads the TSL2561 and adds the data to the pile.
*/
int8_t read_visible_sensor() {
  int8_t ret = 0;
  ret = graph_array_visible.feedFilter(1.0 * tsl2561.getLux());
  return ret;
}


/*
* Reads the TMP102 (near the PSU and battery) and adds the data to the pile.
*/
int8_t read_battery_temperature_sensor() {
  return graph_array_psu_temp.feedFilter(tmp102.temperature());
}


/*
* Reads the GridEye sensor and adds the data to the pile.
*/
int8_t read_thermopile_sensor() {
  int8_t ret = 0;
  for (uint8_t i = 0; i < 8; i++) {
    for (uint8_t n = 0; n < 8; n++) {
      uint8_t pix_idx = (7 - i) | (n << 3);  // Sensor is rotated 90-deg.
      graph_array_therm_frame.feedFilter(grideye.getPixelTemperature(pix_idx));
    }
  }
  graph_array_therm_mean.feedFilter(graph_array_therm_frame.value());
  return ret;
}

/*
* Reads the magnetometer and adds the data to the pile.
*/
int8_t read_magnetometer_sensor() {
  Vector3f64* mag_vect = magneto.getFieldVector();
  // TODO: Move into Compass class.
  // TODO: Should be confidence
  graph_array_mag_confidence.feedFilter(mag_vect->length());
  return 0;
}



/*******************************************************************************
* Touch callbacks
*******************************************************************************/

static void cb_button(int button, bool pressed) {
  last_interaction = millis();
  if (pressed) {
    vibrateOn(19);
  }
  else {
  }
  ledOn(LED_B_PIN, 2, 3500);
  dirty_button = true;
}


static void cb_slider(int slider, int value) {
  last_interaction = millis();
  ledOn(LED_R_PIN, 60, 3500);
  dirty_slider = true;
}


static void cb_longpress(int button, uint32_t duration) {
  ledOn(LED_G_PIN, 50, 3500);
}


/*******************************************************************************
* Console callbacks
*******************************************************************************/

int callback_help(StringBuilder* text_return, StringBuilder* args) {
  if (0 < args->count()) {
    console.printHelp(text_return, args->position_trimmed(0));
  }
  else {
    console.printHelp(text_return);
  }
  return 0;
}

int callback_print_history(StringBuilder* text_return, StringBuilder* args) {
  console.printHistory(text_return);
  return 0;
}

int callback_reboot(StringBuilder* text_return, StringBuilder* args) {
  text_return->concat("Unimplemented\n");
  return 0;
}


int callback_print_sensor_profiler(StringBuilder* text_return, StringBuilder* args) {
  if (args->count() > 0) {
    stopwatch_sensor_baro.reset();
    stopwatch_sensor_uv.reset();
    stopwatch_sensor_grideye.reset();
    stopwatch_sensor_imu.reset();
    stopwatch_sensor_lux.reset();
    stopwatch_sensor_tmp102.reset();
    stopwatch_sensor_mag.reset();
    stopwatch_touch_poll.reset();
  }
  StopWatch::printDebugHeader(text_return);
  stopwatch_sensor_baro.printDebug("Baro", text_return);
  stopwatch_sensor_uv.printDebug("UV", text_return);
  stopwatch_sensor_grideye.printDebug("GridEye", text_return);
  stopwatch_sensor_imu.printDebug("IMU", text_return);
  stopwatch_sensor_lux.printDebug("TSL2561", text_return);
  stopwatch_sensor_tmp102.printDebug("PSU Temp", text_return);
  stopwatch_sensor_mag.printDebug("Magnetometer", text_return);
  stopwatch_touch_poll.printDebug("Touch", text_return);
  return 0;
}


int callback_print_app_profiler(StringBuilder* text_return, StringBuilder* args) {
  if (args->count() > 0) {
    stopwatch_main_loop_time.reset();
    stopwatch_display.reset();
    stopwatch_app_app_select.reset();
    stopwatch_app_touch_test.reset();
    stopwatch_app_configurator.reset();
    stopwatch_app_data_mgmt.reset();
    stopwatch_app_synthbox.reset();
    stopwatch_app_comms.reset();
    stopwatch_app_meta.reset();
    stopwatch_app_i2c_scanner.reset();
    stopwatch_app_tricorder.reset();
    stopwatch_app_standby.reset();
    stopwatch_app_suspend.reset();
  }
  StopWatch::printDebugHeader(text_return);
  stopwatch_main_loop_time.printDebug("Main loop", text_return);
  stopwatch_display.printDebug("Display", text_return);
  stopwatch_app_app_select.printDebug("APP_SELECT", text_return);
  stopwatch_app_touch_test.printDebug("TOUCH TEST", text_return);
  stopwatch_app_configurator.printDebug("CONFIGURATOR", text_return);
  stopwatch_app_data_mgmt.printDebug("DATA MGMT", text_return);
  stopwatch_app_synthbox.printDebug("SYNTH BOX", text_return);
  stopwatch_app_comms.printDebug("COMMS", text_return);
  stopwatch_app_meta.printDebug("META", text_return);
  stopwatch_app_i2c_scanner.printDebug("I2C SCANNER", text_return);
  stopwatch_app_tricorder.printDebug("TRICORDER", text_return);
  stopwatch_app_standby.printDebug("STANDBY", text_return);
  stopwatch_app_suspend.printDebug("SUSPEND", text_return);
  return 0;
}


int callback_touch_info(StringBuilder* text_return, StringBuilder* args) {
  touch->printDebug(text_return);
  return 0;
}


int callback_touch_reset(StringBuilder* text_return, StringBuilder* args) {
  text_return->concat("SX8634 reset ");
  text_return->concat((0 == touch->reset()) ? "succeded" : "failed");
  return 0;
}


int callback_touch_mode(StringBuilder* text_return, StringBuilder* args) {
  if (args->count() > 0) {
    int mode_int = args->position_as_int(0);
    switch (mode_int) {
      case 0:
      case 1:
      case 2:
      case 3:
        if (0 == touch->setMode((SX8634OpMode) mode_int)) {
          text_return->concatf("SX8634 mode set to %s.\n", SX8634::getModeStr((SX8634OpMode) mode_int));
        }
        else {
          text_return->concat("SX8634 mode set failed.\n");
        }
        break;
      default:
        return -1;
    }
  }
  else {
    text_return->concatf("SX8634 mode set to %s.\n", SX8634::getModeStr(touch->operationalMode()));
  }
  return 0;
}


int callback_display_rate(StringBuilder* text_return, StringBuilder* args) {
  if (args->count() > 0) {
    int rate_int = args->position_as_int(0);
    disp_update_rate = (rate_int > 0) ? rate_int : 0;
  }
  if (disp_update_rate) {
    text_return->concatf("Display capped at %ufps.\n", disp_update_rate);
  }
  else {
    text_return->concat("Display framerate is not limited.\n");
  }
  return 0;
}


int callback_display_test(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  uint32_t millis_0 = millis();
  uint32_t millis_1 = millis_0;
  switch (arg0) {
    case 0:    display.fillScreen(BLACK);                   break;
    case 1:    redraw_app_window("Test App Title", 0, 0);   break;
    case 2:    redraw_touch_test_window();                  break;
    case 3:    redraw_tricorder_window();                   break;
    case 4:    redraw_fft_window();                         break;
    case 5:
      display.setAddrWindow(0, 0, 96, 64);
      for (uint8_t h = 0; h < 64; h++) {
        for (uint8_t w = 0; w < 96; w++) {
          if (w > 83) {       display.writePixel(w, h, WHITE);     }
          else if (w > 71) {  display.writePixel(w, h, BLUE);      }
          else if (w > 59) {  display.writePixel(w, h, GREEN);     }
          else if (w > 47) {  display.writePixel(w, h, CYAN);      }
          else if (w > 35) {  display.writePixel(w, h, RED);       }
          else if (w > 23) {  display.writePixel(w, h, MAGENTA);   }
          else if (w > 11) {  display.writePixel(w, h, YELLOW);    }
          else {              display.writePixel(w, h, BLACK);     }
        }
      }
      display.endWrite();
      break;
    case 6:
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_THERMO), 14, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_IMU), 32, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_GPS), 32, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_LIGHT), 28, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_UVI), 32, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_SOUND), 40, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_RH), 22, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_MIC), 19, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_MAGNET), 22, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_BATTERY), 52, 32, 0xFFFF);
      delay(5000);
      break;
    case 7:
      display.fillScreen(BLACK);
      render_button_icon(0, 0, 0, WHITE);
      render_button_icon(1, 10, 0, WHITE);
      render_button_icon(2, 20, 0, WHITE);
      render_button_icon(3, 30, 0, WHITE);
      render_button_icon(4, 0, 20, WHITE);
      render_button_icon(5, 0, 20, WHITE);
      break;
    case 8:
      display.fillScreen(BLACK);
      draw_progress_bar_vertical(0, 0, 12, 64, CYAN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(0, 0, 12, 64, CYAN, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_vertical(14, 0, 7, 64, BLUE, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(14, 0, 7, 64, BLUE, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_vertical(23, 0, 7, 31, YELLOW, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(23, 0, 7, 31, YELLOW, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_vertical(23, 33, 7, 31, RED, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(23, 33, 7, 31, RED, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_vertical(32, 0, 24, 64, GREEN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(32, 0, 24, 64, GREEN, false, true, (i * 0.01));
        delay(40);
      }
      break;

    case 9:    // Progress bar test
      display.fillScreen(BLACK);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 0, 95, 12, BLUE, true, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_horizontal(0, 14, 95, 7, BLUE, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 14, 95, 7, BLUE, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_horizontal(0, 23, 46, 7, YELLOW, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 23, 46, 7, YELLOW, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_horizontal(48, 23, 46, 7, RED, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(48, 23, 46, 7, RED, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_horizontal(0, 34, 95, 14, GREEN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 34, 95, 14, GREEN, false, true, (i * 0.01));
        delay(40);
      }
      break;
    case 10:    // Vector display test
      display.fillScreen(BLACK);
      draw_3vector(0, 0, 50, 50, RED,    true,  false, 1.0, 0.0, 0.0);
      draw_3vector(0, 0, 50, 50, GREEN,  false, false, 0.0, 1.0, 0.0);
      draw_3vector(0, 0, 50, 50, BLUE,   false, false, 0.0, 0.0, 1.0);
      draw_3vector(0, 0, 50, 50, YELLOW, false, false, 1.0, 1.0, 0.0);
      draw_3vector(0, 0, 50, 50, CYAN,   false, false, 0.13, 0.65, 0.0);
      break;
    default:
      return -1;
  }
  millis_1 = millis();
  text_return->concatf("Display update took %ums\n", millis_1-millis_0);
  return 0;
}

int callback_vibrator_test(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = (0 < args->count()) ? args->position_as_int(0) : 40;  // Duration
  int arg1 = (1 < args->count()) ? args->position_as_int(1) : 4095;  // Intensity
  vibrateOn(arg0, arg1);
  return 0;
}

int callback_led_test(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  int arg1 = (1 < args->count()) ? args->position_as_int(1) : 10;    // Duration
  int arg2 = (2 < args->count()) ? args->position_as_int(2) : 2048;  // Intensity
  switch (arg0) {
    case 0:   ledOn(LED_R_PIN, arg1, arg2);  break;
    case 1:   ledOn(LED_G_PIN, arg1, arg2);  break;
    case 2:   ledOn(LED_B_PIN, arg1, arg2);  break;
    default:  return -1;
  }
  return 0;
}

int callback_fft_mix(StringBuilder* text_return, StringBuilder* args) {
  float arg0 = args->position_as_double(0);
  float arg1 = args->position_as_double(1);
  float arg2 = args->position_as_double(2);
  float arg3 = args->position_as_double(3);
  mixerFFT.gain(0, arg0);  // sineL
  mixerFFT.gain(1, arg1);  // sineR
  mixerFFT.gain(2, arg2);  // light_adc
  mixerFFT.gain(3, arg3);  // mic_adc
  return 0;
}

int callback_aout_mix(StringBuilder* text_return, StringBuilder* args) {
  float arg0 = args->position_as_double(0);
  float arg1 = args->position_as_double(1);
  float arg2 = args->position_as_double(2);
  float arg3 = args->position_as_double(3);
  mixerL.gain(0, arg0);  // queueL
  mixerL.gain(1, arg1);  // sineL
  mixerL.gain(2, arg2);  // light_adc
  mixerL.gain(3, arg3);  // mic_adc
  mixerR.gain(0, arg0);  // queueR
  mixerR.gain(1, arg1);  // sineR
  mixerR.gain(2, arg2);  // light_adc
  mixerR.gain(3, arg3);  // mic_adc
  return 0;
}

int callback_synth_set(StringBuilder* text_return, StringBuilder* args) {
  AudioSynthWaveformSine* synth = nullptr;
  int arg0 = args->position_as_int(0);
  switch (arg0) {
    case 0:   synth = &sineL;  break;
    case 1:   synth = &sineR;  break;
    default:  return -1;
  }
  if (1 < args->count()) {
    synth->frequency(args->position_as_int(1));
  }
  if (2 < args->count()) {
    synth->amplitude(args->position_as_double(2));
  }
  if (3 < args->count()) {
    synth->phase(args->position_as_double(3));
  }
  return 0;
}


int callback_active_app(StringBuilder* text_return, StringBuilder* args) {
  if (0 < args->count()) {
    int arg0 = args->position_as_int(0);
    switch (arg0) {
      case 0:   active_app = AppID::APP_SELECT;    break;
      case 1:   active_app = AppID::TOUCH_TEST;    break;
      case 2:   active_app = AppID::CONFIGURATOR;  break;
      case 3:   active_app = AppID::DATA_MGMT;     break;
      case 4:   active_app = AppID::SYNTH_BOX;     break;
      case 5:   active_app = AppID::COMMS_TEST;    break;
      case 6:   active_app = AppID::META;          break;
      case 7:   active_app = AppID::I2C_SCANNER;   break;
      case 8:   active_app = AppID::TRICORDER;     break;
      case 9:   active_app = AppID::HOT_STANDBY;   break;
      case 10:  active_app = AppID::SUSPEND;       break;
      default:
        text_return->concatf("Unsupported app: %d\n", arg0);
        return -1;
    }
  }
  else {   // No arguments means print the app index list.
    listAllApplications(text_return);
  }
  return 0;
}


int callback_sensor_info(StringBuilder* text_return, StringBuilder* args) {
  if (0 < args->count()) {
    int arg0 = args->position_as_int(0);
    switch ((SensorID) arg0) {
      case SensorID::MAGNETOMETER:   magneto.printDebug(text_return);  break;
      //case SensorID::BARO:           baro.printDebug(text_return);          break;
      //case SensorID::LIGHT:            break;
      //case SensorID::UV:             uv.printDebug(text_return);            break;
      //case SensorID::THERMOPILE:     grideye.printDebug(text_return);       break;
      //case SensorID::LUX:            tsl2561.printDebug(text_return);       break;
      //case SensorID::BATT_VOLTAGE:       break;
      //case SensorID::IMU:                break;
      //case SensorID::MIC:                break;
      //case SensorID::GPS:                break;
      //case SensorID::PSU_TEMP:       tmp102.printDebug(text_return);        break;
      default:
        text_return->concatf("Unsupported sensor: %d\n", arg0);
        return -1;
    }
  }
  else {   // No arguments means print the sensor index list.
    listAllSensors(text_return);
  }
  return 0;
}

int callback_sensor_filter_info(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  if (0 < args->count()) {
    switch ((SensorID) arg0) {
      case SensorID::MAGNETOMETER:
        break;
      case SensorID::BARO:
        text_return->concat("Baro Filters:\n");
        graph_array_humidity.printFilter(text_return);
        graph_array_air_temp.printFilter(text_return);
        graph_array_pressure.printFilter(text_return);
        graph_array_psu_temp.printFilter(text_return);
        break;
      case SensorID::LIGHT:
        graph_array_ana_light.printFilter(text_return);
        break;
      case SensorID::UV:
        text_return->concat("UV Filters:\n");
        graph_array_uva.printFilter(text_return);
        graph_array_uvb.printFilter(text_return);
        graph_array_uvi.printFilter(text_return);
        break;
      case SensorID::THERMOPILE:
        text_return->concat("GridEye Filters:\n");
        graph_array_therm_mean.printFilter(text_return);
        graph_array_therm_frame.printFilter(text_return);
        break;
      case SensorID::BATT_VOLTAGE:
      case SensorID::IMU:
      case SensorID::MIC:
      case SensorID::GPS:
      case SensorID::PSU_TEMP:
        break;
      case SensorID::LUX:
        text_return->concat("Lux Filters:\n");
        graph_array_visible.printFilter(text_return);
        break;
      default:
        text_return->concatf("Unsupported sensor: %d\n", arg0);
        return -1;
    }
  }
  else {   // No arguments means print the sensor index list.
    listAllSensors(text_return);
  }
  return 0;
}

int callback_meta_filter_info(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  if (0 < args->count()) {
    switch (arg0) {
      case 0:
        text_return->concat("CPU Time Filter:\n");
        graph_array_cpu_time.printFilter(text_return);
        break;
      case 1:
        text_return->concat("Framerate Filter:\n");
        graph_array_frame_rate.printFilter(text_return);
        break;
      default:
        text_return->concatf("Unsupported filter: %d\n", arg0);
        return -1;
    }
  }
  return 0;
}



int callback_sensor_filter_set_strat(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  int arg0 = args->position_as_int(0);
  int arg1 = args->position_as_int(1);
  uint8_t arg2 = (2 < args->count()) ? args->position_as_int(2) : 255;

  switch ((SensorID) arg0) {
    case SensorID::MAGNETOMETER:  ret = magneto.setFilter((FilteringStrategy) arg1);   break;
    case SensorID::BARO:
      switch (arg2) {
        case 0:    ret = graph_array_humidity.setStrategy((FilteringStrategy) arg1);   break;
        case 1:    ret = graph_array_air_temp.setStrategy((FilteringStrategy) arg1);   break;
        case 2:    ret = graph_array_pressure.setStrategy((FilteringStrategy) arg1);   break;
        case 255:
          if (0 == graph_array_humidity.setStrategy((FilteringStrategy) arg1)) {
            if (0 == graph_array_air_temp.setStrategy((FilteringStrategy) arg1)) {
              ret = graph_array_pressure.setStrategy((FilteringStrategy) arg1);
            }
          }
          break;
        default:
          ret = -3;
          break;
      }
      break;
    case SensorID::LIGHT:    ret = graph_array_ana_light.setStrategy((FilteringStrategy) arg1);   break;
    case SensorID::UV:
      switch (arg2) {
        case 0:    ret = graph_array_uva.setStrategy((FilteringStrategy) arg1);   break;
        case 1:    ret = graph_array_uvb.setStrategy((FilteringStrategy) arg1);   break;
        case 2:    ret = graph_array_uvi.setStrategy((FilteringStrategy) arg1);   break;
        case 255:
          if (0 == graph_array_uva.setStrategy((FilteringStrategy) arg1)) {
            if (0 == graph_array_uvb.setStrategy((FilteringStrategy) arg1)) {
              ret = graph_array_uvi.setStrategy((FilteringStrategy) arg1);
            }
          }
          break;
        default:
          ret = -3;
          break;
      }
      break;
    case SensorID::THERMOPILE:
      switch (arg2) {
        case 0:    ret = graph_array_therm_frame.setStrategy((FilteringStrategy) arg1);  break;
        case 1:    ret = graph_array_therm_mean.setStrategy((FilteringStrategy) arg1);   break;
        case 255:
          if (0 == graph_array_therm_frame.setStrategy((FilteringStrategy) arg1)) {
            ret = graph_array_therm_mean.setStrategy((FilteringStrategy) arg1);
          }
          break;
        default:
          ret = -3;
          break;
      }
      break;
    case SensorID::BATT_VOLTAGE:
    case SensorID::IMU:
    case SensorID::MIC:
    case SensorID::GPS:
      break;
    case SensorID::PSU_TEMP:      ret = graph_array_psu_temp.setStrategy((FilteringStrategy) arg1);    break;
    case SensorID::LUX:           ret = graph_array_visible.setStrategy((FilteringStrategy) arg1);     break;
    default:
      text_return->concatf("Unsupported sensor: %d\n", arg0);
      return -1;
  }
  if (-3 == ret) {
    text_return->concatf("Unknown datum index (%u) for sensor: %d\n", arg2, arg0);
  }
  else {
    text_return->concatf("Setting sensor %d filter strategy to %s returned %d.\n", arg0, getFilterStr((FilteringStrategy) arg1), ret);
  }
  return 0;
}


int callback_meta_filter_set_strat(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  int arg0 = args->position_as_int(0);
  uint8_t arg1 = args->position_as_int(1);
  switch (arg0) {
    case 0:    ret = graph_array_cpu_time.setStrategy((FilteringStrategy) arg1);     break;
    case 1:    ret = graph_array_frame_rate.setStrategy((FilteringStrategy) arg1);   break;
    default:
      text_return->concatf("Unsupported filter: %d\n", arg0);
      return -1;
  }
  text_return->concatf("Setting meta filter %d strategy to %s returned %d.\n", arg0, getFilterStr((FilteringStrategy) arg1), ret);
  return 0;
}


int callback_magnetometer_fxns(StringBuilder* text_return, StringBuilder* args) {
  int ret = -3;
  if (0 < args->count()) {
    ret = 0;
    switch (args->position_as_int(0)) {
      case 0:
        switch (args->position_as_int(1)) {
          default:
          case 0:   magneto.printDebug(text_return);           break;
          case 1:   magneto.printField(text_return);           break;
          case 2:   magneto.printfilter(text_return);          break;
          case 3:   magneto.printBearing(HeadingType::MAGNETIC_NORTH, text_return);  break;
          case 4:   magneto.printChannelValues(text_return);   break;
          case 5:   magneto.printRegs(text_return);            break;
          case 6:   magneto.printPins(text_return);            break;
          case 7:   magneto.printData(text_return);            break;
          case 8:   magneto.printTimings(text_return);         break;
          case 9:
            text_return->concatf("ADC Temperature: %u.\n", (uint8_t) magneto.getTemperature());
            break;
        }
        break;

      case 1:
        stopwatch_sensor_mag.markStart();
        ret = magneto.poll();
        stopwatch_sensor_mag.markStop();
        break;

      case 2:
        if (1 < args->count()) {
          ret = magneto.setGain((MCP356xGain) args->position_as_int(1));
        }
        text_return->concatf("Gain is now %u.\n", 1 << ((uint8_t) magneto.getGain()));
        break;

      case 3:
        if (1 < args->count()) {
          ret = magneto.setOversamplingRatio((MCP356xOversamplingRatio) args->position_as_int(1));
        }
        text_return->concatf("Oversampling ratio is now %u.\n", (uint8_t) magneto.getOversamplingRatio());
        break;

      case 4:
        if (1 < args->count()) {
          magneto.setMagGnomon((GnomonType) args->position_as_int(1));
        }
        magneto.printSpatialTransforms(text_return);
        break;

      case 5:
        if (1 < args->count()) {
          bool en = (0 != args->position_as_int(1));
          magneto.setBandwidth(en ? DRV425Bandwidth::BW0 : DRV425Bandwidth::BW1);
          text_return->concatf("Magnetometer bandwidth is %s\n", en ? "BW0" : "BW1");
        }
        break;
      case 6:   ret = magneto.reset();               break;
      case 7:   ret = magneto.refresh();             break;
      default:  ret = -3;                            break;
    }
  }
  if (-3 == ret) {
    text_return->concat("---< Magnetometer functions >--------------------------\n");
    text_return->concat("0:  Info\n");
    text_return->concat("1:  Poll\n");
    text_return->concat("2:  Gain\n");
    text_return->concat("3:  Oversampling ratio\n");
    text_return->concat("4:  Gnomons\n");
    text_return->concat("5:  Bandwidth\n");
    text_return->concat("6:  Reset\n");
    text_return->concat("7:  Refresh\n");
  }
  else if (0 != ret) {
    text_return->concatf("Function returned %d\n", ret);
  }
  else {
    text_return->concat("Success\n");
  }
  return ret;
}


int callback_sensor_init(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (1 == args->count()) {
    int arg0 = args->position_as_int(0);
    //int arg1 = args->position_as_int(1);
    switch ((SensorID) arg0) {
      case SensorID::MAGNETOMETER:   ret = magneto.init(&Wire1, &SPI);   break;
      case SensorID::BARO:           ret = baro.init(&Wire1);            break;
      case SensorID::LUX:            ret = tsl2561.init(&Wire1);         break;
      case SensorID::UV:             ret = uv.init(&Wire1);              break;
      case SensorID::THERMOPILE:     ret = grideye.init(&Wire1);         break;
      case SensorID::PSU_TEMP:       ret = tmp102.init(&Wire1);          break;
      //case SensorID::BATT_VOLTAGE:       break;
      //case SensorID::IMU:                break;
      //case SensorID::MIC:                break;
      //case SensorID::GPS:                break;
      //case SensorID::LIGHT:              break;
      default:
        text_return->concatf("Unsupported sensor: %d\n", arg0);
        return -1;
    }
    text_return->concatf("Sensor %d init() returned %d\n", arg0, ret);
  }
  else {   // No arguments means print the sensor index list.
    listAllSensors(text_return);
  }
  return 0;
}


int callback_sensor_enable(StringBuilder* text_return, StringBuilder* args) {
  if (0 < args->count()) {
    int arg0  = args->position_as_int(0);
    bool arg1 = (1 < args->count()) ? (1 == args->position_as_int(1)) : true;
    bool en   = false;
    switch ((SensorID) arg0) {
      case SensorID::MAGNETOMETER:  magneto.power(arg1);    en = magneto.power();     break;
      case SensorID::BARO:          en = baro.enabled();            break;
      case SensorID::LUX:           tsl2561.enabled(arg1);  en = tsl2561.enabled();   break;
      case SensorID::UV:            uv.enabled(arg1);       en = uv.enabled();        break;
      case SensorID::THERMOPILE:    grideye.enabled(arg1);  en = grideye.enabled();   break;
      case SensorID::BATT_VOLTAGE:  tmp102.enabled(arg1);   en = tmp102.enabled();    break;
      case SensorID::IMU:           break;
      case SensorID::MIC:           break;
      case SensorID::GPS:           break;
      case SensorID::PSU_TEMP:      break;
      case SensorID::LIGHT:         break;
      default:
        text_return->concatf("Unsupported sensor: %d\n", arg0);
        return -1;
    }
    text_return->concatf("Sensor %d is now %sabled\n", arg0, en?"en":"dis");
  }
  else {   // No arguments means print the sensor index list.
    listAllSensors(text_return);
  }
  return 0;
}


int callback_audio_volume(StringBuilder* text_return, StringBuilder* args) {
  if (1 == args->count()) {
    float arg0 = args->position_as_double(0);
    ampL.gain(arg0);
    ampR.gain(arg0);
    text_return->concatf("Audio volume: %.2f\n", arg0);
  }
  return 0;
}


int callback_cbor_tests(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (1 == args->count()) {
    int arg0 = args->position_as_int(0);
  }
  text_return->concat("CBOR test results: %d\n", ret);
  return ret;
}




/*******************************************************************************
* Setup function
*******************************************************************************/
void setup() {
  boot_time = millis();
  float percent_setup = 0.0;
  char* init_step_str = "";
  int cursor_height = 26;
  Serial.begin(115200);   // USB

  pinMode(IMU_IRQ_PIN, INPUT_PULLUP);
  pinMode(DRV425_CS_PIN, INPUT); // Wrong
  pinMode(PSU_SX_IRQ_PIN, INPUT);
  pinMode(ANA_LIGHT_PIN, INPUT);
  pinMode(LED_R_PIN, INPUT);
  pinMode(LED_G_PIN, INPUT);
  pinMode(LED_B_PIN, INPUT);

  SPI.setSCK(SPISCK_PIN);
  SPI.setMISO(SPIMISO_PIN);
  SPI.setMOSI(SPIMOSI_PIN);
  SPI.begin();

  Wire.setSDA(SDA0_PIN);
  Wire.setSCL(SCL0_PIN);
  Wire.setClock(400000);
  Wire.begin();
  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire.setClock(400000);
  Wire1.begin();

  Serial1.setRX(GPS_TX_PIN);
  Serial1.setTX(GPS_RX_PIN);
  Serial1.begin(9600);    // GPS

  Serial6.setRX(COMM_TX_PIN);
  Serial6.setTX(COMM_RX_PIN);
  Serial6.begin(115200);    // Comm
  AudioMemory(32);

  //SD.begin(BUILTIN_SDCARD);

  sineL.amplitude(1.0);
  sineL.frequency(440);
  sineL.phase(0);
  sineR.amplitude(0.8);
  sineR.frequency(660);
  sineR.phase(0);

  mixerFFT.gain(0, 0.0);  // sineL
  mixerFFT.gain(1, 1.0);  // sineR
  mixerFFT.gain(2, 0.0);  // light_adc
  mixerFFT.gain(3, 0.0);  // mic_adc

  mixerL.gain(0, 0.0);  // queueL
  mixerL.gain(1, 1.0);  // sineL
  mixerL.gain(2, 0.0);  // light_adc
  mixerL.gain(3, 0.0);  // mic_adc

  mixerR.gain(0, 0.0);  // queueR
  mixerR.gain(1, 1.0);  // sineR
  mixerR.gain(2, 0.0);  // light_adc
  mixerR.gain(3, 0.0);  // mic_adc

  pinkNoise.amplitude(1.0);

  ampL.gain(0.4);
  ampR.gain(0.4);

  analogWriteResolution(12);

  /* Allocate memory for the filters. */
  graph_array_pressure.init();
  graph_array_humidity.init();
  graph_array_air_temp.init();
  graph_array_psu_temp.init();
  graph_array_uva.init();
  graph_array_uvb.init();
  graph_array_uvi.init();
  graph_array_ana_light.init();
  graph_array_visible.init();
  graph_array_therm_mean.init();
  graph_array_therm_frame.init();
  graph_array_cpu_time.init();
  graph_array_frame_rate.init();
  graph_array_mag_confidence.init();

  display.begin();
  display.fillScreen(BLACK);
  display.setCursor(14, 0);
  display.setTextSize(1);
  display.println("Motherflux0r");
  display.setTextSize(0);
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, true, false, percent_setup);

  init_step_str = (const char*) "Audio           ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  delay(10);

  init_step_str = (const char*) "GridEye         ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (0 == grideye.init(&Wire1)) {
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }

  init_step_str = (const char*) "Magnetometer    ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  magneto.setOptions(COMPASS_FLAG_TILT_COMPENSATE, true);
  magneto.setOptions(COMPASS_FLAG_INVERT_X | COMPASS_FLAG_INVERT_Z, true);
  magneto.setGnomons(GnomonType::LH_NEG_Y, GnomonType::LH_POS_Z);
  if (0 == magneto.init(&Wire1, &SPI)) {
    magneto.setGravity(0.0, 0.0, 1.0);
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }

  init_step_str = (const char*) "Baro            ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (0 == baro.init(&Wire1)) {
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }

  init_step_str = (const char*) "UVI";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (VEML6075_ERROR_SUCCESS == uv.init(&Wire1)) {
  }
  else {
    display.setTextColor(RED, BLACK);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }

  init_step_str = (const char*) "Lux ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  int8_t lux_ret = tsl2561.init(&Wire1);
  if (0 == lux_ret) {
    tsl2561.integrationTime(TSLIntegrationTime::MS_101);
  }
  else {
    display.setTextColor(RED, BLACK);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    display.print(lux_ret);
    cursor_height += 8;
  }
  delay(10);

  init_step_str = (const char*) "PSU Temperature";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (0 == tmp102.init(&Wire)) {
  }
  else {
    display.setTextColor(RED, BLACK);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }

  init_step_str = (const char*) "Inertial        ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  //if (ICM_20948_Stat_Ok == imu.begin(IMU_CS_PIN, SPI, 6000000)) {
  //  imu.swReset();
  //  imu.sleep(false);
  //  imu.lowPower(false);
  //  imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mag), ICM_20948_Sample_Mode_Continuous);
  //  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  //  myFSS.a = gpm2;         // gpm2, gpm4, gpm8, gpm16
  //  myFSS.g = dps250;       // dps250, dps500, dps1000, dps2000
  //  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  //  // Set up Digital Low-Pass Filter configuration
  //  ICM_20948_dlpcfg_t myDLPcfg;      // Similar to FSS, this uses a configuration structure for the desired sensors
  //  myDLPcfg.a = acc_d473bw_n499bw;   // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
  //                                    // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
  //                                    // acc_d111bw4_n136bw
  //                                    // acc_d50bw4_n68bw8
  //                                    // acc_d23bw9_n34bw4
  //                                    // acc_d11bw5_n17bw
  //                                    // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
  //                                    // acc_d473bw_n499bw
  //  myDLPcfg.g = gyr_d361bw4_n376bw5;    // gyr_d196bw6_n229bw8, gyr_d151bw8_n187bw6, gyr_d119bw5_n154bw3, gyr_d51bw2_n73bw3, gyr_d23bw9_n35bw9, gyr_d11bw6_n17bw8, gyr_d5bw7_n8bw9, gyr_d361bw4_n376bw5
  //  imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  //  // Choose whether or not to use DLPF
  //  ICM_20948_Status_e accDLPEnableStat = imu.enableDLPF((ICM_20948_Internal_Gyr | ICM_20948_Internal_Acc), true);
  //  if (255 != IMU_IRQ_PIN) {
  //    pinMode(IMU_IRQ_PIN, INPUT);
  //    imu.cfgIntActiveLow(true);
  //    imu.cfgIntOpenDrain(false);
  //    imu.cfgIntLatch(true);          // IRQ is a 50us pulse.
  //    imu.intEnableRawDataReady(true);
  //    attachInterrupt(digitalPinToInterrupt(IMU_IRQ_PIN), imu_isr_fxn, FALLING);
  //  }
  if (false) {
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }

  init_step_str = (const char*) "Console         ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);

  disp_update_last = millis();
  disp_update_next = disp_update_last + 1800;
  console.defineCommand("help",        '?', arg_list_1_str, "Prints help to console.", "", 0, callback_help);
  console.defineCommand("history",     arg_list_0, "Print command history.", "", 0, callback_print_history);
  console.defineCommand("reboot",      arg_list_0, "Reboot the controller.", "", 0, callback_reboot);
  console.defineCommand("touchreset",  arg_list_0, "Reset SX8634", "", 0, callback_touch_reset);
  console.defineCommand("touchinfo",   arg_list_0, "SX8634 info", "", 0, callback_touch_info);
  console.defineCommand("touchmode",   arg_list_1_uint, "Get/set SX8634 mode", "", 0, callback_touch_mode);
  console.defineCommand("led",         arg_list_3_uint, "LED Test", "", 1, callback_led_test);
  console.defineCommand("vib",         'v', arg_list_2_uint, "Vibrator test", "", 0, callback_vibrator_test);
  console.defineCommand("disp",        'd', arg_list_1_uint, "Display test", "", 1, callback_display_test);
  console.defineCommand("disprate",    arg_list_1_uint, "Display frame rate", "", 0, callback_display_rate);
  console.defineCommand("aout",        arg_list_4_float, "Mix volumes for the headphones.", "", 4, callback_aout_mix);
  console.defineCommand("fft",         arg_list_4_float, "Mix volumes for the FFT.", "", 4, callback_fft_mix);
  console.defineCommand("synth",       arg_list_4_uuff, "Synth parameters.", "", 2, callback_synth_set);
  console.defineCommand("si",          's', arg_list_1_uint, "Sensor information.", "", 0, callback_sensor_info);
  console.defineCommand("sfi",         arg_list_1_uint, "Sensor filter info.", "", 0, callback_sensor_filter_info);
  console.defineCommand("mfi",         arg_list_1_uint, "Meta filter info.", "", 1, callback_meta_filter_info);
  console.defineCommand("mag",         'm', arg_list_2_uint, "Magnetometer functions.", "", 0, callback_magnetometer_fxns);
  console.defineCommand("sinit",       arg_list_2_uint, "Sensor initialize.", "", 0, callback_sensor_init);
  console.defineCommand("se",          arg_list_2_uint, "Sensor enable.", "", 0, callback_sensor_enable);
  console.defineCommand("sfs",         arg_list_3_uint, "Sensor filter strategy set.", "", 2, callback_sensor_filter_set_strat);
  console.defineCommand("mfs",         arg_list_3_uint, "Meta filter strategy set.", "", 2, callback_meta_filter_set_strat);
  console.defineCommand("app",         'a', arg_list_1_uint, "Select active application.", "", 0, callback_active_app);
  console.defineCommand("sprof",       arg_list_1_uint, "Dump sensor profiler.", "", 0, callback_print_sensor_profiler);
  console.defineCommand("aprof",       arg_list_1_uint, "Dump application profiler.", "", 0, callback_print_app_profiler);
  console.defineCommand("cbor",        arg_list_1_uint, "CBOR test battery.", "", 0, callback_cbor_tests);
  console.defineCommand("vol",         arg_list_1_float, "Audio volume.", "", 0, callback_audio_volume);
  console.setTXTerminator(LineTerm::CRLF);
  console.setRXTerminator(LineTerm::CR);
  console.localEcho(true);
  console.init();

  StringBuilder ptc("Motherflux0r ");
  //ptc.concat(TEST_PROG_VERSION);
  console.printToLog(&ptc);

  init_step_str = (const char*) "Touchpad        ";
  touch = new SX8634(&_touch_opts);
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (0 == touch->init(&Wire)) {
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }

  disp_update_last = millis();
  disp_update_next = disp_update_last + 2000;

  config_time = millis();
  //display.setTextColor(GREEN);
  //display.print((config_time - boot_time), DEC);
  //display.println("ms");

  init_step_str = (const char*) "USB        ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  uint16_t serial_timeout = 0;
  while (!Serial && (100 > serial_timeout)) {
    draw_progress_bar_horizontal(0, 54, 95, 12, RED, true, false, (serial_timeout++ * 0.01));
    delay(70);
  }
  if (Serial) {
    draw_progress_bar_horizontal(0, 54, 95, 12, GREEN, true, false, 1.0);
    while (Serial.available()) {
      Serial.read();
    }
    //display.println("Serial");
  }

  while (disp_update_next > millis()) {}
  if (touch->deviceFound()) {
    touch->poll();
    touch->setMode(SX8634OpMode::ACTIVE);
    touch->setLongpress(800, 0);   // 800ms is a long-press. No rep.
    touch->setButtonFxn(cb_button);
    touch->setSliderFxn(cb_slider);
    touch->setLongpressFxn(cb_longpress);
  }
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, 1.0);
  delay(50);
}



/*******************************************************************************
* Main loop
*******************************************************************************/
void loop() {
  stopwatch_main_loop_time.markStart();
  StringBuilder output;

  if (Serial) {
    const uint8_t RX_BUF_LEN = 32;
    uint8_t ser_buffer[RX_BUF_LEN];
    uint8_t rx_len = 0;
    bool cr_rxd = false;
    memset(ser_buffer, 0, RX_BUF_LEN);
    while ((RX_BUF_LEN > rx_len) && (0 < Serial.available())) {
      ser_buffer[rx_len++] = Serial.read();
    }
    if (rx_len > 0) {
      switch (console.feed(ser_buffer, rx_len)) {
        case -1:   // console buffered the data, but took no other action.
        default:
          ledOn(LED_B_PIN, 5, 500);
          break;
        case 0:   // A full line came in.
          last_interaction  = millis();
          ledOn(LED_R_PIN, 5, 500);
          break;
        case 1:   // A callback was called.
          last_interaction  = millis();
          ledOn(LED_G_PIN, 5, 500);
          break;
      }
    }
    console.fetchLog(&output);
  }

  stopwatch_touch_poll.markStart();
  int8_t t_res = touch->poll();
  if (0 < t_res) {
    // Something changed in the hardware.
    disp_update_rate  = 30;
  }
  stopwatch_touch_poll.markStop();
  if (imu_irq_fired) {
    imu_irq_fired = false;
    //stopwatch_sensor_imu.markStart();
    //read_imu();
    //imu.clearInterrupts();
    stopwatch_sensor_imu.markStop();
  }
  /* Run our async cleanup stuff. */
  uint32_t millis_now = millis();
  if (millis_now >= off_time_led_r) {   pinMode(LED_R_PIN, INPUT);     }
  if (millis_now >= off_time_led_g) {   pinMode(LED_G_PIN, INPUT);     }
  if (millis_now >= off_time_led_b) {   pinMode(LED_B_PIN, INPUT);     }
  if (millis_now >= off_time_vib) {     pinMode(VIBRATOR_PIN, INPUT);  }

  /* Poll each sensor class. */
  stopwatch_sensor_mag.markStart();
  if (2 == magneto.poll()) {       read_magnetometer_sensor();          }
  stopwatch_sensor_mag.markStop();
  stopwatch_sensor_baro.markStart();
  if (0 < baro.poll()) {           read_baro_sensor();                  }
  stopwatch_sensor_baro.markStop();
  stopwatch_sensor_uv.markStart();
  if (0 < uv.poll()) {             read_uv_sensor();                    }
  stopwatch_sensor_uv.markStop();
  stopwatch_sensor_lux.markStart();
  if (0 < tsl2561.poll()) {        read_visible_sensor();               }
  stopwatch_sensor_lux.markStop();
  stopwatch_sensor_grideye.markStart();
  if (0 < grideye.poll()) {        read_thermopile_sensor();            }
  stopwatch_sensor_grideye.markStop();
  stopwatch_sensor_tmp102.markStart();
  if (0 < tmp102.poll()) {         read_battery_temperature_sensor();   }
  stopwatch_sensor_tmp102.markStop();

  if ((last_interaction + 100000) <= millis_now) {
    // After 100 seconds, time-out the display.
    if (AppID::HOT_STANDBY != active_app) {
      active_app = AppID::HOT_STANDBY;
    }
  }

  millis_now = millis();
  if (disp_update_next <= millis_now) {
    // TODO: for some reason this hard-locks the program occasionally if the
    //   framerate is too high at boot. I suspect it has something to do with
    //   touchpad driver.
    stopwatch_display.markStart();
    updateDisplay();
    stopwatch_display.markStop();
    // For tracking framerate, convert from period in micros to hz...
    graph_array_frame_rate.feedFilter(1000000.0 / stopwatch_display.meanTime());
    disp_update_last = millis_now;
    disp_update_next = (0 != disp_update_rate) ? (1000 / disp_update_rate) + disp_update_last : disp_update_last;
  }

  if ((Serial) && (output.length() > 0)) {
    Serial.write(output.string(), output.length());
  }
  stopwatch_main_loop_time.markStop();
  graph_array_cpu_time.feedFilter(stopwatch_main_loop_time.meanTime()/1000.0);
}
