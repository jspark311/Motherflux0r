#include "Motherflux0r.h"
#include "uApp.h"

#include <math.h>
#include <Arduino.h>
#include <StringBuilder.h>
#include <SensorFilter.h>
#include <ParsingConsole.h>
#include <StopWatch.h>
#include <cbor-cpp/cbor.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <SX8634.h>
#include <TinyGPS.h>
#include <TimeLib.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include "VEML6075.h"
#include "ICM20948.h"
#include "BME280.h"
#include "AMG88xx.h"
#include "DRV425.h"
#include "TSL2561.h"
#include "TMP102.h"
#include "VL53L0X.h"

#include <TeensyThreads.h>



/*******************************************************************************
* Globals
*******************************************************************************/
Adafruit_SSD1331 display = Adafruit_SSD1331(&SPI, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);

/* Audio library... */
//AudioInputAnalog         light_adc(ANA_LIGHT_PIN); //xy=224,801.4000358581543
//AudioInputAnalog         mic_adc(MIC_ANA_PIN); //xy=238,757.4000377655029
AudioSynthNoisePink      pinkNoise;      //xy=286.00000381469727,712.4000339508057
AudioSynthWaveformSine   sineL;          //xy=292,640.4000244140625
AudioSynthWaveformSine   sineR;          //xy=292,672.4000339508057
AudioPlayQueue           queueL;         //xy=303,544.4000244140625
AudioPlayQueue           queueR;         //xy=303,580.4000244140625
AudioMixer4              mixerL;         //xy=535,568.4000244140625
AudioMixer4              mixerR;         //xy=536,634.4000244140625
AudioMixer4              mixerFFT;       //xy=544,701.4000244140625
AudioAmplifier           ampR;           //xy=674,629.4000244140625
AudioAmplifier           ampL;           //xy=675,577.4000244140625
AudioAnalyzeFFT256       fft256_1;       //xy=697,701.4000244140625
AudioOutputI2S           i2s_dac;        //xy=821.0000076293945,603.4000329971313
//AudioConnection          patchCord1(light_adc(ANA_LIGHT_PIN), 0, mixerFFT, 3);
//AudioConnection          patchCord2(mic_adc(MIC_ANA_PIN), 0, mixerR, 3);
AudioConnection          patchCord3(pinkNoise, 0, mixerL, 1);
AudioConnection          patchCord4(pinkNoise, 0, mixerR, 1);
AudioConnection          patchCord5(pinkNoise, 0, mixerFFT, 2);
AudioConnection          patchCord6(sineL, 0, mixerL, 2);
AudioConnection          patchCord7(sineR, 0, mixerR, 2);
AudioConnection          patchCord8(queueL, 0, mixerL, 0);
AudioConnection          patchCord9(queueL, 0, mixerFFT, 0);
AudioConnection          patchCord10(queueR, 0, mixerR, 0);
AudioConnection          patchCord11(queueR, 0, mixerFFT, 1);
AudioConnection          patchCord12(mixerL, ampL);
AudioConnection          patchCord13(mixerR, ampR);
AudioConnection          patchCord14(mixerFFT, fft256_1);
AudioConnection          patchCord15(ampR, 0, i2s_dac, 1);
AudioConnection          patchCord16(ampL, 0, i2s_dac, 0);


float volume_left_output  = 0.1;
float volume_right_output = 0.1;
float volume_pink_noise   = 1.0;
float mix_synth_to_fft    = 0.0;
float mix_queueL_to_fft   = 0.0;
float mix_queueR_to_fft   = 0.0;
float mix_noise_to_fft    = 1.0;
float mix_synth_to_line   = 0.0;
float mix_queue_to_line   = 0.0;
float mix_noise_to_line   = 1.0;


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
SX8634* touch = nullptr;


/* Sensor representations... */
DRV425 magneto(DRV425_ADC_IRQ_PIN, DRV425_GPIO_IRQ_PIN, 255, DRV425_CS_PIN);   // No GPIO reset pin.
TMP102 tmp102(0x49, 255);    // No connection to the alert pin.
GridEYE grideye(0x69, AMG8866_IRQ_PIN);
VEML6075 uv;
ICM_20948_SPI imu;
TSL2561 tsl2561(0x49, TSL2561_IRQ_PIN);
BME280I2C baro(baro_settings);
VL53L0X tof;
TinyGPS gps;

/* Immediate data... */
static Vector3f64 grav;       // Gravity vector from the IMU.
static Vector3f64 acc_vect;   // Acceleration vector from the IMU.
static Vector3f64 gyr_vect;   // Gyroscopic vector from the IMU.
static Vector3f64 mag_vect0;  // Magnetism vector from the IMU.
static Vector3f64 mag_vect1;  // Magnetism vector from the DRV425 complex.

/* Data buffers for sensors. */
SensorFilter<float> graph_array_pressure(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_humidity(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_air_temp(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_psu_temp(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_uva(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_uvb(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_uvi(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_ana_light(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_visible(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_therm_mean(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_therm_frame(FilteringStrategy::MOVING_AVG, 64, 0);
SensorFilter<float> graph_array_mag_confidence(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_time_of_flight(FilteringStrategy::RAW, 96, 0);

/* Profiling data */
StopWatch stopwatch_main_loop_time;
StopWatch stopwatch_display;
StopWatch stopwatch_sensor_baro;
StopWatch stopwatch_sensor_uv;
StopWatch stopwatch_sensor_grideye;
StopWatch stopwatch_sensor_imu;
StopWatch stopwatch_sensor_lux;
StopWatch stopwatch_sensor_tmp102;
StopWatch stopwatch_sensor_mag;
StopWatch stopwatch_sensor_gps;
StopWatch stopwatch_sensor_tof;

StopWatch stopwatch_touch_poll;
SensorFilter<float> graph_array_cpu_time(FilteringStrategy::MOVING_MED, 96, 0);
SensorFilter<float> graph_array_frame_rate(FilteringStrategy::RAW, 96, 0);


/* Cheeseball async support stuff. */
uint32_t boot_time         = 0;      // millis() at boot.
uint32_t config_time       = 0;      // millis() at end of setup().
uint32_t last_interaction  = 0;      // millis() when the user last interacted.
uint32_t disp_update_last  = 0;      // millis() when the display last updated.
uint32_t disp_update_next  = 0;      // millis() when the display next updates.
uint32_t disp_update_rate  = 1;      // Update in Hz for the display

uint32_t tof_update_next   = 0;      //



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
uAppTricorder app_tricorder;
uAppMeta app_meta;
uAppTouchTest app_touch_test;
uAppRoot app_root;
uAppSynthBox app_synthbox;
uAppStandby app_standby;
uAppConfigurator app_config;
uAppComms app_comms;

static bool     imu_irq_fired       = false;




/*******************************************************************************
* ISRs
*******************************************************************************/
void imu_isr_fxn() {         imu_irq_fired = true;        }



/*******************************************************************************
* Display functions
*******************************************************************************/
/*
* Called at the frame-rate interval for the display.
*/
void updateDisplay() {
  switch (uApp::appActive()) {
    case AppID::TOUCH_TEST:     app_touch_test.refresh();   break;
    case AppID::META:           app_meta.refresh();         break;
    case AppID::TRICORDER:      app_tricorder.refresh();    break;
    case AppID::APP_SELECT:     app_root.refresh();         break;
    case AppID::SYNTH_BOX:      app_synthbox.refresh();     break;
    case AppID::CONFIGURATOR:   app_config.refresh();       break;
    case AppID::COMMS_TEST:     app_comms.refresh();        break;
    case AppID::SUSPEND:
    case AppID::HOT_STANDBY:    app_standby.refresh();      break;
    default:
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
  if (false) {
    imu.clearInterrupts();
  }
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


int8_t read_time_of_flight_sensor() {
  uint32_t tof_value = tof.readRangeContinuousMillimeters();
  if (!tof.timeoutOccurred()) {
    graph_array_time_of_flight.feedFilter(tof_value);
  }
  return 0;
}


/*******************************************************************************
* GPS
*******************************************************************************/



/*******************************************************************************
* Settings and configuration
*******************************************************************************/

/*
* Read a blob from the
*/
int8_t read_unit_settings() {
  return 0;
}


/*******************************************************************************
* Data aggregation and packaging
*******************************************************************************/

void package_sensor_data_cbor(StringBuilder* cbor_return) {
  cbor::output_dynamic out;
  cbor::encoder encoded(out);
  encoded.write_map(4);
    encoded.write_string("ds_version");
    encoded.write_int(0);

    encoded.write_string("origin");
    encoded.write_map(4);
      encoded.write_string("model");
      encoded.write_string("Motherflux0r");
      encoded.write_string("ser");
      encoded.write_int(1);
      encoded.write_string("fm_ver");
      encoded.write_string(TEST_PROG_VERSION);
      encoded.write_string("ts");
      encoded.write_tag(1);
      encoded.write_int((uint) now());

    encoded.write_string("tilt_cal");
    encoded.write_map(3);
      encoded.write_string("offsetpitch");
      encoded.write_float(4.066);
      encoded.write_string("offsetyaw");
      encoded.write_float(-1.5114);
      encoded.write_string("offsetroll");
      encoded.write_float(-9.701);

    encoded.write_string("meta");
    encoded.write_map(2);
      encoded.write_string("build_data");
      encoded.write_tag(1);
      encoded.write_int(1584023014);
      encoded.write_string("cal_date");
      encoded.write_tag(1);
      encoded.write_int(1584035000);

  cbor_return->concat(out.data(), out.size());
}


/*******************************************************************************
* Touch callbacks
*******************************************************************************/

static void cb_button(int button, bool pressed) {
  last_interaction = millis();
  if (pressed) {
    vibrateOn(19);
  }
  ledOn(LED_B_PIN, 2, 3500);
  uint16_t value = touch->buttonStates();
  last_interaction = millis();
  switch (uApp::appActive()) {
    case AppID::TOUCH_TEST:     app_touch_test.deliverButtonValue(value);    break;
    case AppID::META:           app_meta.deliverButtonValue(value);          break;
    case AppID::TRICORDER:      app_tricorder.deliverButtonValue(value);     break;
    case AppID::APP_SELECT:     app_root.deliverButtonValue(value);          break;
    case AppID::SYNTH_BOX:      app_synthbox.deliverButtonValue(value);      break;
    case AppID::CONFIGURATOR:   app_config.deliverButtonValue(value);        break;
    case AppID::COMMS_TEST:     app_comms.deliverButtonValue(value);         break;
    case AppID::SUSPEND:
    case AppID::HOT_STANDBY:    app_standby.deliverButtonValue(value);       break;
    case AppID::DATA_MGMT:
    case AppID::I2C_SCANNER:
    default:
      break;
  }
}


static void cb_slider(int slider, int value) {
  last_interaction = millis();
  ledOn(LED_R_PIN, 60, 3500);
  switch (uApp::appActive()) {
    case AppID::TOUCH_TEST:     app_touch_test.deliverSliderValue(value);    break;
    case AppID::META:           app_meta.deliverSliderValue(value);          break;
    case AppID::TRICORDER:      app_tricorder.deliverSliderValue(value);     break;
    case AppID::APP_SELECT:     app_root.deliverSliderValue(value);          break;
    case AppID::SYNTH_BOX:      app_synthbox.deliverSliderValue(value);      break;
    case AppID::CONFIGURATOR:   app_config.deliverSliderValue(value);        break;
    case AppID::COMMS_TEST:     app_comms.deliverSliderValue(value);         break;
    case AppID::SUSPEND:
    case AppID::HOT_STANDBY:    app_standby.deliverSliderValue(value);       break;
    case AppID::DATA_MGMT:
    case AppID::I2C_SCANNER:
    default:
      break;
  }
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
    stopwatch_sensor_gps.reset();
    stopwatch_sensor_tof.reset();
    stopwatch_touch_poll.reset();
  }
  StopWatch::printDebugHeader(text_return);
  stopwatch_sensor_baro.printDebug("Baro", text_return);
  stopwatch_sensor_uv.printDebug("UV", text_return);
  stopwatch_sensor_grideye.printDebug("GridEye", text_return);
  stopwatch_sensor_imu.printDebug("IMU", text_return);
  stopwatch_sensor_lux.printDebug("TSL2561", text_return);
  //stopwatch_sensor_tmp102.printDebug("PSU Temp", text_return);
  stopwatch_sensor_tof.printDebug("ToF", text_return);
  stopwatch_sensor_mag.printDebug("Magnetometer", text_return);
  stopwatch_sensor_gps.printDebug("GPS", text_return);
  stopwatch_touch_poll.printDebug("Touch", text_return);
  return 0;
}


int callback_print_app_profiler(StringBuilder* text_return, StringBuilder* args) {
  if (args->count() > 0) {
    app_meta.resetStopwatch();
    app_touch_test.resetStopwatch();
    app_tricorder.resetStopwatch();
    app_root.resetStopwatch();
    app_synthbox.resetStopwatch();
    app_standby.resetStopwatch();
    app_config.resetStopwatch();
    app_comms.resetStopwatch();
    stopwatch_main_loop_time.reset();
    stopwatch_display.reset();
  }
  StopWatch::printDebugHeader(text_return);
  app_meta.printStopwatch(text_return);
  app_touch_test.printStopwatch(text_return);
  app_tricorder.printStopwatch(text_return);
  app_root.printStopwatch(text_return);
  app_synthbox.printStopwatch(text_return);
  app_standby.printStopwatch(text_return);
  app_config.printStopwatch(text_return);
  app_comms.printStopwatch(text_return);
  stopwatch_main_loop_time.printDebug("Main loop", text_return);
  stopwatch_display.printDebug("Display", text_return);
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
    case 1:    uApp::redraw_app_window("Test App Title", 0, 0);   break;
    case 2:    app_touch_test.refresh();                    break;
    case 3:    app_tricorder.refresh();                     break;
    case 4:    app_synthbox.refresh();                      break;
    case 5:      break;
    case 6:      break;
    case 7:      break;
    case 8:
      display.fillScreen(BLACK);
      draw_progress_bar_vertical(0, 0, 12, 63, CYAN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(0, 0, 12, 63, CYAN, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_vertical(14, 0, 7, 63, BLUE, true, false, 1.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(14, 0, 7, 63, BLUE, false, false, 1.0 - (i * 0.01));
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

      draw_progress_bar_vertical(32, 0, 30, 63, GREEN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(32, 0, 30, 63, GREEN, false, true, (i * 0.01));
        delay(40);
      }
      break;

    case 9:    // Progress bar test
      display.fillScreen(BLACK);
      draw_progress_bar_horizontal(0, 14, 95, 7, CYAN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 14, 95, 7, CYAN, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar_horizontal(0, 0, 95, 12, BLUE, true, false, 1.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 0, 95, 12, BLUE, false, false, 1.0-(i * 0.01));
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
      case 0:   uApp::setAppActive(AppID::APP_SELECT);    break;
      case 1:   uApp::setAppActive(AppID::TOUCH_TEST);    break;
      case 2:   uApp::setAppActive(AppID::CONFIGURATOR);  break;
      case 3:   uApp::setAppActive(AppID::DATA_MGMT);     break;
      case 4:   uApp::setAppActive(AppID::SYNTH_BOX);     break;
      case 5:   uApp::setAppActive(AppID::COMMS_TEST);    break;
      case 6:   uApp::setAppActive(AppID::META);          break;
      case 7:   uApp::setAppActive(AppID::I2C_SCANNER);   break;
      case 8:   uApp::setAppActive(AppID::TRICORDER);     break;
      case 9:   uApp::setAppActive(AppID::HOT_STANDBY);   break;
      case 10:  uApp::setAppActive(AppID::SUSPEND);       break;
      default:
        text_return->concatf("Unsupported app: %d\n", arg0);
        return -1;
    }
  }
  else {   // No arguments means print the app index list.
    uApp::listAllApplications(text_return);
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
      //case SensorID::TOF:            tmp102.printDebug(text_return);        break;
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
      case SensorID::TOF:
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
    //case SensorID::PSU_TEMP:      ret = graph_array_psu_temp.setStrategy((FilteringStrategy) arg1);    break;
    case SensorID::TOF:           ret = graph_array_time_of_flight.setStrategy((FilteringStrategy) arg1);    break;
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
      //case SensorID::PSU_TEMP:       ret = tmp102.init(&Wire1);          break;
      case SensorID::TOF:            ret = tof.init(&Wire1);             break;
      //case SensorID::BATT_VOLTAGE:       break;
      case SensorID::IMU:            ret = read_imu();               break;
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
      //case SensorID::PSU_TEMP:      break;
      case SensorID::TOF:           break;
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
    volume_left_output  = arg0;
    volume_right_output = arg0;
    ampL.gain(volume_left_output);
    ampR.gain(volume_right_output);
    text_return->concatf("Audio volume: %.2f\n", arg0);
  }
  return 0;
}


int callback_cbor_tests(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (1 == args->count()) {
    int arg0 = args->position_as_int(0);
  }

  int eeprom_len = EEPROM.length();
  text_return->concatf("EEPROM len:  %d\n", eeprom_len);
  StringBuilder cbor_return;
  package_sensor_data_cbor(&cbor_return);
  text_return->concatf("CBOR test results: %d\n", ret);
  StringBuilder::printBuffer(text_return, cbor_return.string(), cbor_return.length(), "\t");
  return ret;
}




/*******************************************************************************
* Setup function
*******************************************************************************/
void i2c_thread();


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
  pinMode(TOF_IRQ_PIN, INPUT);
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

  // GPS
  Serial1.setRX(GPS_TX_PIN);
  Serial1.setTX(GPS_RX_PIN);
  Serial1.begin(9600);

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
  graph_array_time_of_flight.init();

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

  init_step_str = (const char*) "ToF         ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  tof.setTimeout(500);
  if (tof.init(&Wire1)) {
    tof.startContinuous(100);
  }
  else {
    display.setTextColor(RED, BLACK);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }

  //init_step_str = (const char*) "PSU Temperature";
  //percent_setup += 0.08;
  //draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  //display.setTextColor(WHITE);
  //display.setCursor(4, 14);
  //display.print(init_step_str);
  //if (0 == tmp102.init(&Wire)) {
  //}
  //else {
  //  display.setTextColor(RED, BLACK);
  //  display.setCursor(0, cursor_height);
  //  display.print(init_step_str);
  //  cursor_height += 8;
  //}

  init_step_str = (const char*) "Inertial        ";
  percent_setup += 0.08;
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (ICM_20948_Stat_Ok == imu.begin(IMU_CS_PIN, SPI, 6000000)) {
    imu.swReset();
    imu.sleep(false);
    imu.lowPower(false);
    imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mag), ICM_20948_Sample_Mode_Continuous);
    ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
    myFSS.a = gpm2;         // gpm2, gpm4, gpm8, gpm16
    myFSS.g = dps250;       // dps250, dps500, dps1000, dps2000
    imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;      // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw;   // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                      // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                      // acc_d111bw4_n136bw
                                      // acc_d50bw4_n68bw8
                                      // acc_d23bw9_n34bw4
                                      // acc_d11bw5_n17bw
                                      // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                      // acc_d473bw_n499bw
    myDLPcfg.g = gyr_d361bw4_n376bw5;    // gyr_d196bw6_n229bw8, gyr_d151bw8_n187bw6, gyr_d119bw5_n154bw3, gyr_d51bw2_n73bw3, gyr_d23bw9_n35bw9, gyr_d11bw6_n17bw8, gyr_d5bw7_n8bw9, gyr_d361bw4_n376bw5
    imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    // Choose whether or not to use DLPF
    ICM_20948_Status_e accDLPEnableStat = imu.enableDLPF((ICM_20948_Internal_Gyr | ICM_20948_Internal_Acc), true);
    if (255 != IMU_IRQ_PIN) {
      pinMode(IMU_IRQ_PIN, INPUT);
      //imu.cfgIntActiveLow(true);
      //imu.cfgIntOpenDrain(false);
      //imu.cfgIntLatch(true);          // IRQ is a 50us pulse.
      //imu.intEnableRawDataReady(true);
      attachInterrupt(digitalPinToInterrupt(IMU_IRQ_PIN), imu_isr_fxn, FALLING);
    }
  //if (false) {
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
    draw_progress_bar_horizontal(0, 54, 95, 9, RED, (0 == serial_timeout), false, (serial_timeout++ * 0.01));
    delay(70);
  }
  if (Serial) {
    draw_progress_bar_horizontal(0, 54, 95, 9, GREEN, false, false, 1.0);
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
  //threads.addThread(i2c_thread);
}



/*******************************************************************************
* Main loop
*******************************************************************************/

void i2c_thread() {
}


void loop() {
  stopwatch_main_loop_time.markStart();
  StringBuilder output;

  if (Serial) {
    const uint8_t RX_BUF_LEN = 32;
    uint8_t ser_buffer[RX_BUF_LEN];
    uint8_t rx_len = 0;
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
    stopwatch_sensor_imu.markStart();
    read_imu();
    stopwatch_sensor_imu.markStop();
  }
  /* Run our async cleanup stuff. */
  uint32_t millis_now = millis();
  timeoutCheckVibLED();

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
  //stopwatch_sensor_tmp102.markStart();
  //if (0 < tmp102.poll()) {         read_battery_temperature_sensor();   }
  //stopwatch_sensor_tmp102.markStop();

  if (tof_update_next <= millis_now) {
    stopwatch_sensor_tof.markStart();
    read_time_of_flight_sensor();
    stopwatch_sensor_tof.markStop();
    tof_update_next = now + 100;
  }


  stopwatch_sensor_gps.markStart();
  while (SerialGPS.available()) {
    if (gps.encode(Serial1.read())) { // process gps messages
      unsigned long age;
      int Year;
      byte Month, Day, Hour, Minute, Second;
      gps.crack_datetime(&Year, &Month, &Day, &Hour, &Minute, &Second, NULL, &age);
      if (age < 500) {
        // set the Time to the latest GPS reading
        setTime(Hour, Minute, Second, Day, Month, Year);
        //adjustTime(offset * SECS_PER_HOUR);
      }
    }
  }
  if (timeStatus()!= timeNotSet) {
  }
  stopwatch_sensor_gps.markStop();



  if ((last_interaction + 100000) <= millis_now) {
    // After 100 seconds, time-out the display.
    if (AppID::HOT_STANDBY != uApp::appActive()) {
      uApp::setAppActive(AppID::HOT_STANDBY);
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
