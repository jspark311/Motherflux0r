#include "Motherflux0r.h"
#include "uApp.h"

#include <math.h>
#include <Arduino.h>
#include <StringBuilder.h>
#include <SensorFilter.h>
#include <ParsingConsole.h>
#include <TripleAxisPipe.h>
#include <GPSWrapper.h>
#include <StopWatch.h>
#include <Image/Image.h>
#include <cbor-cpp/cbor.h>
#include <uuid.h>

#include <SPIAdapter.h>
#include <I2CAdapter.h>

#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include <EEPROM.h>
#include <TimeLib.h>

#include <ManuvrDrivers.h>
#include "ICM20948.h"
#include "DRV425.h"

#include "CommPeer.h"

//#include <TinyGPS.h>


/* Forward declarations for 3-axis callbacks. */
int8_t callback_3axis(SpatialSense, Vector3f* dat, Vector3f* err, uint32_t seq_num);


/*******************************************************************************
* Globals
*******************************************************************************/

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


/*******************************************************************************
* 3-Axis pipelines
*******************************************************************************/

// Magnetic pipeline
TripleAxisTerminus     mag_vect(SpatialSense::MAG, callback_3axis);   // The magnetic field vector.
TripleAxisCompass      compass(callback_3axis);
TripleAxisFork         mag_fork(&compass, &mag_vect);
TripleAxisSingleFilter mag_filter(SpatialSense::MAG, &mag_fork, FilteringStrategy::MOVING_AVG, 2, 0);
TripleAxisConvention   mag_conv(&mag_filter, GnomonType::RH_POS_Z);   // TODO: Determine true value.

// Inertial pipeline
TripleAxisTerminus     down(SpatialSense::ACC, callback_3axis);  // The tilt sensor's best-estimate of "down".
TripleAxisFork         imu_fork(&compass, &down);
TripleAxisConvention   tilt_conv(&imu_fork, GnomonType::RH_POS_Z);   // TODO: Determine true value.


/*******************************************************************************
* Buses
*******************************************************************************/
const I2CAdapterOptions i2c0_opts(
  0,   // Device number
  SDA0_PIN,
  SCL0_PIN,
  0,   // No pullups.
  400000
);

const I2CAdapterOptions i2c1_opts(
  1,   // Device number
  SDA1_PIN,
  SCL1_PIN,
  0,   // No pullups.
  400000
);

I2CAdapter i2c0(&i2c0_opts);
I2CAdapter i2c1(&i2c1_opts);


/*******************************************************************************
* Display
*******************************************************************************/
/* Configuration for the back panel display. */
const SSD13xxOpts disp_opts(
  ImgOrientation::ROTATION_0,
  DISPLAY_RST_PIN,
  DISPLAY_DC_PIN,
  DISPLAY_CS_PIN,
  SSDModel::SSD1331
);

/* Driver classes */
SPIAdapter spi0(0, SPISCK_PIN, SPIMOSI_PIN, SPIMISO_PIN, 8);
SSD13xx display(&disp_opts);


/*******************************************************************************
* Sensors
*******************************************************************************/
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
GridEYE grideye(0x69, AMG8866_IRQ_PIN);
VEML6075 uv;
ICM_20948_SPI imu;
TSL2561 tsl2561(0x49, TSL2561_IRQ_PIN);
BME280I2C baro(baro_settings);
VL53L0X tof;
//TinyGPS gps;
GPSWrapper gps;

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
uint32_t tof_update_next   = 0;      //

/* Console junk... */
ParsingConsole console(128);
static const TCode arg_list_4_uuff[]  = {TCode::UINT32, TCode::UINT32, TCode::FLOAT, TCode::FLOAT, TCode::NONE};
static const TCode arg_list_4_float[] = {TCode::FLOAT,  TCode::FLOAT,  TCode::FLOAT, TCode::FLOAT, TCode::NONE};

/* Application tracking and interrupts... */
extern uAppBoot app_boot;
extern uAppTricorder app_tricorder;
extern uAppMeta app_meta;
extern uAppTouchTest app_touch_test;
extern uAppRoot app_root;
extern uAppSynthBox app_synthbox;
extern uAppStandby app_standby;
extern uAppConfigurator app_config;
extern uAppComms app_comms;
extern uAppDataMgmt app_data_mgmt;

/* First stab at WakeLock... */
WakeLock* wakelock_tof         = nullptr;
WakeLock* wakelock_mag         = nullptr;
WakeLock* wakelock_lux         = nullptr;
WakeLock* wakelock_imu         = nullptr;
WakeLock* wakelock_grideye     = nullptr;
WakeLock* wakelock_uv          = nullptr;
WakeLock* wakelock_baro        = nullptr;
WakeLock* wakelock_gps         = nullptr;


static bool     imu_irq_fired       = false;



/*******************************************************************************
* ISRs
*******************************************************************************/
void imu_isr_fxn() {         imu_irq_fired = true;        }



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
  int8_t ret = -1;
  int8_t poll_ret = magneto.poll();
  switch (poll_ret) {
    case -5: // if not initialized and enabled.
    case -4: // if not calibrated.
    case -3: // if the ADC needed to be read, but doing so failed.
    case -2: // if the GPIO needed to be read, but doing so failed.
    case -1: // if we experienced a fault signal from the sensor.
      break;
    case 0:  // if nothing needs doing.
    case 1:  // if new ADC data is ready.
    case 2:  // if new compass data is ready.
      stopwatch_sensor_mag.markStop();
      ret = 0;
      break;
  }
  return ret;
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
/*
* All packets
*/
void pack_cbor_comm_packet(cbor::encoder* pkt, CommPeer* peer) {
  const uint8_t PROTO_VER = 0;
  if (nullptr == peer) {
    pkt->write_map(3);
  }
  else {
    pkt->write_map(4);
    peer->serializeCBOR(pkt, PROTO_VER);
  }
  pkt->write_string("comver");
  pkt->write_int(PROTO_VER);   // The protocol version

    pkt->write_string("orig");
    pkt->write_map(4);
      pkt->write_string("mod");
      pkt->write_string("Motherflux0r");
      pkt->write_string("ser");
      pkt->write_int(1);
      pkt->write_string("firm_ver");
      pkt->write_string(TEST_PROG_VERSION);
      pkt->write_string("ts");
      pkt->write_tag(1);
      pkt->write_int((uint) now());
    pkt->write_string("pdu");
}


void package_sensor_data_cbor(StringBuilder* cbor_return) {
  cbor::output_dynamic out;
  cbor::encoder encoded(out);
  encoded.write_map(2);
    encoded.write_string("ds_ver");
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
* 3-axis callbacks
*******************************************************************************/
/*
* Pipeline sinks that have a callback configured will call this function on
*   value updates. If this function returns zero, the terminating class will not
*   be marked "fresh", since we have presumably already noted the value.
*/
int8_t callback_3axis(SpatialSense s, Vector3f* dat, Vector3f* err, uint32_t seq_num) {
  StringBuilder output;
  int8_t ret = 0;
  switch (s) {
    case SpatialSense::BEARING:
      // TODO: Move calculation into Compass class.
      // TODO: Should be a confidence value.
      graph_array_mag_confidence.feedFilter(compass.getError()->length());
      ret = -1;
      break;
    case SpatialSense::MAG:
      ret = -1;
      break;
    case SpatialSense::UNITLESS:
    case SpatialSense::ACC:
    case SpatialSense::GYR:
    case SpatialSense::EULER_ANG:
    default:
      //output.concatf("Unhandled SpatialSense in callback: %s\n", TripleAxisPipe::spatialSenseStr(s));
      break;
  }
  return ret;
}


/*******************************************************************************
* Touch callbacks
*******************************************************************************/

static void cb_button(int button, bool pressed) {
  last_interaction = millis();
  if (pressed) {
    vibrateOn(19);
  }
  ledOn(LED_B_PIN, 2, (4095 - graph_array_ana_light.value() * 3000));
  uint16_t value = touch->buttonStates();
  last_interaction = millis();
  uApp::appActive()->deliverButtonValue(value);
}


static void cb_slider(int slider, int value) {
  last_interaction = millis();
  ledOn(LED_R_PIN, 30, (4095 - graph_array_ana_light.value() * 3000));
  uApp::appActive()->deliverSliderValue(value);
}


static void cb_longpress(int button, uint32_t duration) {
  ledOn(LED_G_PIN, 30, (4095 - graph_array_ana_light.value() * 3000));
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


int callback_display_test(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  uint32_t millis_0 = millis();
  uint32_t millis_1 = millis_0;
  switch (arg0) {
    case 0:    display.fill(WHITE);     break;
    case 1:    text_return->concatf("display.reset() returns %d\n", display.reset());        break;
    case 2:    text_return->concatf("display.init() returns %d\n", display.init());          break;
    case 3:    text_return->concatf("commitFrameBuffer() returns %d\n", display.commitFrameBuffer());   break;
    case 4:    break;
    case 5:    break;
    case 6:    break;
    case 7:    break;
    case 8:
      display.fill(BLACK);
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
      display.fill(BLACK);
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
      display.fill(BLACK);
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
      case 7:   uApp::setAppActive(AppID::TRICORDER);     break;
      case 8:   uApp::setAppActive(AppID::HOT_STANDBY);   break;
      case 9:   uApp::setAppActive(AppID::SUSPEND);       break;
      default:
        text_return->concatf("Unsupported app: %d\n", arg0);
        return -1;
    }
  }
  else {
    uApp::appActive()->refresh();
  }
  return 0;
}


int callback_sensor_info(StringBuilder* text_return, StringBuilder* args) {
  if (0 < args->count()) {
    int arg0 = args->position_as_int(0);
    switch ((SensorID) arg0) {
      case SensorID::MAGNETOMETER:   magneto.printDebug(text_return);  break;
      case SensorID::BARO:           baro.printDebug(text_return);     break;
      case SensorID::LIGHT:          i2c1.printDebug(text_return);     break;
      case SensorID::UV:             uv.printDebug(text_return);       break;
      case SensorID::THERMOPILE:     grideye.printDebug(text_return);  break;
      case SensorID::LUX:            tsl2561.printDebug(text_return);  break;
      case SensorID::BATT_VOLTAGE:   grideye.poll();               break;
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
        mag_filter.printFilter(text_return);
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
    case SensorID::MAGNETOMETER:  ret = mag_filter.setStrategy((FilteringStrategy) arg1);   break;
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
    int arg0 = args->position_as_int(0);
    ret = 0;
    switch (arg0) {
      case 0:
        switch (args->position_as_int(1)) {
          default:
          case 0:   magneto.printDebug(text_return);           break;
          case 1:   magneto.printPipeline(text_return);        break;
          case 2:   compass.printPipe(text_return, 0, 7);      break;
          case 3:   compass.printBearing(HeadingType::MAGNETIC_NORTH, text_return);  break;
          case 4:   magneto.printChannelValues(text_return);   break;
          case 5:   magneto.printRegs(text_return);            break;
          case 6:   magneto.printPins(text_return);            break;
          case 7:   magneto.printData(text_return);            break;
          case 8:   magneto.printTimings(text_return);         break;
          case 9:
            text_return->concatf("ADC Temperature: %u.\n", (uint8_t) magneto.getTemperature());
            break;
          case 10:  magneto.printChannelValues(text_return);   break;
        }
        break;

      case 1:
        stopwatch_sensor_mag.markStart();
        ret = magneto.poll();
        stopwatch_sensor_mag.markStop();
        text_return->concatf("Polling the magnetometer returns %d.\n", ret);
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
          mag_conv.setAfferentGnomon((GnomonType) args->position_as_int(1));
        }
        mag_conv.printPipe(text_return, 0, 0);
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
      case 8:
        if (1 < args->count()) {
          uint32_t arg1 = args->position_as_int(1);
          if (0 != arg1) {
            ret = mag_filter.setParam0(strict_min(arg1, 512));
          }
        }
        text_return->concatf("Magnetometer filter size is now %u.\n", mag_filter.getParam0());
        break;
      case 9:
        if (1 < args->count()) {
          if (0 != args->position_as_int(1)) {
            magneto.getWakeLock()->acquire();
          }
          else {
            magneto.getWakeLock()->release();
          }
        }
        text_return->concatf("Mag WAKELOCK held: %c.\n", magneto.getWakeLock()->isHeld() ? 'y':'n');
        break;
      case 10:
        text_return->concatf("Magnetometer init() returns %d\n", magneto.init(&i2c1, &spi0));
        break;

      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
      case 16:
      case 17:
      case 18:
        {
          uint8_t eidx = ((uint8_t) arg0 - 11);
          DRV425State state = (DRV425State) eidx;
          text_return->concatf("magneto.setDesiredState(%s) returns %d\n", DRV425::drvStateStr(state), magneto.setDesiredState(state));
        }
        break;
      case 19:
        text_return->concatf("Magnetometer calibrate() returns %d\n", magneto.calibrate());
        break;
      default:
        ret = -3;
        break;
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
    text_return->concat("8:  Filter depth\n");
    text_return->concat("9:  WakeLock\n");
    text_return->concat("10: Init\n");
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
      case SensorID::MAGNETOMETER:   ret = magneto.init(&i2c1, &spi0);   break;
      case SensorID::BARO:           ret = baro.init();                  break;
      case SensorID::LUX:            ret = tsl2561.init();               break;
      case SensorID::UV:             ret = uv.init();                    break;
      case SensorID::THERMOPILE:     ret = grideye.init(&i2c1);          break;
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
      case SensorID::BATT_VOLTAGE:  break;
      case SensorID::IMU:           break;
      case SensorID::MIC:           break;
      case SensorID::GPS:           break;
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
  int ret = 0;
  int eeprom_len = EEPROM.length();
  text_return->concatf("EEPROM len:  %d\n", eeprom_len);
  StringBuilder cbor_return;
  package_sensor_data_cbor(&cbor_return);
  text_return->concatf("CBOR test results: %d\n", ret);
  StringBuilder::printBuffer(text_return, cbor_return.string(), cbor_return.length(), "\t");
  text_return->concatf("CBOR test results: %d\n", ret);
  UUID testuuid;
  uuid_gen(&testuuid);
  uuid_to_sb(&testuuid, text_return);
  return ret;
}




/*******************************************************************************
* Setup function
*******************************************************************************/
void setup() {
  boot_time = millis();
  Serial.begin(115200);   // USB

  pinMode(IMU_IRQ_PIN,    GPIOMode::INPUT_PULLUP);
  pinMode(DRV425_CS_PIN,  GPIOMode::INPUT); // Wrong
  pinMode(PSU_SX_IRQ_PIN, GPIOMode::INPUT);
  pinMode(ANA_LIGHT_PIN,  GPIOMode::INPUT);
  pinMode(TOF_IRQ_PIN,    GPIOMode::INPUT);
  pinMode(LED_R_PIN,      GPIOMode::INPUT);
  pinMode(LED_G_PIN,      GPIOMode::INPUT);
  pinMode(LED_B_PIN,      GPIOMode::INPUT);

  AudioMemory(32);
  analogWriteResolution(12);

  uint16_t serial_timeout = 0;
  while (!Serial && (100 > serial_timeout)) {
    serial_timeout++;
    delay(70);
  }

  spi0.init();
  i2c0.init();
  i2c1.init();

  // GPS
  Serial1.setRX(GPS_TX_PIN);
  Serial1.setTX(GPS_RX_PIN);
  Serial1.begin(9600);

  Serial6.setRX(COMM_TX_PIN);
  Serial6.setTX(COMM_RX_PIN);
  Serial6.begin(115200);    // Comm

  //SD.begin(BUILTIN_SDCARD);

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

  console.defineCommand("help",        '?', ParsingConsole::tcodes_str_1, "Prints help to console.", "", 0, callback_help);
  console.defineCommand("history",     ParsingConsole::tcodes_0, "Print command history.", "", 0, callback_print_history);
  console.defineCommand("reboot",      ParsingConsole::tcodes_0, "Reboot the controller.", "", 0, callback_reboot);
  console.defineCommand("touchreset",  ParsingConsole::tcodes_0, "Reset SX8634", "", 0, callback_touch_reset);
  console.defineCommand("touchinfo",   ParsingConsole::tcodes_0, "SX8634 info", "", 0, callback_touch_info);
  console.defineCommand("touchmode",   ParsingConsole::tcodes_uint_1, "Get/set SX8634 mode", "", 0, callback_touch_mode);
  console.defineCommand("led",         ParsingConsole::tcodes_uint_3, "LED Test", "", 1, callback_led_test);
  console.defineCommand("vib",         'v', ParsingConsole::tcodes_uint_2, "Vibrator test", "", 0, callback_vibrator_test);
  console.defineCommand("disp",        'd', ParsingConsole::tcodes_uint_1, "Display test", "", 1, callback_display_test);
  console.defineCommand("aout",        arg_list_4_float, "Mix volumes for the headphones.", "", 4, callback_aout_mix);
  console.defineCommand("fft",         arg_list_4_float, "Mix volumes for the FFT.", "", 4, callback_fft_mix);
  console.defineCommand("synth",       arg_list_4_uuff, "Synth parameters.", "", 2, callback_synth_set);
  console.defineCommand("si",          's', ParsingConsole::tcodes_uint_1, "Sensor information.", "", 0, callback_sensor_info);
  console.defineCommand("sfi",         ParsingConsole::tcodes_uint_1, "Sensor filter info.", "", 0, callback_sensor_filter_info);
  console.defineCommand("mfi",         ParsingConsole::tcodes_uint_1, "Meta filter info.", "", 1, callback_meta_filter_info);
  console.defineCommand("mag",         'm', ParsingConsole::tcodes_uint_2, "Magnetometer functions.", "", 0, callback_magnetometer_fxns);
  console.defineCommand("sinit",       ParsingConsole::tcodes_uint_2, "Sensor initialize.", "", 0, callback_sensor_init);
  console.defineCommand("se",          ParsingConsole::tcodes_uint_2, "Sensor enable.", "", 0, callback_sensor_enable);
  console.defineCommand("sfs",         ParsingConsole::tcodes_uint_3, "Sensor filter strategy set.", "", 2, callback_sensor_filter_set_strat);
  console.defineCommand("mfs",         ParsingConsole::tcodes_uint_3, "Meta filter strategy set.", "", 2, callback_meta_filter_set_strat);
  console.defineCommand("app",         'a', ParsingConsole::tcodes_uint_1, "Select active application.", "", 0, callback_active_app);
  console.defineCommand("sprof",       ParsingConsole::tcodes_uint_1, "Dump sensor profiler.", "", 0, callback_print_sensor_profiler);
  console.defineCommand("aprof",       ParsingConsole::tcodes_uint_1, "Dump application profiler.", "", 0, callback_print_app_profiler);
  console.defineCommand("cbor",        ParsingConsole::tcodes_uint_1, "CBOR test battery.", "", 0, callback_cbor_tests);
  console.defineCommand("vol",         ParsingConsole::tcodes_float_1, "Audio volume.", "", 0, callback_audio_volume);
  console.setTXTerminator(LineTerm::CRLF);
  console.setRXTerminator(LineTerm::CR);
  console.localEcho(true);
  console.printHelpOnFail(true);
  console.init();

  StringBuilder ptc("Motherflux0r ");
  ptc.concat(TEST_PROG_VERSION);
  ptc.concat("\t Build date " __DATE__ " " __TIME__ "\n");
  console.printToLog(&ptc);

  touch = new SX8634(&_touch_opts);
  touch->assignBusInstance(&i2c0);
  touch->setButtonFxn(cb_button);
  touch->setSliderFxn(cb_slider);
  touch->setLongpressFxn(cb_longpress);

  config_time = millis();
  //display.setTextColor(GREEN);
  //display.writeString((config_time - boot_time), DEC);
  //display.writeString("ms\n");

  wakelock_tof     = nullptr;
  wakelock_mag     = magneto.getWakeLock();
  wakelock_lux     = nullptr;
  wakelock_imu     = nullptr;
  wakelock_grideye = nullptr;
  wakelock_uv      = nullptr;
  wakelock_baro    = nullptr;
  wakelock_gps     = nullptr;

  wakelock_mag->referenceCounted(false);
}



/*******************************************************************************
* Main loop
*******************************************************************************/

void spi_spin() {
  int8_t polling_ret = spi0.poll();
  while (0 < polling_ret) {
    polling_ret = spi0.poll();
    //Serial.println("spi_spin() 0");
  }
  polling_ret = spi0.service_callback_queue();
  while (0 < polling_ret) {
    polling_ret = spi0.service_callback_queue();
    //Serial.println("spi_spin() 1");
  }
}

void i2c0_spin() {
  int8_t polling_ret = i2c0.poll();
  while (0 < polling_ret) {
    polling_ret = i2c0.poll();
    //Serial.print("i2c0_spin() ");
    //Serial.println(polling_ret);
  }
}

void i2c1_spin() {
  int8_t polling_ret = i2c1.poll();
  while (0 < polling_ret) {
    polling_ret = i2c1.poll();
    //Serial.print("i2c1_spin() ");
    //Serial.println(polling_ret);
  }
}


void loop() {
  stopwatch_main_loop_time.markStart();
  StringBuilder output;

  if (Serial) {
    const uint8_t RX_BUF_LEN = 32;
    StringBuilder console_input;
    uint8_t ser_buffer[RX_BUF_LEN];
    uint8_t rx_len = 0;
    memset(ser_buffer, 0, RX_BUF_LEN);
    while ((RX_BUF_LEN > rx_len) && (0 < Serial.available())) {
      ser_buffer[rx_len++] = Serial.read();
    }
    if (rx_len > 0) {
      last_interaction = millis();
      console_input.concat(ser_buffer, rx_len);
      console.provideBuffer(&console_input);
      ledOn(LED_B_PIN, 5, 500);
    }
    console.fetchLog(&output);
  }

  spi_spin();
  i2c0_spin();
  i2c1_spin();

  stopwatch_touch_poll.markStart();
  int8_t t_res = touch->poll();
  if (0 < t_res) {
    // Something changed in the hardware.
  }
  stopwatch_touch_poll.markStop();

  //if (imu_irq_fired) {
  //  imu_irq_fired = false;
  //  stopwatch_sensor_imu.markStart();
  //  read_imu();
  //  stopwatch_sensor_imu.markStop();
  //}

  /* Run our async cleanup stuff. */
  uint32_t millis_now = millis();
  timeoutCheckVibLED();

  // /* Poll each sensor class. */
  if (magneto.power()) {
     stopwatch_sensor_mag.markStart();
     magneto.poll();
     stopwatch_sensor_mag.markStop();
  }
  stopwatch_sensor_baro.markStart();
  if (0 < baro.poll()) {           read_baro_sensor();                  }
  stopwatch_sensor_baro.markStop();

  stopwatch_sensor_uv.markStart();
  if (0 < uv.poll()) {             read_uv_sensor();                    }
  stopwatch_sensor_uv.markStop();

  stopwatch_sensor_lux.markStart();
  if (0 < tsl2561.poll()) {
    read_visible_sensor();
    stopwatch_sensor_lux.markStop();
  }

  if (grideye.enabled()) {
    stopwatch_sensor_grideye.markStart();
    grideye.poll();
    if (0 < grideye.frameReady()) {      read_thermopile_sensor();           }
    stopwatch_sensor_grideye.markStop();
  }

  //if (tof_update_next <= millis_now) {
  //  stopwatch_sensor_tof.markStart();
  //  read_time_of_flight_sensor();
  //  stopwatch_sensor_tof.markStop();
  //  tof_update_next = millis_now + 100;
  //}

  // stopwatch_sensor_gps.markStart();
  // const int GPS_LEN = 160;
  // if (SerialGPS.available()) {
  //   uint8_t gps_buf[GPS_LEN];
  //   int gps_bytes_read = 0;
  //   while (SerialGPS.available() && (gps_bytes_read < GPS_LEN)) {
  //     gps_buf[gps_bytes_read++] = SerialGPS.read();
  //   }
  //   gps.feed(gps_buf, gps_bytes_read);
  //   stopwatch_sensor_gps.markStop();
  // }

  if ((last_interaction + 100000) <= millis_now) {
    // After 100 seconds, time-out the display.
    if (&app_standby != uApp::appActive()) {
      uApp::setAppActive(AppID::HOT_STANDBY);
    }
  }

  millis_now = millis();
  stopwatch_display.markStart();
  uApp::appActive()->refresh();
  stopwatch_display.markStop();
  // For tracking framerate, convert from period in micros to hz...
  graph_array_frame_rate.feedFilter(1000000.0 / 1+stopwatch_display.meanTime());
  //delay(80);

  if ((Serial) && (output.length() > 0)) {
    Serial.write(output.string(), output.length());
  }
  stopwatch_main_loop_time.markStop();
  graph_array_cpu_time.feedFilter(stopwatch_main_loop_time.meanTime()/1000.0);
}
