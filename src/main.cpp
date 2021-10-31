#include "Motherflux0r.h"
#include "uApp.h"

#include <math.h>
#include <StringBuilder.h>
#include <SensorFilter.h>
#include <ParsingConsole.h>
#include <TripleAxisPipe.h>
#include <GPSWrapper.h>
#include <StopWatch.h>
#include <Image/Image.h>
#include <ManuvrLink/ManuvrLink.h>
#include <cbor-cpp/cbor.h>
#include <uuid.h>

#include <SPIAdapter.h>
#include <I2CAdapter.h>
#include <UARTAdapter.h>

#include <Audio.h>
#include <SD.h>
#include <EEPROM.h>
#include <TimeLib.h>

#include <ManuvrDrivers.h>
#include <ManuvrArduino.h>
#include <Composites/ManuvrPMU/ManuvrPMU-r2.h>

#include "SensorGlue.h"



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

UsrConfRecord user_conf;
CalConfRecord cal_conf;


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

ManuvrLinkOpts link_opts(
  100,   // ACK timeout is 100ms.
  2000,  // Send a KA every 2s.
  2048,  // MTU for this link is 2 kibi.
  TCode::CBOR,   // Payloads should be CBOR encoded.
  // This side of the link will send a KA while IDLE, and
  //   allows remote log write.
  (MANUVRLINK_FLAG_SEND_KA | MANUVRLINK_FLAG_ALLOW_LOG_WRITE)
);

UARTOpts comm_unit_uart_opts {
  .bitrate       = 115200,
  .start_bits    = 0,
  .bit_per_word  = 8,
  .stop_bits     = UARTStopBit::STOP_1,
  .parity        = UARTParityBit::NONE,
  .flow_control  = UARTFlowControl::NONE,
  .xoff_char     = 0,
  .xon_char      = 0,
  .padding       = 0
};

UARTOpts gps_uart_opts {
  .bitrate       = 9600,
  .start_bits    = 0,
  .bit_per_word  = 8,
  .stop_bits     = UARTStopBit::STOP_1,
  .parity        = UARTParityBit::NONE,
  .flow_control  = UARTFlowControl::NONE,
  .xoff_char     = 0,
  .xon_char      = 0,
  .padding       = 0
};

UARTOpts usb_comm_opts {
  .bitrate       = 115200,
  .start_bits    = 0,
  .bit_per_word  = 8,
  .stop_bits     = UARTStopBit::STOP_1,
  .parity        = UARTParityBit::NONE,
  .flow_control  = UARTFlowControl::NONE,
  .xoff_char     = 0,
  .xon_char      = 0,
  .padding       = 0
};


UARTAdapter console_uart(0, 255, 255, 255, 255, 48, 256);
UARTAdapter comm_unit_uart(1, COMM_RX_PIN, COMM_TX_PIN, 255, 255, 2048, 2048);
UARTAdapter gps_uart(6, GPS_RX_PIN, GPS_TX_PIN, 255, 255, 48, 256);

/* This object will contain our relationship with the Comm unit. */
ManuvrLink* m_link = nullptr;



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
* PMU
*******************************************************************************/
const LTC294xOpts gas_gauge_opts(
  255,     // Alert pin
  LTC294X_OPT_ADC_AUTO | LTC294X_OPT_INTEG_SENSE
);

const ManuvrPMUOpts powerplant_opts(
  255,  // 2v5 select pin does not apply on this hardware.
  RADIO_ENABLE_PIN,  // Aux regulator enable pin is handled by the touch board.
  DIGITAB_PMU_FLAG_ENABLED  // Regulator enabled @3.3v
);

const BatteryOpts battery_opts (
  BatteryChemistry::LIPO,
  3000,    // Battery capacity (in mAh)
  3.45f,   // Battery dead (in volts)
  3.60f,   // Battery weak (in volts)
  4.15f,   // Battery float (in volts)
  4.2f     // Battery max (in volts)
);

const BQ24155Opts charger_opts(
  &battery_opts,
  68,  // Sense resistor is 68 mOhm.
  BQ24155USBCurrent::LIMIT_800,  // Hardware limits (if any) on source draw..
  255, // STAT is unconnected.
  255, // ISEL is nailed high.
  BQ24155_FLAG_ISEL_HIGH  // We want to start the ISEL pin high.
);

ManuvrPMU pmu(&charger_opts, &gas_gauge_opts, &powerplant_opts);


/*******************************************************************************
* Sensors
*******************************************************************************/
BME280Settings baro_settings(
  0x76,
  BME280OSR::X1,
  BME280OSR::X1,
  BME280OSR::X1,
  BME280Mode::Normal,
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

GPSWrapper gps;
LocationFrame current_location;

/* Immediate data... */
static Vector3f64 grav;       // Gravity vector from the IMU.
static Vector3f64 acc_vect;   // Acceleration vector from the IMU.
static Vector3f64 gyr_vect;   // Gyroscopic vector from the IMU.
static Vector3f64 mag_vect0;  // Magnetism vector from the IMU.
static Vector3f64 mag_vect1;  // Magnetism vector from the DRV425 complex.


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
SensorFilter<float> graph_array_cpu_time(96, FilteringStrategy::MOVING_MED);
SensorFilter<float> graph_array_frame_rate(96, FilteringStrategy::RAW);


/* Cheeseball async support stuff. */
uint32_t boot_time         = 0;      // millis() at boot.
uint32_t config_time       = 0;      // millis() at end of setup().
uint32_t last_interaction  = 0;      // millis() when the user last interacted.
uint32_t tof_update_next   = 0;      //

/* Console junk... */
ParsingConsole console(128);
const char* console_prompt_str = "Motherflux0r # ";
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
* Link callbacks
*******************************************************************************/
void link_callback_state(ManuvrLink* cb_link) {
  StringBuilder log;
  log.concatf("Link (0x%x) entered state %s\n", cb_link->linkTag(), ManuvrLink::sessionStateStr(cb_link->getState()));
  //printf("%s\n\n", (const char*) log.string());
}


void link_callback_message(uint32_t tag, ManuvrMsg* msg) {
  StringBuilder log;
  KeyValuePair* kvps_rxd = nullptr;
  log.concatf("link_callback_message(0x%x): \n", tag, msg->uniqueId());
  msg->printDebug(&log);
  msg->getPayload(&kvps_rxd);
  if (kvps_rxd) {
    //kvps_rxd->printDebug(&log);
  }
  if (msg->expectsReply()) {
    int8_t ack_ret = msg->ack();
    log.concatf("\nlink_callback_message ACK'ing %u returns %d.\n", msg->uniqueId(), ack_ret);
  }
  //printf("%s\n\n", (const char*) log.string());
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
  graph_array_broad_ir.feedFilter(1.0 * tsl2561.getIR());
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
  stopwatch_sensor_mag.markStart();
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
* Battery state callback
*******************************************************************************/

int8_t battery_state_callback(ChargeState e) {
  return 0;
}


/*******************************************************************************
* Location callback
*******************************************************************************/

int8_t location_callback(LocationFrame* loc) {
  //gps.fetchLog(log);
  current_location.copyFrame(loc);
  //uint64_t ts = loc->timestamp;
  //if (!rtcAccurate() & (0 != ts)) {
  //  // If the clock might be stale, set it from the GPS fix.
  //  // TODO: This is not correct.
  //  uint16_t y = ts / 31556926;
  //  ts -= (y * 31556926);
  //  uint8_t m  = ts / 2629744;
  //  ts -= (m * 2629744);
  //  uint8_t d  = ts / 86400;
  //  ts -= (d * 86400);
  //  uint8_t h  = ts / 3600;
  //  ts -= (h * 3600);
  //  uint8_t mi = ts / 60;
  //  ts -= (mi * 60);
  //  uint8_t s  = ts;
  //  //setTimeAndDate(y + 1970, m+1, d+1, h, mi, s);
  //}
  //else {
  //  // The RTC seems to think it is accurate... Check to be sure...
  //}
  return 0;
}


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
      //graph_array_mag_confidence.feedFilter(compass.getError()->length());
      ret = -1;
      break;
    case SpatialSense::MAG:
      ret = 0;
      Vector3f* mag_fv = mag_filter.getData();
      graph_array_mag_strength_x.feedFilter(mag_fv->x);
      graph_array_mag_strength_y.feedFilter(mag_fv->y);
      graph_array_mag_strength_z.feedFilter(mag_fv->z);
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

int callback_link_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd = args->position_trimmed(0);
  if (0 == StringBuilder::strcasecmp(cmd, "info")) {
    m_link->printDebug(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    text_return->concatf("Link reset() returns %d\n", m_link->reset());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "hangup")) {
    text_return->concatf("Link hangup() returns %d\n", m_link->hangup());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "verbosity")) {
    switch (args->count()) {
      case 2:
        m_link->verbosity(0x07 & args->position_as_int(1));
      default:
        text_return->concatf("Link verbosity is %u\n", m_link->verbosity());
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "log")) {
    //if (1 < args->count()) {
      StringBuilder tmp_log("This is a remote log test.\n");
      int8_t ret_local = m_link->writeRemoteLog(&tmp_log, false);
      text_return->concatf("Remote log write returns %d\n", ret_local);
    //}
    //else text_return->concat("Usage: link log <logText>\n");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "desc")) {
    // Send a description request message.
    KeyValuePair a((uint32_t) millis(), "time_ms");
    a.append((uint32_t) randomUInt32(), "rand");
    int8_t ret_local = m_link->send(&a, true);
    text_return->concatf("Description request send() returns ID %u\n", ret_local);
  }
  else text_return->concat("Usage: [info|reset|hangup|verbosity|desc]\n");

  return ret;
}


int callback_conf_tools(StringBuilder* text_return, StringBuilder* args) {
  char* conf_group_str = args->position_trimmed(0);
  char* key            = args->position_trimmed(1);
  char* val            = args->position_trimmed(2);
  char* rec_type_str   = (char*) "UNKNOWN";
  ConfRecord* conf_rec = nullptr;
  text_return->concat("\n");

  if (0 == StringBuilder::strcasecmp(conf_group_str, "usr")) {
    rec_type_str = (char*) "Usr";
    conf_rec = &user_conf;
    switch (args->count()) {
      case 3:
        text_return->concatf(
          "Setting key %s to %s returns %d.\n",
          key, val, conf_rec->setConf((const char*) key, val)
        );
        break;
      default:
        conf_rec->printConf(text_return, (1 < args->count()) ? key : nullptr);
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(conf_group_str, "cal")) {
    rec_type_str = (char*) "Cal";
    conf_rec = &cal_conf;
    switch (args->count()) {
      case 3:
        text_return->concatf(
          "Setting key %s to %s returns %d.\n",
          key, val, conf_rec->setConf(key, val)
        );
        break;
      default:
        conf_rec->printConf(text_return, (1 < args->count()) ? key : nullptr);
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(conf_group_str, "pack")) {
    // Tool for serializing conf into buffers. Should support CBOR and BINARY,
    //   for interchange and local storage, respectively.
    StringBuilder ser_out;
    TCode fmt = TCode::BINARY;
    int8_t ret = 0;
    if (3 == args->count()) {
      if (0 == StringBuilder::strcasecmp(val, "cbor")) {
        fmt = TCode::CBOR;
      }
      else if (0 == StringBuilder::strcasecmp(val, "bin")) {
        fmt = TCode::BINARY;
      }
      else {
        ret = -1;
      }
      if (0 == ret) {
        if (0 == StringBuilder::strcasecmp(key, "usr")) {
          ret = user_conf.serialize(&ser_out, fmt);
          rec_type_str = (char*) "Usr";
        }
        else if (0 == StringBuilder::strcasecmp(key, "cal")) {
          ret = cal_conf.serialize(&ser_out, fmt);
          rec_type_str = (char*) "Cal";
        }

        if (0 != ret) {
          text_return->concatf("Conf serializer returned %d.\n", ret);
        }
      }
    }

    if (0 < ser_out.length()) {
      text_return->concatf("\n---< %s Conf >-------------------------------\n", rec_type_str);
      StringBuilder::printBuffer(text_return, ser_out.string(), ser_out.length(), "\t");
    }
    else {
      text_return->concat("Usage: pack [agency|usr|cal] [cbor|bin]\n");
    }
  }
  else if (0 == StringBuilder::strcasecmp(conf_group_str, "save")) {
    if (2 == args->count()) {
      ConfRecord* crec = nullptr;
      if (0 == StringBuilder::strcasecmp(key, "usr")) {
        crec = (ConfRecord*) &user_conf;
      }
      else if (0 == StringBuilder::strcasecmp(key, "cal")) {
        crec = (ConfRecord*) &cal_conf;
      }

      if (nullptr != crec) {
        text_return->concatf("Saving %s returned %d.\n", key, crec->save(key));
      }
    }
    else {
      text_return->concat("Usage: save [usr|cal]\n");
    }
  }
  else if (0 == StringBuilder::strcasecmp(conf_group_str, "load")) {
    if (2 == args->count()) {
      ConfRecord* crec = nullptr;
      if (0 == StringBuilder::strcasecmp(key, "usr")) {
        crec = (ConfRecord*) &user_conf;
      }
      else if (0 == StringBuilder::strcasecmp(key, "cal")) {
        crec = (ConfRecord*) &cal_conf;
      }

      if (nullptr != crec) {
        text_return->concatf("Loading %s returned %d.\n", key, crec->load(key));
      }
    }
    else {
      text_return->concat("Usage: load [usr|cal]\n");
    }
  }
  else {
    text_return->concat("First argument must be the conf group (usr, cal) or a subcommand (pack, save, load).\n");
  }
  return 0;
}


int callback_i2c_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  int   bus_id = args->position_as_int(0);
  char* cmd    = args->position_trimmed(1);
  int   arg2   = args->position_as_int(2);
  I2CAdapter* bus = nullptr;
  switch (bus_id) {
    case 0:    bus = &i2c0;  break;
    case 1:    bus = &i2c1;  break;
    default:
      text_return->concatf("Unsupported bus: %d\n", bus_id);
      break;
  }
  if (nullptr != bus) {
    if (0 == StringBuilder::strcasecmp(cmd, "purge")) {
      bus->purge_current_job();
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "ragepurge")) {
      bus->purge_queued_work();
      bus->purge_current_job();
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "ping")) {
      bus->ping_slave_addr((args->count() > 2) ? arg2 : 1);
      text_return->concatf("i2c%d.ping_slave_addr(0x%02x) started.\n", bus_id, arg2);
    }
    else {
      bus->printDebug(text_return);
      bus->printPingMap(text_return);
    }
  }
  return ret;
}


int callback_help(StringBuilder* text_return, StringBuilder* args) {
  if (0 < args->count()) {
    console.printHelp(text_return, args->position_trimmed(0));
  }
  else {
    console.printHelp(text_return);
  }
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


int callback_touch_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  char* cmd = args->position_trimmed(0);
  if (0 < args->count()) {
    ret = 0;
    if (0 == StringBuilder::strcasecmp(cmd, "info")) {
      touch->printDebug(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "poll")) {
      text_return->concatf("SX8634 poll() returns %d.\n", touch->poll());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
      text_return->concatf("SX8634 init() returns %d.\n", touch->init());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
      text_return->concatf("SX8634 reset() returns %d.\n", touch->reset());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "ping")) {
      text_return->concatf("SX8634 ping() returns %d\n", touch->ping());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "mode")) {
      if (1 < args->count()) {
        int mode_int = args->position_as_int(1);
        text_return->concatf("SX8634 setMode(%s) returns %d.\n", SX8634::getModeStr((SX8634OpMode) mode_int), touch->setMode((SX8634OpMode) mode_int));
      }
      text_return->concatf("SX8634 mode set to %s.\n", SX8634::getModeStr(touch->operationalMode()));
    }
  }
  return ret;
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
    case 4:
      #ifdef SPI_HAS_TRANSFER_ASYNC
        text_return->concat("SPI_HAS_TRANSFER_ASYNC is set.\n");
      #else
        text_return->concat("SPI_HAS_TRANSFER_ASYNC is NOT set.\n");
      #endif
      break;
    case 5:    break;
    case 6:    break;
    case 7:    break;
    case 8:
      display.fill(BLACK);
      draw_progress_bar_vertical(0, 0, 12, 63, CYAN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(0, 0, 12, 63, CYAN, false, false, (i * 0.01));
      }

      draw_progress_bar_vertical(14, 0, 7, 63, BLUE, true, false, 1.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(14, 0, 7, 63, BLUE, false, false, 1.0 - (i * 0.01));
      }

      draw_progress_bar_vertical(23, 0, 7, 31, YELLOW, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(23, 0, 7, 31, YELLOW, false, false, (i * 0.01));
      }

      draw_progress_bar_vertical(23, 33, 7, 31, RED, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(23, 33, 7, 31, RED, false, false, (i * 0.01));
      }

      draw_progress_bar_vertical(32, 0, 30, 63, GREEN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_vertical(32, 0, 30, 63, GREEN, false, true, (i * 0.01));
      }
      break;

    case 9:    // Progress bar test
      display.fill(BLACK);
      draw_progress_bar_horizontal(0, 14, 95, 7, CYAN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 14, 95, 7, CYAN, false, false, (i * 0.01));
      }

      draw_progress_bar_horizontal(0, 0, 95, 12, BLUE, true, false, 1.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 0, 95, 12, BLUE, false, false, 1.0-(i * 0.01));
      }

      draw_progress_bar_horizontal(0, 23, 46, 7, YELLOW, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 23, 46, 7, YELLOW, false, false, (i * 0.01));
      }

      draw_progress_bar_horizontal(48, 23, 46, 7, RED, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(48, 23, 46, 7, RED, false, false, (i * 0.01));
      }

      draw_progress_bar_horizontal(0, 34, 95, 14, GREEN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar_horizontal(0, 34, 95, 14, GREEN, false, true, (i * 0.01));
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


int callback_sensor_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  int ret_local = 0;
  char* cmd    = args->position_trimmed(0);
  int   s_id   = args->position_as_int(1);
  bool list_sensors = false;
  if (0 < args->count()) {
    if (0 == StringBuilder::strcasecmp(cmd, "info")) {
      if (1 < args->count()) {
        switch ((SensorID) s_id) {
          case SensorID::MAGNETOMETER:   magneto.printDebug(text_return);  break;
          case SensorID::BARO:           baro.printDebug(text_return);     break;
          case SensorID::LIGHT:          i2c1.printDebug(text_return);     break;
          case SensorID::UV:             uv.printDebug(text_return);       break;
          case SensorID::THERMOPILE:     grideye.printDebug(text_return);  break;
          case SensorID::LUX:            tsl2561.printDebug(text_return);  break;
          case SensorID::BATT_VOLTAGE:   grideye.printFrame(text_return);  break;
          //case SensorID::IMU:                break;
          //case SensorID::MIC:                break;
          case SensorID::GPS:            gps.printDebug(text_return);      break;
          //case SensorID::TOF:            tmp102.printDebug(text_return);        break;
          default:
            text_return->concatf("Unsupported sensor: %d\n", s_id);
            list_sensors = true;
            break;
        }
      }
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
      if (1 < args->count()) {
        switch ((SensorID) s_id) {
          case SensorID::MAGNETOMETER:   ret_local = magneto.init(&i2c1, &spi0);   break;
          case SensorID::BARO:           ret_local = baro.init();                  break;
          case SensorID::LUX:            ret_local = tsl2561.init();               break;
          case SensorID::UV:             ret_local = uv.init();                    break;
          case SensorID::THERMOPILE:     ret_local = grideye.init(&i2c1);          break;
          //case SensorID::TOF:            ret_local = tof.init(&i2c1);              break;
          //case SensorID::BATT_VOLTAGE:       break;
          case SensorID::IMU:            ret_local = read_imu();               break;
          //case SensorID::MIC:                break;
          case SensorID::GPS:            ret_local = gps.init();               break;
          //case SensorID::LIGHT:              break;
          default:
            text_return->concatf("Unsupported sensor: %d\n", s_id);
            list_sensors = true;
            break;
        }
        text_return->concatf("Sensor %d init() returned %d\n", s_id, ret_local);
      }
    }
    if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
      if (1 < args->count()) {
        switch ((SensorID) s_id) {
          // case SensorID::MAGNETOMETER:   magneto.printDebug(text_return);  break;
          case SensorID::BARO:              ret_local = baro.refresh();     break;
          // case SensorID::LIGHT:          i2c1.printDebug(text_return);     break;
          // case SensorID::UV:             uv.printDebug(text_return);       break;
          // case SensorID::THERMOPILE:     grideye.printDebug(text_return);  break;
          // case SensorID::LUX:            tsl2561.printDebug(text_return);  break;
          // case SensorID::BATT_VOLTAGE:   grideye.printFrame(text_return);  break;
          //case SensorID::IMU:                break;
          //case SensorID::TOF:            tmp102.printDebug(text_return);        break;
          default:
            text_return->concatf("Unsupported sensor: %d\n", s_id);
            list_sensors = true;
            break;
        }
        text_return->concatf("Sensor %d refresh() returned %d\n", s_id, ret_local);
      }
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "poll")) {
      if (1 < args->count()) {
        switch ((SensorID) s_id) {
          case SensorID::MAGNETOMETER:   ret_local = magneto.poll();   break;
          case SensorID::BARO:           ret_local = baro.poll();                  break;
          case SensorID::LUX:            ret_local = tsl2561.poll();               break;
          case SensorID::UV:             ret_local = uv.poll();                    break;
          case SensorID::THERMOPILE:     ret_local = grideye.poll();          break;
          //case SensorID::TOF:            ret_local = tof.poll();              break;
          //case SensorID::BATT_VOLTAGE:       break;
          //case SensorID::IMU:            ret_local = read_imu();               break;
          //case SensorID::MIC:                break;
          //case SensorID::GPS:                break;
          //case SensorID::LIGHT:              break;
          default:
            text_return->concatf("Unsupported sensor: %d\n", s_id);
            list_sensors = true;
            break;
        }
        text_return->concatf("Sensor %d poll() returned %d\n", s_id, ret_local);
      }
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "enable")) {
      if (1 < args->count()) {
        bool s_en = (1 < args->count()) ? (1 == args->position_as_int(1)) : true;
        bool en   = false;
        switch ((SensorID) s_id) {
          case SensorID::MAGNETOMETER:  magneto.power(s_en);    en = magneto.power();     break;
          case SensorID::BARO:          en = baro.enabled();            break;
          case SensorID::LUX:           tsl2561.enabled(s_en);  en = tsl2561.enabled();   break;
          case SensorID::UV:            uv.enabled(s_en);       en = uv.enabled();        break;
          case SensorID::THERMOPILE:    grideye.enabled(s_en);  en = grideye.enabled();   break;
          case SensorID::BATT_VOLTAGE:  break;
          case SensorID::IMU:           break;
          case SensorID::MIC:           break;
          case SensorID::GPS:           break;
          case SensorID::TOF:           break;
          case SensorID::LIGHT:         break;
          default:
            text_return->concatf("Unsupported sensor: %d\n", s_id);
            list_sensors = true;
            break;
        }
        text_return->concatf("Sensor %d is now %sabled\n", s_id, en?"en":"dis");
      }
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "profiler")) {
      if (args->count() > 1) {
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
    }
    else {
      ret = -1;
    }
  }
  else {   // No arguments means print the sensor index list.
    list_sensors = true;
  }

  if (list_sensors) {
    listAllSensors(text_return);
  }
  return ret;
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


int callback_console_tools(StringBuilder* text_return, StringBuilder* args) {
  //inline void setPromptString(const char* str) {    _prompt_string = (char*) str;   };
  //inline bool hasColor() {               return _console_flag(CONSOLE_FLAG_HAS_ANSI);                   };
  //inline void hasColor(bool x) {         return _console_set_flag(CONSOLE_FLAG_HAS_ANSI, x);            };
  int ret = 0;
  char* cmd    = args->position_trimmed(0);
  int   arg1   = args->position_as_int(1);
  bool  print_term_enum = false;
  if (0 == StringBuilder::strcasecmp(cmd, "echo")) {
    if (1 < args->count()) {
      console.localEcho(0 != arg1);
    }
    text_return->concatf("Console RX echo %sabled.\n", console.localEcho()?"en":"dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "history")) {
    if (1 < args->count()) {
      console.emitPrompt(0 != arg1);
      char* subcmd = args->position_trimmed(1);
      if (0 == StringBuilder::strcasecmp(subcmd, "clear")) {
        console.clearHistory();
        text_return->concat("History cleared.\n");
      }
      else if (0 == StringBuilder::strcasecmp(subcmd, "depth")) {
        if (2 < args->count()) {
          arg1 = args->position_as_int(2);
          console.maxHistoryDepth(arg1);
        }
        text_return->concatf("History depth: %u\n", console.maxHistoryDepth());
      }
      else if (0 == StringBuilder::strcasecmp(subcmd, "logerrors")) {
        if (2 < args->count()) {
          arg1 = args->position_as_int(2);
          console.historyFail(0 != arg1);
        }
        text_return->concatf("History %scludes failed commands.\n", console.historyFail()?"in":"ex");
      }
      else text_return->concat("Valid options are [clear|depth|logerrors]\n");
    }
    else console.printHistory(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "help-on-fail")) {
    if (1 < args->count()) {
      console.printHelpOnFail(0 != arg1);
    }
    text_return->concatf("Console prints command help on failure: %s.\n", console.printHelpOnFail()?"yes":"no");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "prompt")) {
    if (1 < args->count()) {
      console.emitPrompt(0 != arg1);
    }
    text_return->concatf("Console autoprompt %sabled.\n", console.emitPrompt()?"en":"dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "force")) {
    if (1 < args->count()) {
      console.forceReturn(0 != arg1);
    }
    text_return->concatf("Console force-return %sabled.\n", console.forceReturn()?"en":"dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "rxterm")) {
    if (1 < args->count()) {
      switch (arg1) {
        case 0:  case 1:  case 2:  case 3:
          console.setRXTerminator((LineTerm) arg1);
          break;
        default:
          print_term_enum = true;
          break;
      }
    }
    text_return->concatf("Console RX terminator: %s\n", ParsingConsole::terminatorStr(console.getRXTerminator()));
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "txterm")) {
    if (1 < args->count()) {
      switch (arg1) {
        case 0:  case 1:  case 2:  case 3:
          console.setTXTerminator((LineTerm) arg1);
          break;
        default:
          print_term_enum = true;
          break;
      }
    }
    text_return->concatf("Console TX terminator: %s\n", ParsingConsole::terminatorStr(console.getTXTerminator()));
  }
  else {
    ret = -1;
  }

  if (print_term_enum) {
    text_return->concat("Terminator options:\n");
    text_return->concat("\t0: ZEROBYTE\n");
    text_return->concat("\t1: CR\n");
    text_return->concat("\t2: LF\n");
    text_return->concat("\t3: CRLF\n");
  }
  return ret;
}



/*******************************************************************************
* Setup function
*******************************************************************************/
void setup() {
  platform_init();
  boot_time = millis();
  console_uart.init(&usb_comm_opts);
  console_uart.readCallback(&console);    // Attach the UART to console...
  console.setOutputTarget(&console_uart); // ...and console to UART.

  console.emitPrompt(true);
  console.setTXTerminator(LineTerm::CRLF);
  console.setRXTerminator(LineTerm::CR);
  console.localEcho(true);
  console.printHelpOnFail(true);
  console.setPromptString(console_prompt_str);

  pinMode(IMU_IRQ_PIN,    GPIOMode::INPUT_PULLUP);
  pinMode(DRV425_CS_PIN,  GPIOMode::INPUT); // Wrong
  pinMode(ANA_LIGHT_PIN,  GPIOMode::INPUT);
  pinMode(TOF_IRQ_PIN,    GPIOMode::INPUT);
  pinMode(LED_R_PIN,      GPIOMode::INPUT);
  pinMode(LED_G_PIN,      GPIOMode::INPUT);
  pinMode(LED_B_PIN,      GPIOMode::INPUT);
  pinMode(RADIO_ENABLE_PIN, GPIOMode::OUTPUT);
  setPin(RADIO_ENABLE_PIN, true);

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

  comm_unit_uart.init(&comm_unit_uart_opts);

  gps_uart.init(&gps_uart_opts);
  gps_uart.readCallback(&gps);  // Attach the GPS UART to its parser.
  gps.setCallback(location_callback);

  //SD.begin(BUILTIN_SDCARD);

  /* Allocate memory for the filters. */
  if (0 != init_sensor_memory()) {
    console_uart.write("Failed to allocate memory for sensors.\n");
  }

  graph_array_cpu_time.init();
  graph_array_frame_rate.init();

  console.defineCommand("help",        '?', ParsingConsole::tcodes_str_1, "Prints help to console.", "", 0, callback_help);
  platform.configureConsole(&console);
  console.defineCommand("touch",       ParsingConsole::tcodes_str_4, "SX8634 tools", "", 0, callback_touch_tools);
  console.defineCommand("led",         ParsingConsole::tcodes_uint_3, "LED Test", "", 1, callback_led_test);
  console.defineCommand("vib",         'v', ParsingConsole::tcodes_uint_2, "Vibrator test", "", 0, callback_vibrator_test);
  console.defineCommand("disp",        'd', ParsingConsole::tcodes_uint_1, "Display test", "", 1, callback_display_test);
  console.defineCommand("aout",        arg_list_4_float, "Mix volumes for the headphones.", "", 4, callback_aout_mix);
  console.defineCommand("fft",         arg_list_4_float, "Mix volumes for the FFT.", "", 4, callback_fft_mix);
  console.defineCommand("synth",       arg_list_4_uuff, "Synth parameters.", "", 2, callback_synth_set);
  console.defineCommand("sensor",      's', ParsingConsole::tcodes_str_4, "Sensor tools", "", 0, callback_sensor_tools);
  console.defineCommand("sfi",         ParsingConsole::tcodes_uint_1, "Sensor filter info.", "", 0, callback_sensor_filter_info);
  console.defineCommand("mfi",         ParsingConsole::tcodes_uint_1, "Meta filter info.", "", 1, callback_meta_filter_info);
  console.defineCommand("sfs",         ParsingConsole::tcodes_uint_3, "Sensor filter strategy set.", "", 2, callback_sensor_filter_set_strat);
  console.defineCommand("mfs",         ParsingConsole::tcodes_uint_3, "Meta filter strategy set.", "", 2, callback_meta_filter_set_strat);
  console.defineCommand("app",         'a', ParsingConsole::tcodes_uint_1, "Select active application.", "", 0, callback_active_app);
  console.defineCommand("aprof",       ParsingConsole::tcodes_uint_1, "Dump application profiler.", "", 0, callback_print_app_profiler);
  console.defineCommand("vol",         ParsingConsole::tcodes_float_1, "Audio volume.", "", 0, callback_audio_volume);
  console.defineCommand("i2c",         '\0', ParsingConsole::tcodes_uint_3, "I2C tools", "Usage: i2c <bus> <action> [addr]", 1, callback_i2c_tools);
  console.defineCommand("conf",        'c',  ParsingConsole::tcodes_str_3, "Dump/set conf key.", "[usr|cal|pack] [conf_key] [value]", 1, callback_conf_tools);
  console.defineCommand("console",     '\0', ParsingConsole::tcodes_str_3, "Console conf.", "[echo|prompt|force|rxterm|txterm]", 0, callback_console_tools);
  console.defineCommand("link",        'l', ParsingConsole::tcodes_str_4, "Linked device tools.", "", 0, callback_link_tools);
  console.init();

  StringBuilder ptc("Motherflux0r ");
  ptc.concat(TEST_PROG_VERSION);
  ptc.concat("\t Build date " __DATE__ " " __TIME__ "\n");
  console.printToLog(&ptc);

  magneto.configureConsole(&console);

  pmu.configureConsole(&console);
  pmu.attachCallback(battery_state_callback);

  touch = new SX8634(&_touch_opts);
  touch->assignBusInstance(&i2c0);
  touch->setButtonFxn(cb_button);
  touch->setSliderFxn(cb_slider);
  touch->setLongpressFxn(cb_longpress);

  tsl2561.assignBusInstance(&i2c1);
  uv.assignBusInstance(&i2c1);
  baro.assignBusInstance(&i2c1);
  grideye.assignBusInstance(&i2c1);
  // tof.assignBusInstance(&i2c1);

  wakelock_tof     = nullptr;
  wakelock_mag     = magneto.getWakeLock();
  wakelock_lux     = nullptr;
  wakelock_imu     = nullptr;
  wakelock_grideye = nullptr;
  wakelock_uv      = nullptr;
  wakelock_baro    = nullptr;
  wakelock_gps     = nullptr;

  wakelock_mag->referenceCounted(false);

  config_time = millis();
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

/*
* Drain any of the UARTs that have data.
*/
void poll_uarts() {
}


void loop() {
  stopwatch_main_loop_time.markStart();
  StringBuilder output;
  const uint8_t RX_BUF_LEN = 64;
  uint8_t ser_buffer[RX_BUF_LEN];
  uint8_t rx_len = 0;
  memset(ser_buffer, 0, RX_BUF_LEN);

  //last_interaction = millis();
  gps_uart.poll();
  comm_unit_uart.poll();

  spi_spin();
  i2c0.poll();
  i2c1.poll();

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

  /* Poll each sensor class. */
  read_magnetometer_sensor();
  magneto.adc.fetchLog(&output);

  stopwatch_sensor_baro.markStart();
  if (0 < baro.poll()) {
    read_baro_sensor();
  }
  stopwatch_sensor_baro.markStop();

  stopwatch_sensor_uv.markStart();
  if (0 < uv.poll()) {
    read_uv_sensor();
  }
  stopwatch_sensor_uv.markStop();

  stopwatch_sensor_lux.markStart();
  if (0 < tsl2561.poll()) {
    read_visible_sensor();
    stopwatch_sensor_lux.markStop();
  }

  if (grideye.enabled()) {
    stopwatch_sensor_grideye.markStart();
    if (0 < grideye.poll()) {
      read_thermopile_sensor();
    }
    stopwatch_sensor_grideye.markStop();
  }

  //if (tof_update_next <= millis_now) {
  //  stopwatch_sensor_tof.markStart();
  //  read_time_of_flight_sensor();
  //  stopwatch_sensor_tof.markStop();
  //  tof_update_next = millis_now + 100;
  //}

  if ((last_interaction + 100000) <= millis_now) {
    // After 100 seconds, time-out the display.
    if (&app_standby != uApp::appActive()) {
      uApp::setAppActive(AppID::HOT_STANDBY);
    }
  }
  switch (pmu.poll()) {
    case 1:
      graph_array_batt_voltage.feedFilter(pmu.battVoltage());
      graph_array_batt_current.feedFilter(pmu.ltc294x.batteryCurrent());
      break;
    default:
      break;
  }
  pmu.fetchLog(&output);

  millis_now = millis();
  stopwatch_display.markStart();
  uApp::appActive()->refresh();
  stopwatch_display.markStop();
  // For tracking framerate, convert from period in micros to hz...
  graph_array_frame_rate.feedFilter(1000000.0 / 1+stopwatch_display.meanTime());

  console.printToLog(&output);
  console_uart.poll();
  stopwatch_main_loop_time.markStop();
  graph_array_cpu_time.feedFilter(stopwatch_main_loop_time.meanTime()/1000.0);
}
