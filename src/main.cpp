#include "Motherflux0r.h"
#include "uApp.h"

#include <math.h>
#include <StringBuilder.h>
#include <TimeSeries/SensorFilter.h>
#include <C3PLogger.h>
#include <uuid.h>

#include <Audio.h>
#include <SD.h>
#include <EEPROM.h>
#include <TimeLib.h>

#include <ManuvrDrivers.h>
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
float volume_pink_noise   = 0.75;
float mix_synth_to_fft    = 0.0;
float mix_queueL_to_fft   = 0.0;
float mix_queueR_to_fft   = 0.0;
float mix_noise_to_fft    = 0.99;
float mix_synth_to_line   = 0.0;
float mix_queue_to_line   = 0.0;
float mix_noise_to_line   = 0.99;


/*******************************************************************************
* 3-Axis pipelines
*******************************************************************************/

// Magnetic pipeline
TripleAxisTerminus     mag_vect(SpatialSense::MAG, callback_3axis);   // The magnetic field vector.
TripleAxisCompass      compass(callback_3axis);
TripleAxisFork         mag_fork(&compass, &mag_vect);
TripleAxisSingleFilter mag_filter(SpatialSense::MAG, &mag_fork, FilteringStrategy::MOVING_AVG, 360, 0);

// Inertial pipeline
TripleAxisTerminus     down(SpatialSense::ACC, callback_3axis);  // The tilt sensor's best-estimate of "down".
TripleAxisFork         imu_fork(&compass, &down);


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

SPIAdapter spi0(0, SPISCK_PIN, SPIMOSI_PIN, SPIMISO_PIN, 16, 24);
I2CAdapter i2c0(&i2c0_opts, 4, 24);
I2CAdapter i2c1(&i2c1_opts, 8, 24);

PlatformUART console_uart(0, 255, 255, 255, 255, 8192, 2048);
PlatformUART comm_unit_uart(1, COMM_RX_PIN, COMM_TX_PIN, 255, 255, 2048, 2048);
PlatformUART gps_uart(6, GPS_RX_PIN, GPS_TX_PIN, 255, 255, 48, 256);


/* We use CppPotpourri's logging class, shunted to the serial port. */
C3PLogger c3p_log_obj(
  (LOGGER_FLAG_PRINT_LEVEL | LOGGER_FLAG_PRINT_TAG)
);


M2MLinkOpts link_opts(
  100,   // ACK timeout is 100ms.
  2000,  // Send a KA every 2s.
  2048,  // MTU for this link is 2 kibi.
  TCode::CBOR,   // Payloads should be CBOR encoded.
  // This side of the link will send a KA while IDLE, and
  //   allows remote log write.
  (M2MLINK_FLAG_SEND_KA | M2MLINK_FLAG_ALLOW_LOG_WRITE)
);

/* This object will contain our relationship with the Comm unit. */
M2MLink* m_link = nullptr;



/*******************************************************************************
* Display
*******************************************************************************/
/* Configuration for the back panel display. */
const SSD13xxOpts disp_opts(
  ImgOrientation::ROTATION_0,
  DISPLAY_RST_PIN,
  DISPLAY_DC_PIN,
  DISPLAY_CS_PIN
);

SSD1331 display(&disp_opts);


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
* GPIO expander config
*******************************************************************************/

// This is the desired SX1503 GPIO state on init.
#define SX1503_OUTPUT_PINS_0 ((uint8_t) ((1 << SX_PIN_ADC_OSC_EN) | (1 << SX_PIN_REGULATOR_EN)))
#define SX1503_OUTPUT_PINS_1 ((uint8_t) (1 << (SX_PIN_BANDWIDTH & 0x07)))
#define SX1503_INPUT_PINS_0  ((uint8_t) ~SX1503_OUTPUT_PINS_0)
#define SX1503_INPUT_PINS_1  ((uint8_t) ~SX1503_OUTPUT_PINS_1)

#define SX1503_INITIAL_VAL_0  0 //(SX1503_OUTPUT_PINS_0)
#define SX1503_INITIAL_VAL_1  0 //((uint8_t) 0)

uint8_t SX_CONFIG[SX1503_SERIALIZE_SIZE] = {
  0x01,                              // Serialization format
  DRV425_GPIO_IRQ_PIN, 255,          // IRQ and RESET pins. No reset pin.
  0x00, 0x00,                        // Driver flags. Nothing special.
  SX1503_INITIAL_VAL_1, SX1503_INITIAL_VAL_0, // Registers start here. Set initial output values.
  SX1503_INPUT_PINS_1,  SX1503_INPUT_PINS_0,  // Direction registers. 1 implies input.
  SX1503_INPUT_PINS_1,  SX1503_INPUT_PINS_0,  // All inputs have pull-ups.
  0, 0,                                       // No pull-downs
  SX1503_OUTPUT_PINS_1, SX1503_OUTPUT_PINS_0, // All inputs generate interrupts...
  0xff, 0xff, 0xff, 0xff,                     // ...on both edges.
  0x00, 0x00, 0x00, 0x00, 0x00,               // No use of the PLD feature.
  0x00, 0x00, 0x00, 0x00, 0x00,               // No use of the PLD feature.
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,         // No use of the PLD feature.
  0x04                                        // No BOOST. IRQ autoclears on status read.
};

/*******************************************************************************
* Sensors
*******************************************************************************/

MCP356xConfig MCP3564_CONF_OBJ(
  0x00001700,  // DIFF_A,B,C and TEMP
  0,           // No special flags
  MCP356xMode::CONTINUOUS,
  MCP356xGain::GAIN_2,
  MCP356xBiasCurrent::NONE,
  MCP356xOversamplingRatio::OSR_20480,
  MCP356xAMCLKPrescaler::OVER_2
);


const DRV425Config DRV425_CONFIG = {
  .ADC_OSC_EN_PIN   = SX_PIN_ADC_OSC_EN,
  .REGULATOR_EN_PIN = SX_PIN_REGULATOR_EN,
  .BANDWIDTH_PIN    = SX_PIN_BANDWIDTH,
  .ERR_0_PIN        = SX_PIN_ERR_0,
  .OVERRUN_0_PIN    = SX_PIN_OVERRUN_0,
  .ERR_1_PIN        = SX_PIN_ERR_1,
  .OVERRUN_1_PIN    = SX_PIN_OVERRUN_1,
  .ERR_2_PIN        = SX_PIN_ERR_2,
  .OVERRUN_2_PIN    = SX_PIN_OVERRUN_2
};

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
SX1503   sx1503(SX_CONFIG, SX1503_SERIALIZE_SIZE);
MCP356x  mag_adc(DRV425_ADC_IRQ_PIN, DRV425_CS_PIN, 255, 1, &MCP3564_CONF_OBJ);
DRV425   magneto((GPIOWrapper*) &sx1503, &DRV425_CONFIG);

GridEYE grideye(0x69, AMG8866_IRQ_PIN);
VEML6075 uv;
ICM_20948_SPI imu;
TSL2561 tsl2561(0x49, TSL2561_IRQ_PIN);
BME280I2C baro(baro_settings);
VL53L0X tof;

GPSWrapper gps;
LocationFrame current_location;

/* Profiling data */
StopWatch stopwatch_main_loop_time;
StopWatch stopwatch_display;
StopWatch stopwatch_sensor_baro;
StopWatch stopwatch_sensor_uv;
StopWatch stopwatch_sensor_grideye;
StopWatch stopwatch_sensor_imu;
StopWatch stopwatch_sensor_lux;
StopWatch stopwatch_sensor_gps;
StopWatch stopwatch_sensor_tof;
StopWatch stopwatch_touch_poll;

TimeSeries<float> graph_array_cpu_time(96);
TimeSeries<float> graph_array_frame_rate(96);


/* Cheeseball async support stuff. */
uint32_t boot_time         = 0;      // millis() at boot.
uint32_t config_time       = 0;      // millis() at end of setup().
uint32_t last_interaction  = 0;      // millis() when the user last interacted.
uint32_t tof_update_next   = 0;      //

/* Console junk... */
ParsingConsole console(128);

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
extern uAppDataMgmt app_magnetometer;


// class TestPollable : public C3PPollable {
//   public:
//     TestPollable() {};
//     ~TestPollable() {};
//
//     PollResult poll();
// };
//
// PollResult TestPollable::poll() {
//   c3p_log(LOG_LEV_WARN, "MAIN", "\nALIGNMENT_TEST\n");
//   return PollResult::NO_ACTION;
// }
//
// TestPollable test_pollable;

/* Schedules... */
// C3PScheduledPolling schedule_usb("usb_svc",      6083,  -1, false,  (C3PPollable*) &console_uart);
// C3PScheduledPolling schedule_gps("gps_svc",      10083, -1, false,  (C3PPollable*) &gps_uart);
// C3PScheduledPolling schedule_comms("comms_svc",  10983, -1, false,  (C3PPollable*) &comm_unit_uart);
C3PScheduledLambda schedule_ui {
  "ui_svc",
  40000,
  -1,      // Repeats forever if enabled.
  false,   // Disabled.
  []() {
    uApp::appActive()->refresh();
    graph_array_frame_rate.feedSeries(1000000.0 / (1+stopwatch_display.meanTime()));
    return 0;
  }
};



/*******************************************************************************
* ISRs
*******************************************************************************/

static bool imu_irq_fired = false;
void imu_isr_fxn() {  imu_irq_fired = true;  }


/*******************************************************************************
* Link callbacks
*******************************************************************************/
void link_callback_state(M2MLink* cb_link) {
  StringBuilder log;
  log.concatf("Link (0x%x) entered state %s\n", cb_link->linkTag(), M2MLink::sessionStateStr(cb_link->currentState()));
  //printf("%s\n\n", (const char*) log.string());
}


void link_callback_message(uint32_t tag, M2MMsg* msg) {
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
* TODO: This must be handled closer to the nexus of concern. Use C3PType.
*******************************************************************************/
/*
* All packets
*/
void pack_cbor_comm_packet(cbor::encoder* pkt, CommPeer* peer) {
//  const uint8_t PROTO_VER = 0;
//  if (nullptr == peer) {
//    pkt->write_map(3);
//  }
//  else {
//    pkt->write_map(4);
//    peer->serializeCBOR(pkt, PROTO_VER);
//  }
//  pkt->write_string("comver");
//  pkt->write_int(PROTO_VER);   // The protocol version
//
//    pkt->write_string("orig");
//    pkt->write_map(4);
//      pkt->write_string("mod");
//      pkt->write_string("Motherflux0r");
//      pkt->write_string("ser");
//      pkt->write_int(1);
//      pkt->write_string("firm_ver");
//      pkt->write_string(TEST_PROG_VERSION);
//      pkt->write_string("ts");
//      pkt->write_tag(1);
//      pkt->write_int((uint) now());
//    pkt->write_string("pdu");
}


void package_sensor_data_cbor(StringBuilder* cbor_return) {
//  cbor::output_dynamic out;
//  cbor::encoder encoded(out);
//  encoded.write_map(2);
//    encoded.write_string("ds_ver");
//    encoded.write_int(0);
//
//    encoded.write_string("origin");
//    encoded.write_map(4);
//      encoded.write_string("model");
//      encoded.write_string("Motherflux0r");
//      encoded.write_string("ser");
//      encoded.write_int(1);
//      encoded.write_string("fm_ver");
//      encoded.write_string(TEST_PROG_VERSION);
//      encoded.write_string("ts");
//      encoded.write_tag(1);
//      encoded.write_int((uint) now());
//
//    encoded.write_string("meta");
//    encoded.write_map(2);
//      encoded.write_string("build_data");
//      encoded.write_tag(1);
//      encoded.write_int(1584023014);
//      encoded.write_string("cal_date");
//      encoded.write_tag(1);
//      encoded.write_int(1584035000);
//
//  cbor_return->concat(out.data(), out.size());
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
      //graph_array_mag_confidence.feedSeries(compass.getError()->length());
      ret = -1;
      break;
    case SpatialSense::MAG:
      {
        Vector3f* mag_fv = mag_filter.getData();
        graph_array_mag_strength_x.feedSeries(mag_fv->x);
        graph_array_mag_strength_y.feedSeries(mag_fv->y);
        graph_array_mag_strength_z.feedSeries(mag_fv->z);
        ret = 0;
      }
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
* ADC callbacks
*******************************************************************************/
double  diff_chans[3]  = {0.0d, 0.0d, 0.0d};
uint8_t adc_chans_seen = 0;

void callback_adc_value(uint8_t chan, double voltage) {
  uint8_t chan_idx  = 2;
  uint8_t chan_mask = 0;

  switch (chan) {
    case 8:   chan_idx--;
    case 9:   chan_idx--;
    case 10:
      chan_mask = (1 << chan_idx);
      if (adc_chans_seen & chan_mask) {
        // Some piece of clockwork skipped. We aren't cycling continuously.
      }
      else {
        adc_chans_seen |= chan_mask;
        diff_chans[chan_idx] = voltage;
        if (7 == adc_chans_seen) {
          Vector3f64 ana_vect(diff_chans[0], diff_chans[1], diff_chans[2]);
          int8_t mag_ret = magneto.provideAnalogVector(&ana_vect);
          if (0 != mag_ret) {
            c3p_log(LOG_LEV_WARN, "main", "Magnetometer rejected vector (%d).", mag_ret);
          }
          adc_chans_seen = 0;
          diff_chans[0] = 0.0d;
          diff_chans[1] = 0.0d;
          diff_chans[2] = 0.0d;
        }
      }
      break;
    case 12:
      break;
    default:
      c3p_log(LOG_LEV_DEBUG, "main", "callback_adc_value(%u, %.5fV).", chan, voltage);
      break;
  }
}


/*******************************************************************************
* Touch callbacks
*******************************************************************************/

void cb_button(int button, bool pressed) {
  last_interaction = millis();
  if (pressed) {
    vibrateOn(19);
  }
  ledOn(LED_B_PIN, 2, (4095 - graph_array_ana_light.value() * 3000));
  uint16_t value = touch->buttonStates();
  last_interaction = millis();
  uApp::appActive()->deliverButtonValue(value);
}


void cb_slider(int slider, int value) {
  last_interaction = millis();
  ledOn(LED_R_PIN, 30, (4095 - graph_array_ana_light.value() * 3000));
  uApp::appActive()->deliverSliderValue(value);
}


void cb_longpress(int button, uint32_t duration) {
  ledOn(LED_G_PIN, 30, (4095 - graph_array_ana_light.value() * 3000));
}


/*******************************************************************************
* Console callbacks
*******************************************************************************/
int callback_help(StringBuilder* text_return, StringBuilder* args) {
  return console.console_handler_help(text_return, args);
}

int callback_console_tools(StringBuilder* text_return, StringBuilder* args) {
  return console.console_handler_conf(text_return, args);
}

int callback_touch_tools(StringBuilder* text_return, StringBuilder* args) {
  return touch->console_handler(text_return, args);
}

int callback_magnetometer_fxns(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd = args->position_trimmed(0);
  // We interdict if the command is something specific to this application.
  if (0 == StringBuilder::strcasecmp(cmd, "adc")) {
    args->drop_position(0);  // Drop this cmd and defer to the MCP356x's console handler.
    ret = mag_adc.console_handler(text_return, args);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "gpio")) {
    args->drop_position(0);  // Drop this cmd and defer to the SX1503's console handler.
    ret = sx1503.console_handler(text_return, args);
  }
  else {
    ret = magneto.console_handler(text_return, args);
  }
  return ret;
}

int callback_pmu_tools(StringBuilder* text_return, StringBuilder* args) {
  return pmu.console_handler(text_return, args);
}



int callback_link_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  char* cmd = args->position_trimmed(0);
  // We interdict if the command is something specific to this application.
  if (0 == StringBuilder::strcasecmp(cmd, "desc")) {
    // Send a description request message.
    KeyValuePair a("time_ms", (uint32_t) millis());
    a.append((uint32_t) randomUInt32(), "rand");
    int8_t ret_local = m_link->send(&a, true);
    text_return->concatf("Description request send() returns ID %u\n", ret_local);
    ret = 0;
  }
  else ret = m_link->console_handler(text_return, args);

  return ret;
}


int callback_spi_debug(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (0 < args->count()) {
    int bus_id = args->position_as_int(0);
    args->drop_position(0);
    switch (bus_id) {
      case 0:   ret = spi0.console_handler(text_return, args);  break;
      default:
        text_return->concatf("Unsupported bus: %d\n", bus_id);
        break;
    }
  }
  return ret;
}


#if defined(CONFIG_C3P_I2CADAPTER_ENABLE_CONSOLE)
int callback_i2c_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (0 < args->count()) {
    int bus_id = args->position_as_int(0);
    args->drop_position(0);
    switch (bus_id) {
      case 0:   ret = i2c0.console_handler(text_return, args);  break;
      case 1:   ret = i2c1.console_handler(text_return, args);  break;
      default:
        text_return->concatf("Unsupported bus: %d\n", bus_id);
        break;
    }
  }
  return ret;
}
#endif


int callback_conf_tools(StringBuilder* text_return, StringBuilder* args) {
  char* conf_group_str = args->position_trimmed(0);
  char* key            = args->position_trimmed(1);
  char* val            = args->position_trimmed(2);
  char* rec_type_str   = (char*) "UNKNOWN";
  ConfRecord* conf_rec = nullptr;
  text_return->concat("\n");

  if (0 == StringBuilder::strcasecmp(conf_group_str, "usr")) {
    rec_type_str = (char*) "Usr";
    conf_rec = &usr_conf;
    switch (args->count()) {
      case 3:
        text_return->concatf(
          "Setting key %s to %s returns %d.\n",
          key, val, conf_rec->setConf((const char*) key, val)
        );
        break;
      default:
        conf_rec->printConfRecord(text_return, (1 < args->count()) ? key : nullptr);
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
        conf_rec->printConfRecord(text_return, (1 < args->count()) ? key : nullptr);
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
          ret = usr_conf.serialize(&ser_out, fmt);
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
        crec = (ConfRecord*) &usr_conf;
      }
      else if (0 == StringBuilder::strcasecmp(key, "cal")) {
        crec = (ConfRecord*) &cal_conf;
      }

      if (nullptr != crec) {
        //text_return->concatf("Saving %s returned %d.\n", key, crec->save(key));
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
        crec = (ConfRecord*) &usr_conf;
      }
      else if (0 == StringBuilder::strcasecmp(key, "cal")) {
        crec = (ConfRecord*) &cal_conf;
      }

      if (nullptr != crec) {
        //text_return->concatf("Loading %s returned %d.\n", key, crec->load(key));
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


int callback_print_app_profiler(StringBuilder* text_return, StringBuilder* args) {
  int8_t ret = 0;
  char* cmd = args->position_trimmed(0);
  if (0 == StringBuilder::strcasecmp(cmd, "app")) {
    StopWatch::printDebugHeader(text_return);
    app_boot.printStopwatch(text_return);
    app_meta.printStopwatch(text_return);
    app_touch_test.printStopwatch(text_return);
    app_tricorder.printStopwatch(text_return);
    app_root.printStopwatch(text_return);
    app_synthbox.printStopwatch(text_return);
    app_magnetometer.printStopwatch(text_return);
    app_standby.printStopwatch(text_return);
    app_config.printStopwatch(text_return);
    app_comms.printStopwatch(text_return);
    stopwatch_main_loop_time.printDebug("Main loop", text_return);
    stopwatch_display.printDebug("Display", text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "sch")) {
    C3PScheduler::getInstance()->printDebug(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    app_boot.resetStopwatch();
    app_meta.resetStopwatch();
    app_touch_test.resetStopwatch();
    app_tricorder.resetStopwatch();
    app_root.resetStopwatch();
    app_synthbox.resetStopwatch();
    app_magnetometer.resetStopwatch();
    app_standby.resetStopwatch();
    app_config.resetStopwatch();
    app_comms.resetStopwatch();
    stopwatch_main_loop_time.reset();
    stopwatch_display.reset();
    text_return->concat("Profiler reset.\n");
  }
  else {
    ret = -1;
  }
  return ret;
}


int callback_display_test(StringBuilder* text_return, StringBuilder* args) {
  int ret = display.console_handler(text_return, args);
  if (0 == ret) return ret;
  // ^this^ is using superglue instead of stiches.

  int arg0 = args->position_as_int(0);
  uint32_t millis_0 = millis();
  uint32_t millis_1 = millis_0;
  switch (arg0) {
    case 1:
      #ifdef SPI_HAS_TRANSFER_ASYNC
        text_return->concat("SPI_HAS_TRANSFER_ASYNC is set.\n");
      #else
        text_return->concat("SPI_HAS_TRANSFER_ASYNC is NOT set.\n");
      #endif
      break;
    case 4:    // Vector display test
      display.fill(BLACK);
      // draw_3vector(0, 0, 50, 50, RED,    true,  false, 1.0, 0.0, 0.0);
      // draw_3vector(0, 0, 50, 50, GREEN,  false, false, 0.0, 1.0, 0.0);
      // draw_3vector(0, 0, 50, 50, BLUE,   false, false, 0.0, 0.0, 1.0);
      // draw_3vector(0, 0, 50, 50, YELLOW, false, false, 1.0, 1.0, 0.0);
      // draw_3vector(0, 0, 50, 50, CYAN,   false, false, 0.13, 0.65, 0.0);
      break;
    default:
      return -1;
  }
  millis_1 = millis();
  text_return->concatf("Display update took %ums\n", millis_1-millis_0);
  return ret;
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
          case SensorID::BARO:           ret_local = baro.init();                  break;
          case SensorID::LUX:            ret_local = tsl2561.init();               break;
          case SensorID::UV:             ret_local = uv.init();                    break;
          case SensorID::THERMOPILE:     ret_local = grideye.init(&i2c1);          break;
          //case SensorID::TOF:            ret_local = tof.init(&i2c1);              break;
          //case SensorID::BATT_VOLTAGE:       break;
          //case SensorID::IMU:            ret_local = read_imu();               break;
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
          case SensorID::BARO:              ret_local = baro.refresh();     break;
          // case SensorID::LIGHT:          i2c1.printDebug(text_return);     break;
           case SensorID::UV:               ret_local = uv.refresh();       break;
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
    else if (0 == StringBuilder::strcasecmp(cmd, "enable")) {
      if (1 < args->count()) {
        bool s_en = (1 < args->count()) ? (1 == args->position_as_int(1)) : true;
        bool en   = false;
        switch ((SensorID) s_id) {
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
        //stopwatch_sensor_mag.reset();
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
      //stopwatch_sensor_mag.printDebug("Magnetometer", text_return);
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
        graph_array_humidity.printSeries(text_return);
        graph_array_air_temp.printSeries(text_return);
        graph_array_pressure.printSeries(text_return);
        graph_array_psu_temp.printSeries(text_return);
        break;
      case SensorID::LIGHT:
        graph_array_ana_light.printSeries(text_return);
        break;
      case SensorID::UV:
        text_return->concat("UV Filters:\n");
        graph_array_uva.printSeries(text_return);
        graph_array_uvb.printSeries(text_return);
        graph_array_uvi.printSeries(text_return);
        break;
      case SensorID::THERMOPILE:
        text_return->concat("GridEye Filters:\n");
        graph_array_therm_mean.printSeries(text_return);
        graph_array_therm_frame.printSeries(text_return);
        break;
      case SensorID::BATT_VOLTAGE:
      case SensorID::IMU:
      case SensorID::MIC:
      case SensorID::GPS:
      case SensorID::TOF:
        break;
      case SensorID::LUX:
        text_return->concat("Lux Filters:\n");
        graph_array_visible.printSeries(text_return);
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
        graph_array_cpu_time.printSeries(text_return);
        break;
      case 1:
        text_return->concat("Framerate Filter:\n");
        graph_array_frame_rate.printSeries(text_return);
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

int callback_checklist_dump(StringBuilder* text_return, StringBuilder* args) {
  checklist_boot.printDebug(text_return, "Boot Checklist");
  checklist_cyclic.printDebug(text_return, "Cyclic Checklist");
  return 0;
}


int callback_logger_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd = args->position_trimmed(0);
  int   arg = args->position_as_int(1);

  if (0 == StringBuilder::strcasecmp(cmd, "test")) {
    c3p_log(LOG_LEV_ALERT, "TAG", "LOG_LEV_ALERT");
    c3p_log(LOG_LEV_EMERGENCY, "", "LOG_LEV_EMERGENCY");
    c3p_log(LOG_LEV_NOTICE, __PRETTY_FUNCTION__, "LOG_LEV_NOTICE");
    c3p_log(LOG_LEV_ERROR, "TAG-inc", "LOG_LEV_ERROR");
    c3p_log(LOG_LEV_DEBUG, __PRETTY_FUNCTION__, "LOG_LEV_DEBUG");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "tag")) {
    if (1 < args->count()) c3p_log_obj.printTag(0 != arg);
    text_return->concatf("Print tag:      %c\n", c3p_log_obj.printTag()?'y':'n');
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "severity")) {
    if (1 < args->count()) c3p_log_obj.printSeverity(0 != arg);
    text_return->concatf("Print severity: %c\n", c3p_log_obj.printSeverity()?'y':'n');
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "time")) {
    //if (1 < args->count()) c3p_log_obj.printTime(0 != arg);
    c3p_log_obj.printTime(false);
    text_return->concatf("Print time:     %c\n", c3p_log_obj.printTime()?'y':'n');
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "t1")) {
    c3p_log_obj.printTime(true);
    text_return->concatf("Print time:     %c\n", c3p_log_obj.printTime()?'y':'n');
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "t2")) {
    c3p_log_obj = C3PLogger(0, &console_uart);
    text_return->concatf("Print time:     %c\n", c3p_log_obj.printTime()?'y':'n');
    text_return->concatf("Print severity: %c\n", c3p_log_obj.printSeverity()?'y':'n');
    text_return->concatf("Print tag:      %c\n", c3p_log_obj.printTag()?'y':'n');
  }
  else {
    ret = -1;
  }

  return ret;
}



/*******************************************************************************
* Logger support.
*******************************************************************************/

/**
* This function is declared in CppPotpourri (AbstractPlatform.h).
* Shunt the message into the C3PLogger formatter.
*
* @param severity is the syslog-style importance of the message.
* @param tag is the free-form source of the message.
* @param msg contains the log content.
*/
void c3p_log(uint8_t severity, const char* tag, StringBuilder* msg) {
  c3p_log_obj.print(severity, tag, msg);
  // TODO: The line above should be the only line required.
  // We want to benefit from the line-ending conversion features in
  //   ParsingConsole, so we don't use the BufferAccepter API.
  StringBuilder l_tmp;
  c3p_log_obj.fetchLog(&l_tmp);
  console_uart.pushBuffer(&l_tmp);
}



/*******************************************************************************
* Sensor service functions
*******************************************************************************/

/*
* Reads the VEML6075 and adds the data to the pile.
*/
int8_t read_uv_sensor() {
  int8_t ret = 0;
  graph_array_uva.feedSeries(uv.uva());
  graph_array_uvb.feedSeries(uv.uvb());
  graph_array_uvi.feedSeries(uv.index());
  return ret;
}


/*
* Reads the BME280 and adds the data to the pile.
*/
int8_t read_baro_sensor() {
  int8_t ret = 0;
  graph_array_humidity.feedSeries(baro.hum());
  graph_array_air_temp.feedSeries(baro.temp());
  graph_array_pressure.feedSeries(baro.pres());
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
  ret = graph_array_visible.feedSeries(1.0 * tsl2561.getLux());
  graph_array_broad_ir.feedSeries(1.0 * tsl2561.getIR());
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
      graph_array_therm_frame.feedSeries(grideye.getPixelTemperature(pix_idx));
    }
  }
  graph_array_therm_mean.feedSeries(graph_array_therm_frame.value());
  return ret;
}


int8_t read_time_of_flight_sensor() {
  uint32_t tof_value = tof.readRangeContinuousMillimeters();
  if (!tof.timeoutOccurred()) {
    graph_array_time_of_flight.feedSeries(tof_value);
  }
  return 0;
}



/*******************************************************************************
* Setup function
*******************************************************************************/
void setup() {
  platform_init();
  boot_time = millis();
  checklist_boot.resetSequencer();
  checklist_cyclic.resetSequencer();

  checklist_boot.requestSteps(CHKLST_BOOT_MASK_BOOT_COMPLETE);
  checklist_cyclic.requestSteps(CHKLST_CYC_MAG_ADC_CONF);
  checklist_cyclic.requestSteps(CHKLST_CYC_LUX_INIT);
  checklist_cyclic.requestSteps(CHKLST_CYC_GRIDEYE_CONF);
  checklist_cyclic.requestSteps(CHKLST_CYC_UV_CONF);
  checklist_cyclic.requestSteps(CHKLST_CYC_BARO_CONF);

  AudioMemory(32);
  analogWriteResolution(12);

  //SD.begin(BUILTIN_SDCARD);

  mag_adc.setReferenceRange(3.6, 0.0);
  mag_adc.setMCLKFrequency(19660800.0);   // 19.6608 MHz
  mag_adc.valueCallback(callback_adc_value);
  mag_adc.setCircuitSettleTime(10);
  magneto.attachPipe(&mag_filter);   // Connect the driver to its pipeline.

  touch = new SX8634(&_touch_opts);

  //C3PScheduler::getInstance()->addSchedule(&schedule_usb);
  //C3PScheduler::getInstance()->addSchedule(&schedule_gps);
  //C3PScheduler::getInstance()->addSchedule(&schedule_comms);
  C3PScheduler::getInstance()->addSchedule(&schedule_ui);

  config_time = millis();
}


/*******************************************************************************
* Main loop
*******************************************************************************/

void spi_spin() {
  int respin = 2;
  while ((respin-- > 0) && (PollResult::ACTION == spi0.poll())) {}
  respin = 2;
  while ((respin-- > 0) && (0 < spi0.service_callback_queue())) {}
}


void loop() {
  //////////////////////////////////////////////////////////////////////////////
  // Scheduler service
  C3PScheduler* scheduler = C3PScheduler::getInstance();
  scheduler->advanceScheduler();
  scheduler->serviceSchedules();

  //////////////////////////////////////////////////////////////////////////////
  // Checklist polling
  if (!checklist_boot.request_fulfilled()) {
    if (0 < checklist_boot.poll()) {
      // Actions were taken.
    }
  }
  else if (!checklist_cyclic.request_fulfilled()) {
    if (0 < checklist_cyclic.poll()) {
      // Actions were taken.
    }
  }
  stopwatch_main_loop_time.markStart();

  //////////////////////////////////////////////////////////////////////////////
  // BusQueue polling
  spi_spin();
  i2c0.poll();
  i2c1.poll();


  //////////////////////////////////////////////////////////////////////////////
  // Boot conditional service blocks
  if (checklist_boot.all_steps_have_passed(CHKLST_BOOT_INIT_GPS)) {
    gps_uart.poll();
  }

  if (checklist_boot.all_steps_have_passed(CHKLST_BOOT_INIT_COMMS)) {
    comm_unit_uart.poll();
  }

  if (checklist_boot.all_steps_have_passed(CHKLST_BOOT_INIT_TOUCH_FOUND)) {
    stopwatch_touch_poll.markStart();
    int8_t t_res = touch->poll();
    if (0 < t_res) {
      // Something changed in the hardware.
    }
    stopwatch_touch_poll.markStop();
  }

  /* Run our async cleanup stuff. */
  uint32_t millis_now = millis();
  timeoutCheckVibLED();

  /* Poll each sensor class. */
  if (checklist_boot.all_steps_have_passed(CHKLST_BOOT_INIT_MAG_GPIO)) {
    sx1503.poll();
  }

  if (checklist_boot.all_steps_have_passed(CHKLST_BOOT_INIT_PMU_CHARGER | CHKLST_BOOT_INIT_PMU_GUAGE)) {
    switch (pmu.poll()) {
      case 1:
        graph_array_batt_voltage.feedSeries(pmu.battVoltage());
        graph_array_batt_current.feedSeries(pmu.ltc294x.batteryCurrent());
        break;
      default:
        break;
    }
  }


  //////////////////////////////////////////////////////////////////////////////
  // Cyclic conditional service blocks
  if (checklist_cyclic.all_steps_have_passed(CHKLST_CYC_IMU_INIT)) {
    if (imu_irq_fired) {
      imu_irq_fired = false;
      //read_imu();
    }
  }

  if (checklist_cyclic.all_steps_have_passed(CHKLST_CYC_BARO_INIT)) {
    if (0 < baro.poll()) {
      read_baro_sensor();
    }
  }

  if (checklist_cyclic.all_steps_have_passed(CHKLST_CYC_UV_INIT)) {
    if (0 < uv.poll()) {
      read_uv_sensor();
    }
  }

  if (checklist_cyclic.all_steps_have_passed(CHKLST_CYC_GRIDEYE_INIT)) {
    if (0 < grideye.poll()) {
      read_thermopile_sensor();
    }
  }

  if (magneto.power()) {
    mag_adc.poll();
  }

  if (checklist_cyclic.all_steps_have_passed(CHKLST_CYC_ANA_INIT)) {
  }

  if (checklist_cyclic.all_steps_have_passed(CHKLST_CYC_LUX_INIT)) {
    if (0 < tsl2561.poll()) {
      read_visible_sensor();
    }
  }

  if (checklist_cyclic.all_steps_have_passed(CHKLST_CYC_TOF_INIT)) {
    //  read_time_of_flight_sensor();
    //  stopwatch_sensor_tof.markStop();
    //  tof_update_next = millis_now + 100;
  }


  //////////////////////////////////////////////////////////////////////////////
  // Last interaction polling
  if ((last_interaction + 100000) <= millis_now) {
    // After 100 seconds, time-out the display.
    if (&app_standby != uApp::appActive()) {
      uApp::setAppActive(AppID::HOT_STANDBY);
    }
  }

  console_uart.poll();
  stopwatch_main_loop_time.markStop();
  graph_array_cpu_time.feedSeries(stopwatch_main_loop_time.meanTime()/1000.0);
}
