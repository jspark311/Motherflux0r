#include <inttypes.h>
#include <stdint.h>

/* CppPotpourri */
#include <CppPotpourri.h>
#include <StringBuilder.h>
#include <AsyncSequencer.h>
#include <SensorFilter.h>
#include <Image/Image.h>
#include <Image/ImageUtils.h>

#include <Console/C3PConsole.h>
#include <M2MLink/M2MLink.h>
#include <BusQueue/SPIAdapter.h>
#include <BusQueue/I2CAdapter.h>
#include <BusQueue/UARTAdapter.h>
#include <Pipes/BufferAccepter/GPSWrapper/GPSWrapper.h>
#include <Pipes/TripleAxisPipe/TripleAxisPipe.h>
#include <Pipes/TripleAxisPipe/TripleAxisCompass.h>
#include <ManuvrDrivers.h>
#include <Storage/RecordTypes/ConfRecord.h>
#include <TimerTools/C3PScheduler.h>
#include <Composites/ManuvrPMU/ManuvrPMU-r2.h>

/* This project */
#include "CommPeer.h"
#include "SensorGlue.h"

/* Teensyduino Audio library */
#include <Audio.h>


#ifndef __MOTHERFLUX0R_H__
#define __MOTHERFLUX0R_H__

#define TEST_PROG_VERSION           "1.8"
#define TOUCH_DWELL_LONG_PRESS       1000  // Milliseconds for "long-press".
#define E_VAL                      1.0184

/*******************************************************************************
* Pin definitions and hardware constants.
*******************************************************************************/
#define COMM_TX_PIN          0   // MCU RX
#define COMM_RX_PIN          1   // MCU TX
#define DRV425_GPIO_IRQ_PIN  2
#define DRV425_ADC_IRQ_PIN   3
#define DRV425_CS_PIN        4
#define VIBRATOR_PIN         5
#define TOUCH_IRQ_PIN        6
#define DAC_DIN_PIN          7
#define RADIO_ENABLE_PIN     8   // Sets the aux regulator running.
#define TSL2561_IRQ_PIN    255 // 9
#define DISPLAY_CS_PIN      10
#define SPIMOSI_PIN         11
#define SPIMISO_PIN         12
#define SPISCK_PIN          13
#define LED_R_PIN           14
#define LED_G_PIN           15
#define SCL1_PIN            16   // Sensor service bus.
#define SDA1_PIN            17   // Sensor service bus.
#define SDA0_PIN            18   // Touch controller and PMU bus.
#define SCL0_PIN            19   // Touch controller and PMU bus.
#define DAC_LCK_PIN         20   // XMT is tied high.
#define DAC_BCK_PIN         21   // FLT, FMT are tied low.
#define ANA_LIGHT_PIN       22   // PIN_A8
#define DAC_SCL_PIN         23
#define GPS_RX_PIN          24   // MCU output
#define GPS_TX_PIN          25   // MCU input
#define DISPLAY_DC_PIN      26
#define TOF_IRQ_PIN         27   // A16
#define TOUCH_RESET_PIN     28
#define IMU_IRQ_PIN         29
#define AMG8866_IRQ_PIN    255 // 30
#define IMU_CS_PIN          31
#define DISPLAY_RST_PIN     32
#define LED_B_PIN           33

/* Pin mappings for the SX1503... */
#define SX_PIN_ADC_OSC_EN       6
#define SX_PIN_REGULATOR_EN     7
#define SX_PIN_BANDWIDTH        8
#define SX_PIN_ERR_0           10
#define SX_PIN_OVERRUN_0       11
#define SX_PIN_ERR_1           12
#define SX_PIN_OVERRUN_1       13
#define SX_PIN_ERR_2           14
#define SX_PIN_OVERRUN_2       15


/*******************************************************************************
* Application color scheme (16-bit)
*******************************************************************************/
#define	BLACK           0x0000
#define	BLUE            0x1F00
#define	RED             0x00F8
#define	GREEN           0xE007
#define CYAN            0xFF07
#define MAGENTA         0x1FF8
#define YELLOW          0xE0FF
#define GREY            0x79EF
#define WHITE           0xFFFF

// For sanity, we use a common color scheme for 3-space axes.
#define COLOR_X_AXIS    0x80F9
#define COLOR_Y_AXIS    0xF207
#define COLOR_Z_AXIS    0x031F

#define COLOR_BATT_VOLTAGE  0x03F2
#define COLOR_BATT_CURRENT  0x7EF2

#define COLOR_AIR_PRESSURE  0x03E0
#define COLOR_AIR_HUMIDITY  0x3EE3
#define COLOR_AIR_TEMP      0x83D0



/*******************************************************************************
* Data handling flags
*******************************************************************************/
#define DATA_REC_FLAG_               0x80000000   //
#define DATA_RELAY_FLAG_             0x80000000   //


/*******************************************************************************
* DataRecords
* The program has a set of configurations that it defines and loads at runtime.
* This names the record types and their possible keys.
*******************************************************************************/
enum class CalConfKey : uint8_t {
  CAL_MAG_HARD_IRON,           // Vector3
  CAL_MAG_SOFT_IRON,           // Vector3
  CAL_MAG_ORIENTATION,         // Vector3
  CAL_IMU_ORIENTATION,         // Vector3
  CAL_THERMAL_THRESHOLD_LOW,   // int8
  CAL_THERMAL_THRESHOLD_HIGH,  // int8
  CAL_BATTERY_THRESHOLD_LOW,   // Float
  CAL_BATTERY_THRESHOLD_HIGH,  // Float
  CAL_DATE,                    // uint64
  INVALID
};

enum class UsrConfKey : uint8_t {
  USR_UNIT_LOCKED,
  USR_VIB_LEVEL,
  USR_TIMEOUT_IDLE,
  USR_TIMEOUT_SHUTDOWN,
  USR_BLUETOOTH_ENABLED,
  USR_UNITS,
  USR_VOLUME,
  USR_DISP_BRIGHTNESS,
  USR_LONGPRESS_THRESHOLD,
  USR_APP_POLLING_PERIOD,
  USR_TYPEMATIC_PERIOD,
  USR_SERIAL_BAUD_RATE,
  USR_MAG_OVERSAMPLE,
  USR_MAG_FILTER_DEPTH,
  USR_MAG_BANDWIDTH,
  INVALID
};


/*******************************************************************************
* Types
*******************************************************************************/
enum class SensorID : uint8_t {
  BARO          = 0,  //
  MAGNETOMETER  = 1,  //
  IMU           = 2,  //
  LIGHT         = 3,  //
  MIC           = 4,  //
  UV            = 5,  //
  GPS           = 6,  //
  THERMOPILE    = 7,  //
  TOF           = 8,  //
  BATT_VOLTAGE  = 9,  //
  LUX           = 10  //
};



/*******************************************************************************
* Function prototypes
*******************************************************************************/
void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity = 3500);
void vibrateOn(uint32_t duration, uint16_t intensity = 4095);
void timeoutCheckVibLED();

const char* const getSensorIDString(SensorID);
void listAllSensors(StringBuilder*);

float FindE(int bands, int bins);
void  printFFTBins(StringBuilder*);
uint16_t* bitmapPointer(unsigned int idx);

/* Display helper routines */
void render_button_icon(uint8_t sym, int x, int y, uint16_t color);

// TODO: Nasty shim. Shouldn't be needed.
void draw_graph_obj(
  Image*,
  PixUInt x, PixUInt y, PixUInt w, PixUInt h, uint32_t color,
  bool opt1, bool opt2, bool opt3,
  SensorFilter<float>*
);

void draw_graph_obj(Image* FB,
  PixUInt x, PixUInt y, PixUInt w, PixUInt h, uint32_t color1, uint32_t color2, uint32_t color3,
  bool opt1, bool opt2, bool opt3,
  SensorFilter<float>* filter1, SensorFilter<float>* filter2, SensorFilter<float>* filter3
);


// TODO: Enum this?
#define ICON_CANCEL    0  // Flash-resident bitmap
#define ICON_ACCEPT    1  // Flash-resident bitmap
#define ICON_THERMO    2  // Flash-resident bitmap
#define ICON_IMU       3  // Flash-resident bitmap
#define ICON_GPS       4  // Flash-resident bitmap
#define ICON_LIGHT     5  // Flash-resident bitmap
#define ICON_UVI       6  // Flash-resident bitmap
#define ICON_SOUND     7  // Flash-resident bitmap
#define ICON_RH        8  // Flash-resident bitmap
#define ICON_MIC       9  // Flash-resident bitmap
#define ICON_MAGNET   10  // Flash-resident bitmap
#define ICON_BATTERY  11  // Flash-resident bitmap
#define BUTTON_LEFT  252  // Drawn with code
#define BUTTON_RIGHT 253  // Drawn with code
#define BUTTON_UP    254  // Drawn with code
#define BUTTON_DOWN  255  // Drawn with code


/*******************************************************************************
* Hardware checklist definitions
*******************************************************************************/
// These checks happen once on on every runtime.
#define CHKLST_BOOT_BUS_SPI0         0x00000001  // SPI is operational.
#define CHKLST_BOOT_BUS_I2C0         0x00000002  // I2C is operational.
#define CHKLST_BOOT_BUS_I2C1         0x00000004  // I2C is operational.
#define CHKLST_BOOT_TOUCH            0x00000008  // The config for the touch board was written.
#define CHKLST_BOOT_DISPLAY          0x00000010  // The display is initialized.
#define CHKLST_BOOT_MAG_GPIO         0x00000020  //
#define CHKLST_BOOT_MAG_ADC          0x00000040  //
#define CHKLST_BOOT_AUDIO_STACK      0x00000080  //
#define CHKLST_BOOT_INIT_BARO        0x00000100  //
#define CHKLST_BOOT_INIT_UV          0x00000200  //
#define CHKLST_BOOT_INIT_GRIDEYE     0x00000400  //

// Cyclic hardware states and their dependencies.
//#define CHKLST_CYC_  0x00000001  //
//#define CHKLST_CYC_  0x00000002  //
//#define CHKLST_CYC_  0x08000004  //
//#define CHKLST_CYC_  0x00000008  //


/*******************************************************************************
* Externed singleton resources
*******************************************************************************/
extern ConfRecordValidation<CalConfKey> cal_conf;
extern ConfRecordValidation<UsrConfKey> usr_conf;

extern AsyncSequencer checklist_boot;    // Hardware state checklist.
extern AsyncSequencer checklist_cyclic;  // Hardware state checklist.

extern SPIAdapter spi0;
extern I2CAdapter i2c0;
extern I2CAdapter i2c1;

extern SSD1331 display;
extern SX8634* touch;
extern ManuvrPMU pmu;

/* Audio objects... */
extern float volume_left_output;
extern float volume_right_output;
extern float volume_pink_noise;
extern float mix_synth_to_fft;
extern float mix_queueL_to_fft;
extern float mix_queueR_to_fft;
extern float mix_noise_to_fft;
extern float mix_synth_to_line;
extern float mix_queue_to_line;
extern float mix_noise_to_line;
extern AudioSynthNoisePink      pinkNoise;
extern AudioSynthWaveformSine   sineL;
extern AudioSynthWaveformSine   sineR;
extern AudioPlayQueue           queueL;
extern AudioPlayQueue           queueR;
extern AudioMixer4              mixerL;
extern AudioMixer4              mixerR;
extern AudioMixer4              mixerFFT;
extern AudioAmplifier           ampR;
extern AudioAmplifier           ampL;
extern AudioAnalyzeFFT256       fft256_1;

#endif    // __MOTHERFLUX0R_H__
