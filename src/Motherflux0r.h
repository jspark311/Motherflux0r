#include <inttypes.h>
#include <stdint.h>
#include <CppPotpourri.h>
#include <StringBuilder.h>
#include <SensorFilter.h>

#ifndef __MOTHERFLUX0R_H__
#define __MOTHERFLUX0R_H__

#define TEST_PROG_VERSION          "v1.4"
#define TOUCH_DWELL_LONG_PRESS       1000  // Milliseconds for "long-press".
#define E_VAL                      1.0184

/*******************************************************************************
* Pin definitions and hardware constants.
*******************************************************************************/
#define GPS_TX_PIN           0   // Teensy RX
#define GPS_RX_PIN           1   // Teensy TX
#define DRV425_GPIO_IRQ_PIN  2
#define DRV425_ADC_IRQ_PIN   3
#define DRV425_CS_PIN        4
#define VIBRATOR_PIN         5
#define TOUCH_IRQ_PIN        6
#define DAC_DIN_PIN          7
#define PSU_SX_IRQ_PIN     255   // Presently unused.
#define TSL2561_IRQ_PIN    255 // 9
#define DISPLAY_CS_PIN      10
#define SPIMOSI_PIN         11
#define SPIMISO_PIN         12
#define SPISCK_PIN          13
#define LED_R_PIN           14
#define LED_G_PIN           15
#define SCL1_PIN            16   // Sensor service bus.
#define SDA1_PIN            17   // Sensor service bus.
#define SDA0_PIN            18
#define SCL0_PIN            19
#define DAC_LCK_PIN         20   // XMT is tied high.
#define DAC_BCK_PIN         21   // FLT, FMT are tied low.
#define ANA_LIGHT_PIN       22   // PIN_A8
#define DAC_SCL_PIN         23
#define COMM_RX_PIN         24
#define COMM_TX_PIN         25
#define DISPLAY_DC_PIN      26
#define MIC_ANA_PIN         27   // A16
#define TOUCH_RESET_PIN     28
#define IMU_IRQ_PIN         29
#define AMG8866_IRQ_PIN     255 // 30
#define IMU_CS_PIN          31
#define DISPLAY_RST_PIN     32
#define LED_B_PIN           33



/* Common 16-bit colors */
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF


/*******************************************************************************
* Types
*******************************************************************************/
enum class AppID : uint8_t {
  APP_SELECT   =  0,  // For choosing the app.
  TOUCH_TEST   =  1,  // For diagnostics of the touch pad.
  CONFIGURATOR =  2,  // For tuning all the things.
  DATA_MGMT    =  3,  // For managing recorded datasets.
  SYNTH_BOX    =  4,  // Sound synthesis from data.
  COMMS_TEST   =  5,  // Connecting to the outside world.
  META         =  6,  // Shutdown/reboot/reflash, profiles.
  I2C_SCANNER  =  7,  // Tool for non-intrusively scanning foreign i2c buses.
  TRICORDER    =  8,  // This is the primary purpose of the device.
  HOT_STANDBY  =  9,  // Full operation with powered-down UI elements.
  SUSPEND      = 10   // Minimal power without an obligatory reboot.
};

enum class SensorID : uint8_t {
  BARO          = 0,  //
  MAGNETOMETER  = 1,  //
  IMU           = 2,  //
  LIGHT         = 3,  //
  MIC           = 4,  //
  UV            = 5,  //
  GPS           = 6,  //
  THERMOPILE    = 7,  //
  PSU_TEMP      = 8,  // TMP102
  BATT_VOLTAGE  = 9,  //
  LUX           = 10  //
};


enum class DataVis : uint8_t {
  NONE          = 0,  // A time-series graph.
  GRAPH         = 1,  // A time-series graph.
  VECTOR        = 2,  // A projected 3-space vector.
  COMPASS       = 3,  // A compass render.
  FIELD         = 4,  // A 2d array.
  TEXT          = 5   // Prefer alphanumeric readout.
};


/* Struct for tracking application state. */
// TODO: This should evolve into a class if we irradiate millions of copies of it.
typedef struct {
  const char* const title;           // Name of tha application.
  const AppID       id;              // ID of the application.
  uint8_t           page_count;      // Total page count.
  uint8_t           page_top;        // The currently visible page.
  uint8_t           slider_val;      // Cached slider value.
  uint8_t           frame_rate;      // App's frame rate.
  bool              screen_refresh;  // Set to indicate a refresh is needed.
  bool              app_active;      // This app is active.
  bool              locked;          // This app is locked into its current state.
} AppHandle;

/* Struct for defining global hotkeys. */
typedef struct {
  uint8_t id;        // Uniquely IDs this hotkey combo.
  uint8_t buttons;   // To trigger, the button state must equal this...
  uint32_t duration; // ...for at least this many milliseconds.
} KeyCombo;


/*******************************************************************************
* Function prototypes
*******************************************************************************/
void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity = 3500);
void vibrateOn(uint32_t duration, uint16_t intensity = 4095);
void timeoutCheckVibLED();

const char* const getAppIDString(AppID);
const char* const getSensorIDString(SensorID);
const char* const getDataVisString(DataVis);
void listAllSensors(StringBuilder*);
void listAllApplications(StringBuilder*);

float FindE(int bands, int bins);
void  printFFTBins(StringBuilder*);
uint8_t* bitmapPointer(unsigned int idx);

/* Display helper routines */
void render_button_icon(uint8_t sym, int x, int y, uint16_t color);

void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  float* dataset, uint32_t data_len
);

void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<float>* filt
);

void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<uint32_t>* filt
);

void draw_progress_bar_horizontal(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_val, float percent
);

void draw_progress_bar_vertical(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_val, float percent
);

void draw_compass(
  int x, int y, int w, int h,
  bool scale_needle, bool draw_val, float bearing_field, float bearing_true_north
);

void draw_3vector(
  int x, int y, int w, int h, uint16_t color,
  bool draw_axes, bool draw_val, float vx, float vy, float vz
);

void draw_data_view_selector(
  int x, int y, int w, int h,
  DataVis opt0, DataVis opt1, DataVis opt2, DataVis opt3, DataVis opt4, DataVis opt5,
  DataVis selected
);

#define ICON_CANCEL    0
#define ICON_ACCEPT    1
#define ICON_THERMO    2
#define ICON_IMU       3
#define ICON_GPS       4
#define ICON_LIGHT     5
#define ICON_UVI       6
#define ICON_SOUND     7
#define ICON_RH        8
#define ICON_MIC       9
#define ICON_MAGNET   10
#define ICON_BATTERY  11


#endif    // __MOTHERFLUX0R_H__
