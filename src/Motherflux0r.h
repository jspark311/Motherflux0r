#include <inttypes.h>
#include <stdint.h>
#include <CppPotpourri.h>
#include <StringBuilder.h>
#include <SensorFilter.h>
#include <Image/Image.h>
#include <TripleAxisCompass.h>
#include <ManuvrDrivers.h>
#include <Composites/ManuvrPMU/ManuvrPMU-r2.h>

#include "Storage/DataRecords.h"
#include "CommPeer.h"

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
#define WHITE           0xFFFF

// For sanity, we use a common color scheme for 3-space axes.
#define COLOR_X_AXIS    0x80F9
#define COLOR_Y_AXIS    0xF207
#define COLOR_Z_AXIS    0x031F

#define COLOR_BATT_VOLTAGE  0x03F2
#define COLOR_BATT_CURRENT  0x7EF2



/*******************************************************************************
* Data handling flags
*******************************************************************************/
#define DATA_REC_FLAG_               0x80000000   //
#define DATA_RELAY_FLAG_             0x80000000   //



/*******************************************************************************
* Display related flags
*******************************************************************************/
#define GRAPH_FLAG_LOCK_RANGE_V             0x00800000   // Lock the V range.
#define GRAPH_FLAG_TEXT_RANGE_V             0x01000000   // Text overlay for axis values.
#define GRAPH_FLAG_TEXT_VALUE               0x02000000   // Text overlay for current value.
#define GRAPH_FLAG_PARTIAL_REDRAW           0x04000000   // Partial redraw
#define GRAPH_FLAG_FULL_REDRAW              0x08000000   // Full redraw
#define GRAPH_FLAG_DRAW_RULE_H              0x10000000   //
#define GRAPH_FLAG_DRAW_RULE_V              0x20000000   //
#define GRAPH_FLAG_DRAW_TICKS_H             0x40000000   //
#define GRAPH_FLAG_DRAW_TICKS_V             0x80000000   //



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


enum class DataVis : uint8_t {
  NONE          = 0,  // A time-series graph.
  GRAPH         = 1,  // A time-series graph.
  VECTOR        = 2,  // A projected 3-space vector.
  COMPASS       = 3,  // A compass render.
  FIELD         = 4,  // A 2d array.
  TEXT          = 5   // Prefer alphanumeric readout.
};




/*******************************************************************************
* Function prototypes
*******************************************************************************/
void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity = 3500);
void vibrateOn(uint32_t duration, uint16_t intensity = 4095);
void timeoutCheckVibLED();

const char* const getSensorIDString(SensorID);
const char* const getDataVisString(DataVis);
void listAllSensors(StringBuilder*);


float FindE(int bands, int bins);
void  printFFTBins(StringBuilder*);
uint16_t* bitmapPointer(unsigned int idx);

/* Display helper routines */
void render_button_icon(uint8_t sym, int x, int y, uint16_t color);

void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color0, uint16_t color1, uint16_t color2,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<float>* filt0, SensorFilter<float>* filt1, SensorFilter<float>* filt2
);

void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color0, uint16_t color1,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<float>* filt0, SensorFilter<float>* filt1
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

void draw_data_square_field(
  int x, int y, int w, int h,
  uint32_t flags,
  float* range_min, float* range_max,
  SensorFilter<float>* filt
);

void draw_data_view_selector(
  int x, int y, int w, int h,
  DataVis opt0, DataVis opt1, DataVis opt2, DataVis opt3, DataVis opt4, DataVis opt5,
  DataVis selected
);

void draw_3sphere(
  int x, int y, int w, int h,
  bool opaque,
  int meridians, int parallels,
  float euler_about_x, float euler_about_y   // TODO: A quat would be cleaner.
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

#endif    // __MOTHERFLUX0R_H__
