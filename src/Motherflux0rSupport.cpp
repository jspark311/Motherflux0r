#include "Motherflux0r.h"
#include <inttypes.h>
#include <stdint.h>
#include <math.h>
#include <CppPotpourri.h>
#include <Adafruit_SSD1331.h>


extern Adafruit_SSD1331 display;

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
    case SensorID::PSU_TEMP:      return "PSU_TEMP";
    case SensorID::BATT_VOLTAGE:  return "BATT_VOLTAGE";
    case SensorID::LUX:           return "LUX";
  }
  return "UNKNOWN";
}


const char* const getAppIDString(AppID e) {
  switch (e) {
    case AppID::APP_SELECT:       return "APP_SELECT";
    case AppID::TOUCH_TEST:       return "TOUCH_TEST";
    case AppID::CONFIGURATOR:     return "CONFIGURATOR";
    case AppID::DATA_MGMT:        return "DATA_MGMT";
    case AppID::SYNTH_BOX:        return "SYNTH_BOX";
    case AppID::COMMS_TEST:       return "COMMS_TEST";
    case AppID::META:             return "META";
    case AppID::I2C_SCANNER:      return "I2C_SCANNER";
    case AppID::TRICORDER:        return "TRICORDER";
    case AppID::HOT_STANDBY:      return "HOT_STANDBY";
    case AppID::SUSPEND:          return "SUSPEND";
  }
  return "UNKNOWN";
}

const char* const getDataVisString(DataVis e) {
  switch (e) {
    case DataVis::NONE:       return "NONE";
    case DataVis::GRAPH:      return "GRAPH";
    case DataVis::VECTOR:     return "VECTOR";
    case DataVis::COMPASS:    return "COMPASS";
    case DataVis::FIELD:      return "FIELD";
    case DataVis::TEXT:       return "TEXT";
  }
  return "UNKNOWN";
}


void listAllSensors(StringBuilder* output) {
  for (uint8_t i = 0; i < 11; i++) {
    output->concatf("%2u: %s\n", i, getSensorIDString((SensorID) i));
  }
}

void listAllApplications(StringBuilder* output) {
  for (uint8_t i = 0; i < 11; i++) {
    output->concatf("%2u: %s\n", i, getAppIDString((AppID) i));
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
* Given a data array, and parameters for the graph, draw the data to the
*   display.
*/
void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  float* dataset, uint32_t data_len
) {
  if (draw_base) {   // Draw the basic frame and axes?
    display.drawFastVLine(x, y, h, WHITE);
    display.drawFastHLine(x, y+h, w, WHITE);
  }
  display.fillRect(x+1, y, w, h-1, BLACK);
  if (w < (int32_t) data_len) {
    dataset += (data_len - w);
    data_len = w;
  }

  float v_max = 0.0;
  float v_min = 0.0;
  for (uint32_t i = 0; i < data_len; i++) {
    float tmp = *(dataset + i);
    v_max = strict_max(v_max, tmp);
    v_min = strict_min(v_min, tmp);
  }
  float h_scale = data_len / w;
  float v_scale = (v_max - v_min) / h;
  for (uint32_t i = 0; i < data_len; i++) {
    uint8_t tmp = *(dataset + i) / v_scale;
    display.writePixel(i + x, (y+h)-tmp, color);
  }
  if (draw_v_ticks) {
    display.drawFastHLine(x+1, y, 2, WHITE);
    display.setCursor(x+2, y);
    display.setTextColor(WHITE);
    display.print(v_max);
    display.setCursor(x+2, (y+h) - 9);
    display.print(v_min);
  }
  if (draw_h_ticks) {
    float last_datum = *(dataset + (data_len-1));
    uint8_t tmp = last_datum / v_scale;
    //display.fillCircle(x+w, tmp+y, 1, color);
    display.setCursor(x, strict_min((uint16_t) ((y+h)-tmp), (uint16_t) (h-1)));
    display.setTextColor(color);
    display.print(*(dataset + (data_len-1)));
  }
}


/*
* Given a filter object, and parameters for the graph, draw the data to the
*   display.
*/
void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<float>* filt
) {
  const uint16_t DATA_SIZE = filt->windowSize();
  const uint16_t LAST_SIDX = filt->lastIndex();
  const float*   F_MEM     = filt->memPtr();
  float tmp_data[DATA_SIZE];
  for (uint16_t i = 0; i < DATA_SIZE; i++) {
    tmp_data[i] = *(F_MEM + ((i + LAST_SIDX) % DATA_SIZE));
  }
  draw_graph_obj(x, y, w, h, color, draw_base, draw_v_ticks, draw_h_ticks, tmp_data, (uint32_t) DATA_SIZE);
}


/*
* Given a filter object, and parameters for the graph, draw the data to the
*   display.
*/
void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<uint32_t>* filt
) {
  const uint16_t  DATA_SIZE = filt->windowSize();
  const uint16_t  LAST_SIDX = filt->lastIndex();
  const uint32_t* F_MEM     = filt->memPtr();
  float tmp_data[DATA_SIZE];
  for (uint16_t i = 0; i < DATA_SIZE; i++) {
    tmp_data[i] = *(F_MEM + ((i + LAST_SIDX) % DATA_SIZE));
  }
  draw_graph_obj(x, y, w, h, color, draw_base, draw_v_ticks, draw_h_ticks, tmp_data, (uint32_t) DATA_SIZE);
}


/*
* Displays a progress bar that runs left to right.
* @param percent is in the range [0.0, 1.0]
*/
void draw_progress_bar_horizontal(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_val, float percent
) {
  if (draw_base) {   // Clear the way.
    display.fillRect(x, y, w, h, BLACK);
  }
  uint8_t pix_width = percent * (w-2);
  int blackout_x = x+1+pix_width;
  int blackout_w = (w+2)-pix_width;

  display.fillRoundRect(blackout_x, y+1, blackout_w, h-2, 3, BLACK);
  display.fillRoundRect(x+1, y+1, pix_width, h-2, 3, color);
  display.drawRoundRect(x, y, w, h, 3, WHITE);

  if (draw_val && ((h-4) >= 7)) {
    // If we have space to do so, and the application requested it, draw the
    //   progress value in the middle of the bar.
    int txt_x = x+3;
    int txt_y = y+3;
    StringBuilder temp_str;
    temp_str.concatf("%d%%", (int) (percent*100));
    display.setTextSize(0);
    display.setCursor(txt_x, txt_y);
    display.setTextColor(WHITE);
    display.print((char*) temp_str.string());
  }
}


/*
* Displays a progress bar that runs bottom to top.
* @param percent is in the range [0.0, 1.0]
*/
void draw_progress_bar_vertical(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_val, float percent
) {
  if (draw_base) {   // Clear the way.
    display.fillRect(x, y, w, h, BLACK);
  }
  uint8_t pix_height = percent * (h-2);
  int blackout_h = y+(h-1)-pix_height;
  display.fillRoundRect(x+1, y+1, w-2, blackout_h, 3, BLACK);
  display.fillRoundRect(x+1, (y+h-1)-pix_height, w-2, pix_height, 3, color);
  display.drawRoundRect(x, y, w, h, 3, WHITE);

  if (draw_val && ((w-4) >= 15)) {
    // If we have space to do so, and the application requested it, draw the
    //   progress value in the middle of the bar.
    int txt_x = x+2;
    int txt_y = (y+3+h)-pix_height;
    StringBuilder temp_str;
    temp_str.concatf("%d%%", (int) (percent*100));
    display.setTextSize(0);
    display.setCursor(txt_x, txt_y);
    display.setTextColor(WHITE);
    display.print((char*) temp_str.string());
  }
}


/*
*/
void draw_compass(
  int x, int y, int w, int h,
  bool scale_needle, bool draw_val, float bearing_field, float bearing_true_north
) {
  int origin_x = x + (w >> 1);
  int origin_y = y + (h >> 1);
  int maximal_extent = (strict_min((int16_t) w, (int16_t) h) >> 1) - 1;
  const int NEEDLE_WIDTH = maximal_extent >> 3;
  display.fillCircle(origin_x, origin_y, maximal_extent, BLACK);
  display.drawCircle(origin_x, origin_y, maximal_extent, WHITE);
  int displacement_x = cos(bearing_field * (PI/180.0)) * maximal_extent;
  int displacement_y = sin(bearing_field * (PI/180.0)) * maximal_extent;
  int displacement_tri_x = cos((bearing_field + 90.0) * (PI/180.0)) * NEEDLE_WIDTH;
  int displacement_tri_y = sin((bearing_field + 90.0) * (PI/180.0)) * NEEDLE_WIDTH;

  int needle_tip_n_x = displacement_x + origin_x;
  int needle_tip_n_y = displacement_y + origin_y;
  int needle_tip_s_x = (displacement_x * -1) + origin_x;
  int needle_tip_s_y = (displacement_y * -1) + origin_y;
  int needle_x1 = displacement_tri_x + origin_x;
  int needle_y1 = displacement_tri_y + origin_y;
  int needle_x2 = (displacement_tri_x * -1) + origin_x;
  int needle_y2 = (displacement_tri_y * -1) + origin_y;
  //display.drawLine(origin_x, origin_y, needle_tip_s_x, needle_tip_s_y, WHITE);
  //display.drawLine(origin_x, origin_y, needle_tip_n_x, needle_tip_n_y, RED);
  display.fillTriangle(needle_tip_s_x, needle_tip_s_y, needle_x1, needle_y1, needle_x2, needle_y2, WHITE);
  display.fillTriangle(needle_tip_n_x, needle_tip_n_y, needle_x1, needle_y1, needle_x2, needle_y2, RED);
}


/*
* Given a vector object, and parameters for the graph, draw the data to the
*   display. The given vector must be normalized.
*/
void draw_3vector(
  int x, int y, int w, int h, uint16_t color,
  bool draw_axes, bool draw_val, float vx, float vy, float vz
) {
  const int PERSPECTIVE_SCALE = 1;
  int origin_x = x + (w >> 1);
  int origin_y = y + (h >> 1);
  if (draw_axes) {   // Draw the axes? The origin is in the middle of the field.
    display.drawFastVLine(origin_x, y, h, WHITE);
    display.drawFastHLine(x, origin_y, w, WHITE);
    display.drawLine(x, (y+h), w, y, WHITE);
    // Only 1/8 of a cube (all vector components are positive).
    //display.drawFastVLine(x, y, h, WHITE);
    //display.drawFastHLine(x, (y+h), w, WHITE);
    //display.drawLine(x, (y+h), w>>1, y>>1, WHITE);
  }
  // Project the vector onto the x/y plane.
  // To give a sense of depth, we use a triangle where only a line is required.
  // We want the y-axis to be northward on the display. So we have to change the
  //   sign of that component.
  Vector3<int> projected(vx * (w >> 1), vy*(h >> 1) * -1, 0);   // TODO: z is unimplemented
  int x1 = origin_x + projected.x - PERSPECTIVE_SCALE;
  int y1 = origin_y + projected.y;
  int x2 = origin_x + projected.x;
  int y2 = origin_y + projected.y - PERSPECTIVE_SCALE;
  display.fillTriangle(origin_x, origin_y, x1, y1, x2, y2, color);
}


/*
* Draw the data view selector widget.
*/
void draw_data_view_selector(
  int x, int y, int w, int h,
  DataVis opt0, DataVis opt1, DataVis opt2, DataVis opt3, DataVis opt4, DataVis opt5,
  DataVis selected
) {
  uint16_t offset = 0;
  display.setTextSize(0);
  display.drawFastVLine(x, y, h, WHITE);
  display.drawFastHLine(x, y, w, WHITE);
  display.setCursor(x+2, y+2);
  display.setTextColor(BLACK, WHITE);
  display.print("VIS");
  offset += 9;
  display.drawFastHLine(x, y + offset, w, WHITE);

  if (DataVis::NONE != opt0) {
    if (selected == opt0) {
      display.setTextColor(BLACK, YELLOW);
    }
    else {
      display.setTextColor(YELLOW, BLACK);
    }
    display.setCursor(x+2, y+offset+2);
    display.print(*getDataVisString(opt0));
    offset += 10;
    display.drawFastHLine(x, y + offset, w, WHITE);
  }
  if (DataVis::NONE != opt1) {
    if (selected == opt1) {
      display.setTextColor(BLACK, YELLOW);
    }
    else {
      display.setTextColor(YELLOW, BLACK);
    }
    display.setCursor(x+2, y+offset+2);
    display.print(*getDataVisString(opt1));
    offset += 10;
    display.drawFastHLine(x, y + offset, w, WHITE);
  }
  if (DataVis::NONE != opt2) {
    if (selected == opt2) {
      display.setTextColor(BLACK, YELLOW);
    }
    else {
      display.setTextColor(YELLOW, BLACK);
    }
    display.setCursor(x+2, y+offset+2);
    display.print(*getDataVisString(opt2));
    offset += 10;
    display.drawFastHLine(x, y + offset, w, WHITE);
  }
  if (DataVis::NONE != opt3) {
    if (selected == opt3) {
      display.setTextColor(BLACK, YELLOW);
    }
    else {
      display.setTextColor(YELLOW, BLACK);
    }
    display.setCursor(x+2, y+offset+2);
    display.print(*getDataVisString(opt3));
    offset += 10;
    display.drawFastHLine(x, y + offset, w, WHITE);
  }
  if (DataVis::NONE != opt4) {
    if (selected == opt4) {
      display.setTextColor(BLACK, YELLOW);
    }
    else {
      display.setTextColor(YELLOW, BLACK);
    }
    display.setCursor(x+2, y+offset+2);
    display.print(*getDataVisString(opt4));
    offset += 10;
    display.drawFastHLine(x, y + offset, w, WHITE);
  }
  if (DataVis::NONE != opt5) {
    if (selected == opt5) {
      display.setTextColor(BLACK, YELLOW);
    }
    else {
      display.setTextColor(YELLOW, BLACK);
    }
    display.setCursor(x+2, y+offset+2);
    display.print(*getDataVisString(opt5));
    offset += 10;
    display.drawFastHLine(x, y + offset, w, WHITE);
  }
}
