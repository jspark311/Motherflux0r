#include "Motherflux0r.h"

/*******************************************************************************
* Globals
* These are extern'd into place by translation units that need them.
* Sloppy. But neatness isn't a value at the top level.
*******************************************************************************/

/* Data buffers for sensors. */
SensorFilter<float> graph_array_pressure(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_humidity(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_air_temp(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_psu_temp(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_uva(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_uvb(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_uvi(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_ana_light(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_visible(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_broad_ir(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_therm_mean(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_therm_frame(64, FilteringStrategy::MOVING_AVG);
SensorFilter<float> graph_array_mag_strength_x(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_mag_strength_y(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_mag_strength_z(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_time_of_flight(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_batt_voltage(96, FilteringStrategy::RAW);
SensorFilter<float> graph_array_batt_current(96, FilteringStrategy::RAW);


/*
* Perform the massive heap allocations for all the sensor buffers.
*/
int8_t init_sensor_memory() {
  int8_t ret = -1;
  if (0 != graph_array_pressure.init()) {       return ret;   }
  ret--;
  if (0 != graph_array_humidity.init()) {       return ret;   }
  ret--;
  if (0 != graph_array_air_temp.init()) {       return ret;   }
  ret--;
  if (0 != graph_array_psu_temp.init()) {       return ret;   }
  ret--;
  if (0 != graph_array_uva.init()) {            return ret;   }
  ret--;
  if (0 != graph_array_uvb.init()) {            return ret;   }
  ret--;
  if (0 != graph_array_uvi.init()) {            return ret;   }
  ret--;
  if (0 != graph_array_ana_light.init()) {      return ret;   }
  ret--;
  if (0 != graph_array_visible.init()) {        return ret;   }
  ret--;
  if (0 != graph_array_broad_ir.init()) {       return ret;   }
  ret--;
  if (0 != graph_array_therm_mean.init()) {     return ret;   }
  ret--;
  if (0 != graph_array_therm_frame.init()) {    return ret;   }
  ret--;
  if (0 != graph_array_mag_strength_x.init()) { return ret;   }
  ret--;
  if (0 != graph_array_mag_strength_y.init()) { return ret;   }
  ret--;
  if (0 != graph_array_mag_strength_z.init()) { return ret;   }
  ret--;
  if (0 != graph_array_time_of_flight.init()) { return ret;   }
  ret--;
  if (0 != graph_array_batt_voltage.init()) {   return ret;   }
  ret--;
  if (0 != graph_array_batt_current.init()) {   return ret;   }
  ret--;
  //graph_array_mag_confidence.init();
  return 0;
}
