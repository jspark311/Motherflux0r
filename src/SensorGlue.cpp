#include "Motherflux0r.h"

/*******************************************************************************
* Globals
* These are extern'd into place by translation units that need them.
* Sloppy. But neatness isn't a value at the top level.
*******************************************************************************/

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
SensorFilter<float> graph_array_broad_ir(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_therm_mean(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_therm_frame(FilteringStrategy::MOVING_AVG, 64, 0);
SensorFilter<float> graph_array_mag_strength_x(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_mag_strength_y(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_mag_strength_z(FilteringStrategy::RAW, 96, 0);
SensorFilter<float> graph_array_time_of_flight(FilteringStrategy::RAW, 96, 0);


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
  //graph_array_mag_confidence.init();
  return 0;
}
