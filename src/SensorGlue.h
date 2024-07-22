#include "TimeSeries/TimeSeries.h"
#include "Pipes/TripleAxisPipe/TripleAxisPipe.h"
#include "Pipes/TripleAxisPipe/TripleAxisCompass.h"
#include "ManuvrDrivers.h"
#include "ICM20948.h"
#include "DRV425.h"

/* Sensor representations */
extern SX1503   sx1503;
extern MCP356x  mag_adc;
extern DRV425 magneto;
extern SX1503 sx1503;
extern TMP102 tmp102;
extern GridEYE grideye;
extern VEML6075 uv;
extern ICM_20948_SPI imu;
extern TSL2561 tsl2561;
extern BME280I2C baro;
extern VL53L0X tof;
extern GPSWrapper gps;
extern LocationFrame current_location;

/* TimeSeriess. These are the memory hogs. */
extern TimeSeries<float> graph_array_pressure;
extern TimeSeries<float> graph_array_humidity;
extern TimeSeries<float> graph_array_air_temp;
extern TimeSeries<float> graph_array_psu_temp;
extern TimeSeries<float> graph_array_uva;
extern TimeSeries<float> graph_array_uvb;
extern TimeSeries<float> graph_array_uvi;
extern TimeSeries<float> graph_array_ana_light;
extern TimeSeries<float> graph_array_visible;
extern TimeSeries<float> graph_array_broad_ir;
extern TimeSeries<float> graph_array_therm_mean;
extern TimeSeries<float> graph_array_therm_frame;
//extern TimeSeries<float> graph_array_mag_confidence;
extern TimeSeries<float> graph_array_mag_strength_x;
extern TimeSeries<float> graph_array_mag_strength_y;
extern TimeSeries<float> graph_array_mag_strength_z;
extern TimeSeries<float> graph_array_time_of_flight;
extern TimeSeries<float> graph_array_batt_voltage;
extern TimeSeries<float> graph_array_batt_current;
extern TimeSeries<float> graph_array_cpu_time;
extern TimeSeries<float> graph_array_frame_rate;

/* Magnetic pipeline */
extern TripleAxisTerminus     mag_vect;   // The magnetic field vector.
extern TripleAxisCompass      compass;    // Tilt-compensated compass.
extern TripleAxisFork         mag_fork;
extern TripleAxisSingleFilter mag_filter; // Input-side filter.

/* Inertial pipeline */
extern TripleAxisTerminus     down;       // Gnomon conversion stage.
extern TripleAxisFork         imu_fork;

int8_t init_sensor_memory();
