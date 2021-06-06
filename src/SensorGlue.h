#include <SensorFilter.h>
#include <TripleAxisPipe.h>
#include <TripleAxisCompass.h>
#include <ManuvrDrivers.h>
#include "ICM20948.h"
#include "DRV425.h"

/* Sensor representations */
extern DRV425 magneto;
extern TMP102 tmp102;
extern GridEYE grideye;
extern VEML6075 uv;
extern ICM_20948_SPI imu;
extern TSL2561 tsl2561;
extern BME280I2C baro;
extern VL53L0X tof;
extern GPSWrapper gps;
extern LocationFrame current_location;

/* SensorFilters. These are the memory hogs. */
extern SensorFilter<float> graph_array_pressure;
extern SensorFilter<float> graph_array_humidity;
extern SensorFilter<float> graph_array_air_temp;
extern SensorFilter<float> graph_array_psu_temp;
extern SensorFilter<float> graph_array_uva;
extern SensorFilter<float> graph_array_uvb;
extern SensorFilter<float> graph_array_uvi;
extern SensorFilter<float> graph_array_ana_light;
extern SensorFilter<float> graph_array_visible;
extern SensorFilter<float> graph_array_therm_mean;
extern SensorFilter<float> graph_array_therm_frame;
//extern SensorFilter<float> graph_array_mag_confidence;
extern SensorFilter<float> graph_array_mag_strength_x;
extern SensorFilter<float> graph_array_mag_strength_y;
extern SensorFilter<float> graph_array_mag_strength_z;
extern SensorFilter<float> graph_array_time_of_flight;

/* Magnetic pipeline */
extern TripleAxisTerminus     mag_vect;   // The magnetic field vector.
extern TripleAxisCompass      compass;    // Tilt-compensated compass.
extern TripleAxisSingleFilter mag_filter; // Input-side filter.
extern TripleAxisConvention   mag_conv;   // Gnomon conversion stage.

/* Inertial pipeline */
extern TripleAxisTerminus     down;       // Gnomon conversion stage.
extern TripleAxisConvention   tilt_conv;  // Gnomon conversion stage.



int8_t init_sensor_memory();
