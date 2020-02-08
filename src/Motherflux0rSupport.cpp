#include "Motherflux0r.h"
#include <inttypes.h>
#include <stdint.h>
#include <CppPotpourri.h>

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
