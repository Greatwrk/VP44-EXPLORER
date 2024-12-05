
//Nov 2024
#include "VP44tables.h"

/********************************** Fuel, MAP and timing tables ***************************************/
//  The following tables have interpolation applied later. Fuel range 0 to 4095. Needs about 1500 to idle.
//  The governor limits fuel to which ever value in the next 2 tables is lowest or the TPS if lower.


// Fuel max limit versus Boost. Overboost control too.
const int MapFuel[]{
  //Fuel   PSI
  2500,  //0
  2700,  //2
  2900,  //4
  3200,  //6
  3300,  //8
  3300,  //10
  3300,  //12
  3300,  //14
  3600,  //16
  3700,  //18
  4095,  //20
  4095,  //22
  3600,  //24
  3500,  //26
  3400,  //28
  3300,  //30
  3100,  //32
  3100,  //34
  3000,  //36
  3000,  //38
  2900,  //40
  2800,  //42
  2700,  //44
};

// Fuel max limit versus RPM. Torque management, throttle response, secondary rev limiting.
const int RpmFuel[]{
  //Fuel    RPM
  3600,  //500
  3600,  //600
  2400,  //700
  2400,  //800
  2400,  //900
  2400,  //1000
  2400,  //1100
  2400,  //1200
  2800,  //1300
  2800,  //1400
  3200,  //1500
  3600,  //1600
  3600,  //1700
  3600,  //1800
  3600,  //1900
  3600,  //2000
  3600,  //2100
  3600,  //2200
  3600,  //2300
  3600,  //2400
  3600,  //2500
  3600,  //2600
  3200,  //2700
  3200,  //2800
  3200,  //2900
  2800,  //3000
  2400,  //3100
  2000,  //3200
  1000,  //3300
  0,     //3400
  0,     //3500
  0,     //3600
  0,     //3700
  0,     //3800
  0,     //3900
  0      //4000
};

//  Timing adjustment range is between 9 to 40 degrees approx.
//  Set even lower during starting.
//  These tables simulate a centrifgual advance. Engine load deterines the final value. EX: 2000 RPM 50% load will pick a value
//  halfway between the No load value @ 2000 RPM and Full load value @ 2000 RPM.
const float NoLoadTiming[]{
  //DEG    RPM
  9.0,   //500
  10.0,  //600
  10.0,  //700
  10.0,  //800
  12.0,  //900
  12.0,  //1000
  12.0,  //1100
  12.5,  //1200
  13.8,  //1300
  14.0,  //1400
  15.0,  //1500
  15.7,  //1600
  16.2,  //1700
  16.9,  //1800
  17.6,  //1900
  18.2,  //2000
  18.4,  //2100
  18.6,  //2200
  18.8,  //2300
  19.0,  //2400
  19.0,  //2500
  19.5,  //2600
  20.0,  //2700
  21.0,  //2800
  22.0,  //2900
  23.5,  //3000
  23.7,  //3100
  24.2,  //3200
  24.7,  //3300
  24.9,  //3400
  25.1,  //3500
  25.6,  //3600
  26.0,  //3700
  27.0,  //3800
  28.0,  //3900
  29.0   //4000
};

const float FullLdTiming[]{
  //DEG    RPM
  9.0,   //500
  9.0,   //600
  9.0,   //700
  9.0,   //800
  10.0,  //900
  10.0,  //1000
  10.0,  //1100
  10.0,  //1200
  11.0,  //1300
  12.0,  //1400
  13.0,  //1500
  14.0,  //1600
  14.6,  //1700
  14.6,  //1800
  15.0,  //1900
  15.0,  //2000
  15.0,  //2100
  15.0,  //2200
  15.2,  //2300
  15.5,  //2400
  16.0,  //2500
  16.0,  //2600
  16.5,  //2700
  17.0,  //2800
  17.5,  //2900
  18.0,  //3000
  18.5,  //3100
  18.7,  //3200
  18.9,  //3300
  19.0,  //3400
  19.3,  //3500
  19.6,  //3600
  20.0,  //3700
  21.0,  //3800
  22.5,  //3900
  23.0,  //4000
};

//  Fuel 0 thru 4095 is injection duration in degrees which is linear, however the lobe on the cam ring is not linear-
//  it's close to a sine wave. This table allows to determine the actual amount of fuel injected, which is basically load.
//  this can also be used to compute the actual quanity of fuel in mL. For fuel rate and mileage
const int strokeCorrection[]{
  //Fuel  % of full stroke
  0,     //0
  942,   //12.5
  1365,  //25
  1718,  //37.5
  2048,  //50
  2378,  //62.5
  2730,  //75
  3154,  //87.5
  4095   //100
};

// After the fuel value is sent thru strokeCorrection, use this to determine the no load for timing
// this is how much fuel is needed at listed RPMs in neutral.  
const int BaseFuel[]{
//ml      RPM  
  1525,   //1000
  1562,   //1200
  1600,   //1400
  1670,   //1600
  1725,   //1800
  1820,   //2000
  1875,   //2200
  1940,   //2400
  2060,   //2600
  2150,   //2800
  2165,   //3000
  2250,   //3200
  2340,   //3400
  2440,   //3600
  2550,   //3800
  2650,   //4000
};

//default parameters these kick in if the EEPROM is blank, invalid or re-stored to defaults
//they should be "good enough" to run`
uint TPS_rest = 245, TPS_floored = 2200;
int kp = 250, ki = 12, kd = 200, droop = 150, FuelBias = 1590;
uint16_t duty1 = 600, duty2 = 400, duty3 = 150, duty4 = 350;
uint32_t deadTime1 = 550, deadTime2 = 250, deadTime3 = 30, releaseTime = 825;
int deadTime1Man = 100, deadTime2Man = 0, deadTime3Man = 30, duty3Manual = 150;
float amps2Manual = 11.0, amps4Manual = 9.0;

// end of VP44 tables