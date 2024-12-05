
//Nov 2024
#ifndef VP44tables
#define VP44tables
#include <Arduino.h>

/********************************** Fuel, MAP and timing tables ***************************************/
//  The following tables have interpolation applied later. Fuel range 0 to 4095. Needs about 1500 to idle.
//  The governor limits fuel to which ever value in the next 2 tables is lowest or the TPS if lower.

extern const int MapFuel[];
extern const int RpmFuel[];
extern const float NoLoadTiming[];
extern const float FullLdTiming[];
extern const int strokeCorrection[];  
extern const int BaseFuel[];        

//default parameters; these kick in if the EEPROM is blank, invalid or re-stored to defaults
//they should be "good enough" to run see VP44tables.cpp for details
extern uint TPS_rest, TPS_floored;
extern int kp, ki, kd, droop, FuelBias;
extern uint16_t duty1, duty2, duty3, duty4;
extern uint32_t deadTime1, deadTime2, deadTime3, releaseTime;
extern int deadTime1Man, deadTime2Man, deadTime3Man, duty3Manual;
extern float amps2Manual, amps4Manual;

#endif