
#include <EEPROM.h>
#include <IWatchdog.h>

// VP44 explorer
//  ver 1.0
//  by Ray Malcuit 
//  October 2024
// pump controller. Predict SOI time by kinematics -> pump shaft velocity, acceleration method
// arm core nucleo 64 pin STM32f303RE board
// requires STM32duino core, NOT Maple core. Not for cube or HAL.
// higher resolution timer 2 is now 0.5 microsecond per tick.
// Sensors ADC now with DMA, less CPU overhead
// Phase A captured by Timer 8 now.
// Diagnostic trouble codes added
// Internal watchdog timer added
// Zero cross thresholds stored in emulated EEprom.
// DTC stored
// Added keyed igniton shutdown and save.
// Rapid injection amps adjustment based on battery voltage.
// Inj sol deadtime based on self induced voltage
// Limp home added
// Basic Fuel, LOAD, MAP, and Timing tables added
// Injection line delay correction added
// Much improved idle speed governor

/********************************** Fuel, MAP and timing tables ***************************************/
//  36 table entries each. The following tables have interpolation applied later. Fuel range 0 to 4095. Needs about 1500 to idle.
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
  18.7,   //3200
  18.9,   //3300
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
const int LoadCorrection[]{
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

//Firimg order
const int FiringOrd[]{ 0, 1, 5, 3, 6, 2, 4 };

/*
   Pin assignments
   PA0 inj drive out timer2
   PA1 phase B trigger in
   PA2 USART2 TX to ST-link serial to USB converter
   PA3 USART2 RX ^
   PA6 OPAMP2 out inj. sol. monitor blip
   PA8 Cam position sensor, early 24 valve engines
   PA9 Crankshaft position sensor into timer2 ,early 24 valve engines
   PA10 solenoid current capture into timer2 ch 4 from OPAMP2 out
   PA11 idle validation in IDLE = 5V, pedal pressed = 0V
   PB0 OPAMP3 + in adaptive threshold from PC6
   PB1 OPAMP3 out wired to PC9 Testpoint 2
   PB2 OPAMP3 - in phase A from speed sensor
   PB6 timing control valve drive out (PWM)
   PB7 Tachometer square wave out RPM * 2 to PCM
   PB10 OPAMP4 - in: phase B from speed sensor
   PB11 OPAMP4 + in: adaptive threshold from PC8
   PB12 OPAMP4 out wired to PA1 Testpoint1
   //PB13 inj solenoid drive current to ADC3 ch5 in
   PB14 Injection solenoid monitor OPAMP2 +in
   PB15 O'scope debug out
   PC0 TPS
   PC1 MAP
   PC2 Phase A ADC in
   PC3 Injection solenoid -
   PC4 Phase B ADC in
   PC5 OPAMP2 - in blip detector and ADC2 ch 11 current in
   PC6 PWM sol drive
   PC7 Ignition on sens
   PC8 spare
   PC9 Phase A trigger in
   PC10 USART3 TX to CCD bus future use
   PC11 USART3 RX to CCD bus future use
   PC12 Fuel system relay drive
*/

#define ON 0X3110;         //Timer 2 compare Ch. 1 register pin states on match high
#define OFF 0X3120;        // TIM2->CCMR1 = off, pin goes low when match occurs...
#define NO_CHANGE 0X3100;  //cause interrupt only pin remains the same
#define FORCE_LOW 0x3140;  //forces tim2 ch1 pin low
#define _50HZ 1440;        //TIM16 prescaler
#define _60HZ 1200;        //

uint16_t duty1 = 600, duty2 = 400, duty3 = 150, duty4 = 350;
uint32_t deadTime1 = 550, deadTime2 = 250, deadTime3 = 30, releaseTime = 825, fuelGov2 = 2000, LastTimeout;
int deadTime1Man = 100, deadTime2Man = 0, deadTime3Man = 30, bounceTime = 1100, correction;
int32_t AmpErr, PWAd, PWBd;
volatile uint32_t phaseA[40];
volatile uint32_t phaseB[40], phaseBpredict[40], tempB;
volatile uint32_t BLIPtime[10], apex, predict_apex, TrefTime;
volatile uint32_t edge19Micro, TrefMicro, lastA, lastB;
uint16_t timingDuty;
volatile int triggered, cycle, oldCycle, okToInj, injMalfunction;
volatile int phaseA_edge, phaseB_edge, old_phaseB_edge, CKPsync, CamSync, CKP_cnt1, BLIPcnt;
volatile uint32_t velocity, oldVelocity, SOE, SOI, EOI, RPM, RPMavg;
volatile uint16_t velocityA, oldVelocityA, temp;
volatile int16_t accelA;
volatile int accel, event, predictErr, VPsync, timeout;
int failsafe, TrefEdge, deadTime1Bat, DTC, storedDTC, TimingRetrys;
int TCerror, lastTC, TimError, active_cyl, errorP, errorI, errorD, oldErrorP;
int TPS, MAP, SOL, idle_valid, crank = 2, duty3Manual = 150;
float amps2Manual = 11.0, amps4Manual = 10.0;
float Timing, Timing2, oldTiming, desiredTim, desiredTimNL, desiredTimFL, TerrorP, amps, amps1;
float TPSv, MAPv, BATv, AMPS, ampErr, Load;
uint16_t AD2result[6], AD1result[6];  //ADC2 global var

int amps_ref1;
int amps_ref2;
int amps_hold1;
int amps_hold2;
int duty4AutoAdj;
uint16_t RevCounter, lastGoodThrA, lastGoodThrB, lastGoodSTThrA, lastGoodSTThrB, checksum;
bool eepromValid, Limp, WasIWDreset, Amps_Reduced;
int RpmFuelLimit = 4095, MapFuelLimit = 4095, BOOST;
int fuelp, TPS_scale, TPS_rest, TPS_floored;
int ki = 8, kd = 1, droop = 150, fueld;
int kp = 20, FuelBias = 1575;
int TempInt, coldStart;

//HardwareTimer MainTim8 = HardwareTimer(TIM8);  //this instance is required for attachinterrupt and ISR handler to work-
// Timer 8 is 16 bits wide but has no default pins so pins have to be remapped as ALT FUNC.

/****** Get the factory calibration values from system memory to improve the internal temperature sensor accuracy****/
uint32_t VREF_INTERNAL = 0x1FFFF7BA;
uint16_t Vrefi = *((uint32_t *)(VREF_INTERNAL));
uint32_t TEMP30 = 0X1FFFF7B8;
uint16_t temp30 = *((uint32_t *)(TEMP30));
uint32_t TEMP110 = 0x1FFFF7C2;
uint16_t temp110 = *((uint32_t *)(TEMP110));
float TS = 1710;

void setup() {
  // Arduino core may set these registers as needed, But also added here for some possible low level uses.
  RCC->AHBENR |= 0x300E0003;  //Clock enable ADC 1,2,3,4 GPIO ports A,B,C and DMA1,2
  //RCC->APB1ENR |= 0x24000007;   //Turn on some clocks, DAC1,2 Timer 2,3,4 enabled
  RCC->APB1ENR |= 0x26000007;  //Turn on some clocks, DAC1,2 Timer 2,3,4 enabled   *** CAN enabled
  RCC->APB2ENR |= 0x00027001;  //Clock enable, Tim8, Tim16, SPI1, USART1, SYSCFGEN (required by COMP)

  if (IWatchdog.isReset()) {
    WasIWDreset = 1;         //was it a POR or watchdog reset, used for Diagnostics
    IWatchdog.clearReset();  //clear the reset flag
  }
  IWatchdog.begin(1000000);  //internal watchdog resets CPU after 1 second if execution freezes or go off track

  Serial.begin(115200);  //start serial PA2 TX. PA3 RX. to st-Link which is USB converter
  delay(100);             //let USB get going
  if (WasIWDreset) {
    //Serial.println(" DTC P1688 ");  //internal fuel pump controller failure
    //delay(10);
  }

  pinMode(PA1, INPUT);    //b phase in
  pinMode(PA11, INPUT);   //idle validation in
  pinMode(PB3, INPUT);    //phase A in
  pinMode(PB7, OUTPUT);   //Tachometer out
  pinMode(PB15, OUTPUT);  //O'scope debug out
  pinMode(PC7, INPUT);    //Ignition key on sens
  pinMode(PC13, INPUT);   //user push button sw on nucleo board

  //******** Speed sensor and injection valve events: timer 2 is fixed at 0.5 uS per tick, capture tooth wheel edges on CH 2 ******
  //********** Main Timer 2 setup************
  HardwareTimer *MainTim2 = new HardwareTimer(TIM2);  //this struc is required for attachinterrupt and ISR handler to work-
  //HardwareTimer MainTim2 = HardwareTimer(TIM2);
  //because arduino ISRs can have user named functions
  //could possibly be done with low level - NVIC controller
  // Timer 2 is 32 bits wide but has no default pins so pins have to be remapped as ALT FUNC.

  //                  Timer 2 see datasheet table 14
  GPIOA->AFR[0] |= 0x00000011;  //AF1; PA0 Timer 2 ch 1, PA1 ch 2
  GPIOA->AFR[1] |= 0x00000AA0;  //AF10; PA9 Timer 2 ch 3, PA10 ch 4
  GPIOA->MODER |= 0x00280F0A;   //PA0, PA1 and PA9, PA10 pins set mode alt function.

  TIM2->CR1 = 0X0004;  //INTERNAL CLK, UPCOUNTER, UEV INTERRUPT ENABLED
  TIM2->PSC = 35;      //0.5 micro sec per tick
  TIM2->ARR = 400000;  //200 mS overflows below 100 RPM
  TIM2->CNT = 0;
  TIM2->CCER = 0;        //some timer registers are only writeable when this is disabled
  TIM2->CCMR1 = 0x3130;  //T2C1 force comp out low. T2C2 input capture. WAS 40
  TIM2->CCMR2 = 0X3131;  //input capture ch 3 N8 filter , ch 4 capture
  //the two previous steps have to be done before enabling the channels in the next register below
  TIM2->CCER = 0X3BB1;  // ch1 output compare to pin PA0, ch2 either edge, ch4 capture - edge, ch3 capture any edge was13B1
  TIM2->CCR1 = 400001;  //COMPARE TIMER TO 400001 micro seconds un-reachable

  //****** timer2 interrupts enable *********
  MainTim2->attachInterrupt(T2_OVF);        //Tim2 timeout 200 mS. about 100 RPM
  MainTim2->attachInterrupt(1, T2C1_comp);  //Solenoid driver pin and DAC trigger. PA0
  MainTim2->attachInterrupt(2, T2C2_capt);  //VP speed sensor phase B
  MainTim2->attachInterrupt(3, T2C3_capt);  //Crank Pos sensor capture PA9
  MainTim2->attachInterrupt(4, T2C4_capt);  //solenoid current 'BLIP' detector

  //MainTim2.attachInterrupt(T2_OVF);        //Tim2 timeout 200 mS. about 100 RPM
  //MainTim2.attachInterrupt(1, T2C1_comp);  //Solenoid driver pin and DAC trigger. PA0
  //MainTim2.attachInterrupt(2, T2C2_capt);  //VP speed sensor phase B
  //MainTim2.attachInterrupt(3, T2C3_capt);  //Crank Pos sensor capture PA9
  //MainTim2.attachInterrupt(4, T2C4_capt);  //solenoid current 'BLIP' detector

  //*******start timer 2 **********
  TIM2->CR1 |= 0X0001;  //start TIMER2

  //******** Speed sensor phase A timer 8 is fixed at 0.5 uS per tick, capture tooth wheel edges on CH 4 ******
  //********** Main Timer 8 setup************
  HardwareTimer *MainTim8 = new HardwareTimer(TIM8);  //this instance is required for attachinterrupt and ISR handler to work-
// Timer 8 is 16 bits wide but has no default pins so pins have to be remapped as ALT FUNC.
// HardwareTimer *MainTim8 = new HardwareTimer(TIM8);   //this instance is required for attachinterrupt and ISR handler to work-
  // Timer 8 is 16 bits wide but has no default pins so pins have to be remapped as ALT FUNC.
  //                  Timer 8 see datasheet table 14
  GPIOC->AFR[1] |= 0x00000040;  //AF4; PC9 Timer 8 ch 4
  GPIOC->MODER |= 0x00080000;   //PC9 pin set mode alt function.

  TIM8->CR1 = 0X0004;    //INTERNAL CLK, UPCOUNTER, UEV INTERRUPT ENABLED
  TIM8->PSC = 35;        //0.5 micro sec per tick
  TIM8->ARR = 65535;     //16 bits 2^16 - 1
  TIM8->CCER = 0;        //some timer registers are only writeable when this is disabled
  TIM8->CCMR1 = 0x0000;  //T2C1 force comp out low. T2C2 input capture. WAS 40
  TIM8->CCMR2 = 0X3100;  // ch 4 capture N8
  //the two previous steps have to be done before enabling the channels in the next register below
  TIM8->CCER = 0XB000;  // ch1 output compare to pin PA0, ch2 either edge, ch4 capture any edge, ch3 capture any edge was13B1
  TIM8->BDTR = 0x8000;  //Timer 8 is more advanced, requires MOE master output enable set.

  //****** timer8 interrupts enable *********
  MainTim8->attachInterrupt(4, T8C4_capt);  //Phase A trigger capture

  //*******start timer 8 **********
  TIM8->CR1 |= 0X0001;  //start TIMER8

  //***** TIMER 3 SETUP FOR 24 KHZ PWM ON CH 1, 3 *********
  //******* Timer 3 setup ********

  GPIOC->AFR[0] |= 0x02000000;  //AF2; PC6 mapped to Timer 3 ch 1 compare, speed sensor phase A threshold
  GPIOC->AFR[1] |= 0x00000002;  //AF2; PC8 mapped to Timer 3 ch 3 compare, speed sensor phase B threshold
  GPIOC->MODER |= 0x00022000;   //port pins selected as alternate functions

  TIM3->CR1 = 0X0004;    // INTERNAL CLK, UEV INT ENABLE, UPCOUNT.
  TIM3->PSC = 0;         //prescaler
  TIM3->ARR = 600;       // sets up 120 KHZ PWM
  TIM3->CCMR1 = 0x0060;  // PWM out to ch1
  TIM3->CCMR2 = 0x0060;  // PWM out to ch3
  TIM3->CCER = 0X0101;   // ACTIVE high OUTPUTs ON T3 ch1/ch3 PIN CN10 PC6/PC8 on Nucleo
  TIM3->CCR1 = 200;      //Duty Cycle phase A; just a starting value. Changed later
  TIM3->CCR3 = 200;      //Duty Cycle phase B; just a starting value. Changed later
  TIM3->CR1 |= 0X0001;   //start TIMER3

  //*********** Timer16 setup. PWM channel 1 drives timing piston control valve *****
  //*********** 50 or 60 HZ output mapped to PB6 ************
  //Chanel TIM16_CH1N chosen so PB8 is resevered for CAN_RX future use.

  GPIOB->AFR[0] |= 0x01000000;  //PB6 TIM16 ch1N
  GPIOB->MODER |= 0x00002000;   //PB6 PWM out
  TIM16->CR1 = 0x0004;          // INTERNAL CLK, UEV INT ENABLE, UPCOUNT.
  TIM16->PSC = 1440;            // prescaler 72MHZ / 1440
  TIM16->ARR = 1000;            // sets up 50 60 HZ PWM 100% duty cycle= 1000
  TIM16->CCMR1 = 0x0060;        // PWM out to ch1 PB6
  TIM16->CCER = 0x0004;         // ACTIVE high OUTPUTs
  TIM16->CCR1 = 500;            // DUTY CYCLE, Just a starting value. Changed later
  TIM16->BDTR = 0x8000;         //Timer 16 is more advanced, requires MOE master output enable set.
  TIM16->CR1 |= 0x0001;         // start TIMER16

  //************ Cam position sensor to PA8 setup *******
  pinMode(PA8, INPUT);
  attachInterrupt(digitalPinToInterrupt(PA8), Camshaft_handler, CHANGE);

  //********** set up analog systems *******
  ADC1_setup();
  ADC2_setup();
  DAC_set();
  OPAMP2_set();
  OPAMP3_set();
  OPAMP4_set();
  delay(1);

  if (AD2result[0] > 750) {  //If solenoid has battery over 3.5V then fuel system relay fault
    DTC = 1285;              //should not be turned on yet.
    Serial.print(" DTC P");
    Serial.println(DTC);
  }
  //fuel solenoid will be ruined if left on too long. WARNING! do not defeat the failsafe relay funtion.
  pinMode(PC12, OUTPUT);     // fuel system relay drive also failsafe shutdown incase software or hardware malfunction
  digitalWrite(PC12, HIGH);  // Turn on fuel system relay
  delay(200);                // so the relay has time to engage
  // get data previosly stored in emmulated EEprom (flash last page)
  //  zero crossing circuits starting values
  EEPROM.get(200, lastGoodThrA);
  EEPROM.get(202, lastGoodThrB);
  EEPROM.get(204, storedDTC);
  EEPROM.get(208, checksum);
  EEPROM.get(212, TPS_rest);
  EEPROM.get(216, TPS_floored);
  EEPROM.get(220, kp);
  EEPROM.get(224, ki);
  EEPROM.get(228, droop);
  EEPROM.get(232, FuelBias);
  //if (checksum = lastGoodThrA + lastGoodThrB) {
  Serial.println(TPS_rest);
  Serial.println(TPS_floored);
  Serial.println(kp);
  //Serial.print(" Stored DTC ");
  //Serial.println(storedDTC);
  eepromValid = 1;
  // }

  pinMode(PC12, INPUT);  //fuel system relay should latch on by hardware now. goto high Z state
  Serial.println("start");
  //CANsetup();       //config the CAN periph
  if (tempCalcInternal(AD1result[0]) < 60) { coldStart = 1; }

}  //setup done

//*********** ISR handlers start here *************************
extern "C" void CAN_RX1_IRQHandler(void) {  //STM32duino does not support CAN. Using CMISS LL
  //Serial.println(" isr fired ");
  CANrec();
}

void T8C4_capt(void)              //phaseA_handler(void)   //********** vp tooth wheel sensor phase A signal input******
{                                 //record all the tooth edge times as they pass by sensor A
  if (TIM2->CNT - lastA > 100) {  //50 uS noise filter
    phaseA_edge++;
    //GPIOB->BSRR = 0X8000;    //PB15 sync to O-scope goes high
    phaseA[phaseA_edge] = TIM8->CCR4;  //Timer2 ran out of capture channels so using TIM8
    lastA = TIM2->CNT;

    if (phaseA_edge > 4 && phaseA_edge < 34) {  //ignore the index tooth region when calculating the following
      //Not fractional degrees per microsecond that will require a lot of long math.
      //velocity and accel expressed as the TIME (in microseconds) per 6 degrees of rotation
      //calculated on + and - edges so velocity is updated every 6 degrees
      //pulses are not guaranteed to be 50% duty cycle so use period
      velocityA = phaseA[phaseA_edge] - phaseA[phaseA_edge - 2];  // steady velocity (current period)
      velocityA = (velocityA + oldVelocityA) / 2;                 //rolling average velocity

      accelA = (accel + (velocityA - oldVelocityA)) / 2;  //average acceleration as the diff in velocities
      oldVelocityA = velocityA;

      int cnt1 = phaseB_edge + 2;  //predict the next B edge time from the previous known B time based on leading phase A
      int cnt2 = phaseB_edge + 1;  //velocity and accelleration, predict error is calculated later and used to improve prediction
      phaseBpredict[cnt1] = phaseB[phaseB_edge] + velocityA + accelA + predictErr;
      phaseBpredict[cnt2] = phaseB[phaseB_edge - 1] + velocityA + accelA + predictErr;

      while (cnt1 < 22)  //compute some remaining Phase B edge times from leading edge A
      //math has time to complete between the lag of B from A
      {
        phaseBpredict[cnt1 + 2] = phaseBpredict[cnt1] + velocityA + accelA / 2;
        phaseBpredict[cnt2 + 2] = phaseBpredict[cnt2] + velocityA + accelA / 2;
        cnt1 += 2;
        cnt2 += 2;
      }
      SOI = phaseBpredict[19 + crank];  //we want SOI to start on tooth edge 19 or edge 21 if starting engine
    }

    //**************** SOE - Start Of Energize the Injection solenoid **********
    //SOI start of injection, EOI end of injection
    if (VPsync && event == 0 && phaseA_edge > 7 && okToInj > 1) {
      SOE = SOI - (deadTime1 * 2 + deadTime2 * 2 + deadTime3 * 2);  //copensate for the sol. plunger inertia (dead time)
      SOE = SOE - correction * 2;
      SOE = constrain(SOE, TIM2->CNT + 32, SOE);  //incase the counter just passed SOE
      TIM2->CCR1 = SOE;                           //chan 1 compare match value <think> set alarm time on stopwatch
      TIM2->CCMR1 = ON;                           //T2C1 pin active high on match
    }

    //*************** End Of Injection ************

    if (phaseA_edge >= 17 && event < 5) {
      //4095 units = 48 degrees max fuel duration. 4096 ish / 512 units per tooth width (H+L) period = 6 degrees
      //Ex: if fuelGov2 command  = 1060 and shaft velocity (tooth pulse period) is 2mS wide = 1000 RPM
      //1060/512 = 2.07 tooth width X shaft vel. of 2000 microseconds = 4140 microseconds injector on time, 12.42 degrees
      EOI = ((float)fuelGov2 / 512) * velocityA;  //length of inj. not absolute end time
      if (EOI > releaseTime * 2) {
        EOI = EOI - (releaseTime * 2);  //turn off solenoid early because it takes time for it to move -- release time
        okToInj = 1;
      } else {
        EOI = 0;  //caution EOI is type unsigned long.
        okToInj = -1;
      }

      if (event == 4) {
        //EOI = constrain(EOI, TIM2->CNT + 16, EOI); //incase match was < or = and counter passed it
        //TIM2->CCR1 = SOI + EOI;
        if (SOI + EOI <= TIM2->CNT) {
          TIM2->CCR1 = TIM2->CNT + 32;
        } else {
          TIM2->CCR1 = phaseB[19] + EOI;        //offset "stopwatch alarm" EOI so it happens relative to SOI which was at the time of -
          if (phaseB[19] + EOI <= TIM2->CNT) {  //tooth edge 19 passing under speed sensor signal phase B.
            TIM2->CCR1 = TIM2->CNT + 32;
          }
        }
        TIM2->CCMR1 = OFF;  //T2C1 pin active low on match
      }
    }

    if (phaseA_edge > 38) {  //restart tooth edge counter
      phaseA_edge = 0;
    }
  }
}  //T8C4_capt done

void T2C2_capt(void)  //********** vp tooth wheel sensor phase B signal input******
{                     //record all the tooth edge times as they pass by sensor B
  //tooth wheel input ISR handler from speed - position sensor
  //important! do all this stuff in interrupt handler so critical events don't get missed

  if (TIM2->CNT - lastB > 100)  //50 uS noise filter
  {
    phaseB_edge++;

    if (phaseB_edge == 19) {
      GPIOB->BRR = 0X8000;  // PB15 off - fast method scope trigger
      //edge19Micro = micros();   //for 2nd method timing calculation
    }

    lastB = TIM2->CNT;
    phaseB[phaseB_edge] = TIM2->CCR2;  //record the time of this edge, <think> Lap button on stopwatch

    //************ index tooth detector ********

    // when    PA1 is high  AND    PC9 is low: this is an index tooth
    if (GPIOA->IDR & 0x0002 && !(GPIOC->IDR & 0x0200))  // when phase B(PA1) goes high and phase A(PC9) is low at the same time then trigger
    {                                                   //init a new cycle, <think> reset the stopwatch
      GPIOB->BSRR = 0X8000;                             //PB15 sync to O-scope goes high
      TIM2->EGR = 0x0001;                               // reset timer 2 cntr and all capture, compare events
      phaseA_edge = 0;
      if (phaseB_edge == 36) {
        VPsync = 1;
      } else {
        VPsync = 0;
        Serial.println(" sync loss ");
      }
      phaseB_edge = 0;
      phaseB[0] = 0;
      SOE = 400001;
      SOI = 400001;  //set timer2 compares to un-reachable numbers
      EOI = 400001;
      cycle++;
      timeout = 0;
      if (okToInj > 1 && event != 5) {  //should never be anything except 5 timer2 match events happened when index tooth occurs
        injMalfunction++;
      }
      event = 0;
      BLIPcnt = 0;
      okToInj += 1;
    }


    if (phaseB_edge > 38) {             //edge counter normally does not get this high while engine is running
      phaseB[0] = phaseB[phaseB_edge];  //typicaly 36 edges
      phaseB_edge = 0;
    }
  }

}  //tooth edge ISR done

void T2C1_comp(void)  //****** output compare times used to drive injector solenoid *********
{
  // comes here on Timer2 Chan1 matches <think> alarm clock
  //compare output pin: solenoid pull-in driver
  //T2C1 connected to MOSFET gate driver
  //turn on hard for pull in.
  // T2C1 has went high because of match

  event++;  //count how many times did the stack come here
  switch (event) {
    case 1:                //comes here on SOE pin will be on
      TIM3->CCR1 = duty1;  //full current for pull in
      TIM2->CCR1 = TIM2->CCR1 + deadTime1 * 2;

      break;
    case 2:
      TIM3->CCR1 = duty2;  //reduced current for soft landing
      TIM2->CCR1 = TIM2->CCR1 + deadTime2 * 2;
      break;
    case 3:
      TIM3->CNT = 0;
      TIM3->CCR1 = duty3;  //even less or no current, reduce bouncing
      TIM2->CCR1 = TIM2->CCR1 + deadTime3 * 2;
      TIM2->CCMR1 = ON;      //T2C1 pin active no change on match
      TIM2->DIER |= 0x0010;  //enable blip detector interrupt
      break;
    case 4:
      TIM3->CCR1 = duty4 + 20;  //medium holding current until the end of injection
      SOI = TIM2->CCR1;
      if (SOI + EOI <= TIM2->CNT) {
        TIM2->CCR1 = TIM2->CNT + 32;  //just incase it got missed
      } else {
        TIM2->CCR1 = SOI + EOI;
      }
      TIM2->CCMR1 = OFF;  //T2C1 pin active low on match
      break;
    case 5:                     //comes here at end of injection drive pin will be low by hardware compare match
      TIM2->CCMR1 = FORCE_LOW;  // now also by software, FORCE pull in low to drive inj sol off
      TIM2->DIER &= 0xFFEF;     //disable blip detector interrupt
      break;
  }

}  //T2C1 handler done

void T2C3_capt(void)  //**** Crankshaft position sensor input ******
{
  static int CKP_armed;
  static uint32_t lastCKPTime, lastCKPpWidth, CKPpWidth;

  if (micros() - lastCKPTime > 50)  //50 us noise filter
  {
    CKP_cnt1++;  //CKP sensor signal edge counter

    if (CKP_cnt1 == 12 || CKP_cnt1 == 36 || CKP_cnt1 == 60) {
      //TrefMicro = micros();     //for 2nd method calculation
      TrefTime = TIM2->CCR3;  // <<-- this is what we are looking for in order to compute timing advance
      //GPIOB->BRR = 0X0080;    //for debugging
    }

    if (CKP_cnt1 == 19 || CKP_cnt1 == 43 || CKP_cnt1 == 67) {
      // TrefMicro = micros();     //for 2nd method calculation
      //   GPIOB->BSRR = 0X0080;   //for debugging
    }

    if (CKP_cnt1 == 30 || CKP_cnt1 == 66) {  //provides a RPM*2 signal to pcm
      //if(CKP_cnt1 == 48){
      //if (CKP_cnt1 == 19 || CKP_cnt1 == 43 || CKP_cnt1 == 67 ){  //was 30 and 66 use for o scope timing analasys
      GPIOB->BSRR = 0X0080;  //Tach signal output PB7 low inverted at connector
    }
    if (CKP_cnt1 == 12 || CKP_cnt1 == 48) {
      //if(CKP_cnt1 == 12){
      //if (CKP_cnt1 == 12 || CKP_cnt1 == 36 ||CKP_cnt1 == 60) {  //was 12 and 48 use for o scope timing analasys
      GPIOB->BRR = 0X0080;  // PB7 Tach signal high - inverted at conn
    }

    CKPpWidth = micros() - lastCKPTime;
    lastCKPTime = micros();

    //*********** CKP missing tooth detector ***********
    if (CKPpWidth > lastCKPpWidth * 3 && !(GPIOA->IDR & 0x0200))  //if PA9 went low and notch was 3X as wide as the previous notch
    {
      CKP_armed = CKP_cnt1;  //armed on this edge. The very next pulse width should be narrower than the last pulse
    }
    if (CKP_cnt1 - CKP_armed == 1 && CKPpWidth < lastCKPpWidth / 2)  //was next tooth half as wide?
    {
      (CKP_cnt1 == 70) ? CKPsync = 1 : CKPsync = 0;  //36-1 tooth wheel has 70 edges
      CKP_cnt1 = 0;                                  //reset and sync
      CKP_armed = 0;
      if (CKPsync == 0) { Serial.println(" CKP loss "); }
    }

    lastCKPpWidth = CKPpWidth;
  }
}  //T2C3_capt done

void T2C4_capt(void)  //****** injection solenoid plunger position time recorder ********
{
  //from injection solenoid current BLIP detector (OPAMP2); how long it takes to pull in solenoid.
  //if (TIM2->CCR4 - BLIPtime[BLIPcnt] > 400 && TIM2->CCR4 - SOI > 600)
  //{
  //GPIOB->ODR ^= 0X8000;    //PB15 sync to O-scope goes high
  BLIPcnt++;
  BLIPcnt = constrain(BLIPcnt, 0, 9);
  BLIPtime[BLIPcnt] = TIM2->CCR4;  //captured by T2C4 in microseconds
  //GPIOB->BRR = 0X8000;    //PB15 sync to O-scope goes low
  //}
}

void T2_OVF(void)           //***** shut off solenoids if not turning ********
{                           //engine not turning, T2 overflows stack comes here
  TIM2->CCMR1 = FORCE_LOW;  //FORCE pull in low to drive inj sol off
  timingDuty = 0;           //turn off timing sol PWM to prevent heat and noise
  TIM16->CCR1 = timingDuty;
  timeout++;
  RPM = 0, RPMavg = 0;
  Timing = 0;
  CamSync = 0;
  CKPsync = 0;
  VPsync = 0;
  RevCounter = 0;
}

void Camshaft_handler(void)  //***** Cam sensor used to sync cylinder counter.
{
  if (digitalRead(PA8) == 0)  //signal falling edge sync up cylinder counter
  {
    // GPIOB->ODR ^= 0X8080;    //PB7 sync to O-scope goes toggle
    (cycle == 1) ? CamSync = 1 : CamSync = 0;
    cycle = 1;
    //RevCounter++;
  }
}  //camshaft done

//********* end of ISR handlers ***************************

//********* main loop ************************************

void loop() {
  //call these functions on every tooth edge
  if (phaseB_edge != old_phaseB_edge) {
    old_phaseB_edge = phaseB_edge;
    injManager();  //this fuction computes RPM, and splits up tasks and events
  }

  //call these fuctions on every cylinder injection cycle
  if (cycle != oldCycle) {
    if (cycle > 6) {
      cycle = 1;
    }
    oldCycle = cycle;
    active_cyl = FiringOrd[cycle];
    // timingCompute();
    timingControl();  //adjust TCV PWM to achieve desired timing advance
    if (cycle == 1 || cycle == 5) {
      RevCounter++;  //inc every crankshaft revolution 2x camshaft
    }
    Amps_Reduced = 0;
  }

  //call these functions whenever
  threshold();  //samples the phase A, B VP speed sensor and calc the zero cross thresholds
  sanityCheck();
  REC();  //check serial port for incoming
  AmpsReduce();
  //CANrec();
  printOut();

  //call these functions if engine is stopped
  if (timeout) {
    getSensors();        //convert raw ADC samples to TPS, MAP, etc
    SOL = AD2result[0];  //to calc volts with sol. resting
    SetAmps();           //auto adjust the inj sol. amps
    SetDeadtime2();
    governor();  //not turning, call governor so it updates fuel for starting
    TempInt = tempCalcInternal(AD1result[0]);
    printOut();  //stuff to display on serial monitor
  }

  //time interval functions
  static uint32_t lastCAN_send;
  if (millis() - lastCAN_send > 500) {
    lastCAN_send = millis();
    CANsend();
  }

  IWatchdog.reload();  //if stack got here feed the watchdog

}  //Main loop done

void injManager(void)  //******* this executes on every tooth edge *******
{
  //***** tasks for tooth edges and split up the CPU load *****

  if (phaseB_edge == 1) {
    amps_ref1 = AD2result[1];
    //deadTime1 = deadTime1 - ((BLIPtime[1] - phaseB[19])/8);
    deadTime1 = constrain(deadTime1, 300, 1000);
  }

  if (phaseB_edge == 3) {  //*** calc RPM ***
    RPMavg = (RPMavg + (40000000 / phaseB[36])) / 2;
    RPM = 40000000 / phaseB[36];
    RPM = constrain(RPM, 0, 6000);
    RPMavg = constrain(RPMavg, 0, 6000);
  }

  if (phaseB_edge == 5) {
    amps_ref2 = AD2result[1];
    SOL = (SOL + AD2result[0]) / 2;  //to calc volts with sol. resting
  }
  if (phaseB_edge == 6) {
    SetAmps();
    SetDeadtime2();  //new method
  }

  if (phaseB_edge == 7) {
    governor();
    getSensors();
  }

  if (phaseB_edge == 8)  //  calculate the engine load.
  {
    Load = LoadCalc(fuelGov2);
  }

  if (phaseB_edge == 21 + crank) {
    amps_hold1 = AD2result[1];
  }

  if (phaseB_edge == 22 + crank)  //ADC conversion should be done now
  {
    amps_hold2 = AD2result[1];
  }

  if (phaseB_edge >= 34) {
    TIM2->CCMR1 = FORCE_LOW;  //T2C1 force low. never inject past edge 34. temp 32 for safety
    TempInt = tempCalcInternal(AD1result[0]);
  }
  if (phaseB_edge == 4) {
    timingCompute();
  }

}  //injManage done

void governor(void) {
  /****** Cummins 24V VP44 PID based governor.*******
      notice no dt or time interval. this is called on every injection cycle so dt is speed dependant.
      RPM is measured and calculated on each injection cycle, so there is no sense trying to adjust
      fuel until the RPM change is observed from the last fuel change.
      TPS is 0 - 5 volt sensor with idle saftey switch built in
      It is about .45 v resting and just less then 4 volts WOT.
      TPS output is halfed with 10 Kohm resistors so it wont exceed 3.3 volts limits of STM32F303 chip
  */

  static int fuelGov, lastFuelGov, lastFuelp, fueli, TempComp;
  int SetPoint = 800;  //desired idle speed
  //int FuelBias = 1570;  //changed type to global var. typical fuel level to cause idle speed at 800 RPM
  const int RestingPos = 285;                //TPS raw A/D sensor value plus a little extra while pedal not pressed
  const int MaxFuel = 4095;                  //4095 is the max allowable fueling level. 3600 is stock-ish
  const int SoftRev = 2900, HardRev = 3200;  //Rev limiter
  static int lastFuel;
  //fuelGov2, RPM, TPS, Load, and crank are global var's.
  //if(millis() - lastFuel < bounceTime){return;}
  lastFuel = millis();

  if (coldStart) {
    TempComp = 180 - (RevCounter);  //sets fuel bias a bit higher for smoother cold startup
    TempComp = constrain(TempComp, 0, 400);
  }

  //FuelBias = FuelBias + TempComp;
  TPS_scale = map(TPS, TPS_rest, TPS_floored, (FuelBias + TempComp), 4095);
  TPS_scale = constrain(TPS_scale, FuelBias, 4095);
  SetPoint = SetPoint + (TempComp / 4) + ((TPS_scale - FuelBias) / 4);  //was 5 determines going off idle feel. 5 is lazier than 4
  fuelp = SetPoint - RPM;
  fueli = fueli + ((fuelp*10) / ki);
  fueli = constrain(fueli, -200, 200);
  //fueld = fuelp - lastFuelp;
  //lastFuelp = fuelp;
  //fueld = constrain((fueld * kd), -100, 100);
  if(fuelp <-25 || fuelp >25){kp =40;}
  if(fuelp >-25 || fuelp <25){kp =10;}
  if (RPM < 700 || RPM > 950) {fueli = 0;}     //prevent integral wind up
  fuelGov = constrain(((fuelp * kp) / 10), -300, 300) + constrain((fueli / 1), -300, 300);  // - fueld
  fuelGov = constrain(fuelGov, -300, droop);
  fuelGov = fuelGov + TPS_scale;  // + constrain(((fuelp * kp)/10), -300, droop) + constrain((fueli / ki), -150, droop-25) - fueld;

  //********* Enforce Table limits ******
  //fuel vs RPM
  if (RpmFuelLimit <= MapFuelLimit) {  //enforce the most restrictive results from fuel tables
    //fuelGov = constrain(fuelGov, 0, RpmFuelLimit);
  }
  //fuel vs BOOST
  else {
    //fuelGov = constrain(fuelGov, 0, MapFuelLimit);
  }

  //********** Anti stall ****** mostly un-tested need a manual transmission.
  if (RPM < 740) {
    //fuelGov = fuelGov + 50; //quick response
    //fuelGov = fuelGov + constrain(((740 - RPM) * 2), 0, 300); //porpotional response
  }

  //**********starting***********
  if (RPM < 400) {  //retard timing 3 deg
    //crank = 1;        //SOI on B tooth edge 20
    //fuelGov =  2400 + (TPS - RestingPos) / 2;
  }
  if (RPM < 325) {  //retard timing 6 deg and even more fuel
    crank = 2;      //SOI on B tooth edge 21

  } else {
    crank = 0;  //SOI on B tooth edge 19. Normal running
  }

  //*********** Rev limit ******
  if (RPM > SoftRev) {
    fuelGov = fuelGov - (RPM - SoftRev) * 3;  //eg 2950 - 2800 = 150 times 3 = 450 fuel units removed
  }
  if (RPM > HardRev) {
    fuelGov -= 2000;
  }
  //**********Limp home limits **************
  if (Limp) {
    if (RPM > 1700) {
      fuelGov = fuelGov - (RPM - 1700) * 2;
    }
    fuelGov = constrain(fuelGov, 0, FuelBias + 1000);
  }
  //*******idle validation switch check********
  if (RPM > 1300 && idle_valid)  //idle valid switch is 5V at idle. 0V when pedal pushed
  {
    fuelGov = fuelGov - (RPM - 1300) * 2;             //limit RPM  to about 1300
    fuelGov = constrain(fuelGov, 0, FuelBias + 600);  //hard fuel limit
  }
  //********* finalize fuel *********
  //kd = constrain(kd, 10, 100);
 // fuelGov = lastFuelGov + (((fuelGov - lastFuelGov) * 10) / kd);  //smooth govenor using Kd term. Ex: enter 22 is same as 2.2
 fuelGov = (fuelGov + lastFuelGov) / 2;  //rolling average fuelGov output for smoother response
  lastFuelGov = fuelGov;
  fuelGov2 = constrain(fuelGov, 0, MaxFuel);  //Atomic fuelGov2 (output) is copied from fuelGov so it won't change during interrupt handlers

}  //govenor done

//******** load calculator **********
//calc the fuel stroke % (_load) comp for the non linear ramp of the cam donut. Sinusoidal for now.
float LoadCalc(uint32_t fuel) {
  float _load;
  for (int i; i < 9; i++) {
    if (fuel >= LoadCorrection[i]) {
      _load = (12.5) * i;
      float r = (fuel - LoadCorrection[i]) / (float)(LoadCorrection[i + 1] - LoadCorrection[i]);
      _load = (12.5 * r) + _load;
    }
  }
  //Load is about 33% to Idle so re-scale load so idle is 0%.
  Load = mapf(_load, 33.0, 100.0, 0.0, 100.0);
  Load = constrain(Load, 0.0, 100.0);  //used for Timing advance table
  return Load;
}


//***************************** Timing section ******************************

void timingCompute(void) {  //this fuction is called on every edge of phase B signal

  //******** compute injection start relative to top dead center, in degrees.*******
  //this is the phase relationship between the cam donut and pump shaft
  static float oldTiming;
  //Timing = (TrefTime - (phaseB[19] + bounceTime)) / (float)phaseB[36];
  //Timing = (TrefTime - (phaseB[19] + 1100)) / (float)phaseB[36];
  //1100 ticks or 550 uS line delay. Speed of sound in diesel about 32 inches from pump to injector.
  //Timing = Timing * 120;        //one index to index tooth is 120 degrees phaseB[36] is the period
  //  Timing = Timing - 9.5;       //it takes about 10 degrees of engine rotation for the presure to ramp to pop injector
  //Timing = Timing - 20.0;       //because the timing referece signal happens at 20 degrees after TDC.
  //Timing = constrain(Timing, 0, 40);
  //Timing = (Timing + oldTiming) / 2.0;
  //oldTiming = Timing;                       //average timing for some smoothing

  if (TrefTime < phaseB[19]) {  //TrefTime occured after the Index tooth
    Timing = (phaseB[19] + 1100 - TrefTime) / (float)(phaseB[23] - phaseB[21]);
  } else {
    Timing = (TrefTime - (phaseB[19] + 1100)) / (float)(phaseB[23] - phaseB[21]);
  }
  Timing = Timing * 6.0;
  Timing = Timing - 20.0;
  Timing = constrain(Timing, 0, 40);
  Timing = (Timing + oldTiming) / 2.0;
  oldTiming = Timing;  //average timing for some smoothing

}  //timingCompute done

void timingControl(void) {  //***** timing advance and lookup, TCV servo control ******
  //this is called every cylinder combustion cycle when actual timing data is fresh
  // and not less than 40 Ms apart (2x 50Hz period) so there is time for the valve to respond

  if (millis() - lastTC < 40) {
    return;
  }
  lastTC = millis();

  TIM16->PSC = TCVfreqControl(RPM);  //set the PWM frequency to 50 or 60Hz depends on RPM.

  //lookup from table, timing value for RPM from 500 to 4000 100 RPM steps

  int i = (RPMavg / 100) - 5;  //table indexs every 100 RPM staring at 500 to 4000
  i = constrain(i, 0, 35);
  float r = (float(RPMavg % 100) / 100);  //remainder, percent between table entries used for interpolation

  desiredTimNL = NoLoadTiming[i];  //timing for no load, refer to variable declarations at top of program
  desiredTimFL = FullLdTiming[i];  //timing for full load
  RpmFuelLimit = RpmFuel[i];       //Fuel Vs RPM limiter

  //interpolate tables
  // speed based timing
  if (i < 35) {
    desiredTimNL += (NoLoadTiming[i + 1] - NoLoadTiming[i]) * r;  //no load timing advance
  }
  if (i < 35) {
    desiredTimFL += (FullLdTiming[i + 1] - FullLdTiming[i]) * r;  //full load
  }
  if (i < 35) {
    RpmFuelLimit += (RpmFuel[i + 1] - RpmFuel[i]) * r;  //Fuel Vs RPM
  }

  //cold weather
  if (TempInt < 70) {
    if (desiredTimNL < 14) {
      desiredTimNL = desiredTimNL + 2.0;
    }
  }
  if (TempInt < 55) {
    if (desiredTimNL < 17) {
      desiredTimNL = desiredTimNL + 2.0;
    }
  }

  //load based timing
  desiredTim = mapf(Load, 0.0, 100.0, desiredTimNL, desiredTimFL);  //use load to scale timing between no load and full load.

  // start up timing
  if (RevCounter < 20 && RPM < 1000) {
    //desiredTim = 10.0;
  }
  desiredTim = constrain(desiredTim, 9.0, 33.0);

  //Boost based fueling
  int j = BOOST / 2;        //table entry for every 2 psi
  j = constrain(j, 0, 22);  //44 psi
  MapFuelLimit = MapFuel[j];
  if (BOOST % 2 && j < 22) {  //if odd psi then split the difference
    MapFuelLimit += (MapFuel[j + 1] - MapFuel[j]) / 2;
  }

  //TCV servo control loop, porportional error

  TerrorP = Timing - desiredTim;                    //desired versus actual timing difference
  timingDuty += constrain(TerrorP * 4, -6.0, 6.0);  // Gain X4 and accumulate, limit under shoot / over shoot.

  // error detection

  if (TerrorP < -5.0 || TerrorP > 5.0) {  //timing control probably unlocked
    if (RPM > 600)
      TCerror++;
  }

  if (TerrorP > -1.5 && TerrorP < 1.5) {  //timing control probably locked in
    TCerror--;
  }

  TCerror = constrain(TCerror, 0, 100);
  if (TCerror == 30) {  //if timing control is out for 30 cycles force TCV dutycyle to default
    timingDuty = 650;   //and try to lock again
    TCerror = 0;
    TimingRetrys++;  //for sanity check function
  }

  //final checks
  //check CKP sensor sync
  (CKPsync == 1) ? TimError-- : TimError++;
  TimError = constrain(TimError, 0, 24);
  if (Limp) {  //force timing defaults if in limp mode
    TimError = 24;
  }
  if (TimError > 12 && RPM <= 1000) {
    timingDuty = 700;  //something wrong with timing ref. so default to safe dutycycle
  } else if (TimError > 12 && RPM > 1000) {
    timingDuty = 650;
  }


  else if (TimError > 12 && RPM > 1800) {
    timingDuty = 600;
  }
  if (RPM > 0 && RPM < 600) {
    timingDuty = 990;  //keep timing at minimum while starting
  }

  if (RPM > 0 && RevCounter < 100) {
    //timingDuty = 700; //let the timing control system stabilize.
  }
  //timingDuty = bounceTime;  //for testing use serial rec "bn xxx" to specify a duty cycle
  timingDuty = constrain(timingDuty, 200, 900);

  TIM16->CCR1 = timingDuty;  //timer 16 ch 1 is the PWM 0 thru 1000 for timing solenoid (50/60Hz)

}  //timingControl done

int TCVfreqControl(int rpm)  //Sets the TCV solenoid PWM frequency to 50 or 60 depending on RPM
{                            //prevents resonance in the timing mechanism.
  int f = _50HZ;             //TIM 16 PSC = 72 Mhz / 1440 is 50 Hz, 1200 is 60 Hz
  if (rpm < 860) {
    f = _50HZ;
  } else if (rpm > 900 && rpm < 1060) {
    f = _60HZ;
  } else if (rpm > 1090 && rpm < 1820) {
    f = _50HZ;
  } else if (rpm > 1850 && rpm < 2200) {
    f = _60HZ;
  } else if (rpm > 2230 && rpm < 2760) {
    f = _50HZ;
  } else if (rpm > 2790 && rpm < 3210) {
    f = _60HZ;
  } else if (rpm > 3230 && rpm < 3670) {
    f = _50HZ;
  } else if (rpm > 3700) {
    f = _60HZ;
  }
  return f;
}  //TCVfreqControl done
//************************************* end of timing section ***********************************


//********* funtions ***********

float mapf(float x, float in_min, float in_max, float out_min, float out_max)  //like Arduino map fuction but for floating point numbers
{                                                                              //maps x from x min to y min, x max to y max.
  if (in_max == in_min) {                                                      //catch div. by 0
    return 0.0f;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ADC1_setup(void) {
  RCC->CFGR2 |= 0x00000170;  //ADC1,2 clock source 72 MHZ/ 16 .
  //GPIOC->MODER |= 0x00000FFF;    //PC0,1,2,3,4,5 set as analog inputs
  ADC1_2_COMMON->CCR |= 0x00800000;  //enable the internal temp sensor on ADC1 ch 16
  ADC1->CR = 0x00000000;             //turn on Analog voltage regulator sequence
  ADC1->CR |= 0x10000000;
  delay(1);
  ADC1->CR |= 0x90000000;  //calibrate ADC1
  delay(1);
  ADC1->CR |= 0x00000001;  //enable ADC1
  delay(1);

  DMA1_Channel1->CPAR = (uint32_t) & (ADC1->DR);  //transfer from here  "&" means -> the address of ADC2's Data Register
  DMA1_Channel1->CMAR = (uint32_t)&AD1result[0];  //to here
  DMA1_Channel1->CNDTR = 1;                       //1 samples to transfer

  //                           16 bit transfers        PERPH to MEMory               MEM INC       enabled      circular buff
  DMA1_Channel1->CCR = (0b01 << DMA_CCR_MSIZE_Pos) | (0b01 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_EN | DMA_CCR_CIRC;


  //*******setup ADC part 2 ********
  //               DMA enabled       12 bit resolution right   DMA circular      continous conv    overwrite
  ADC1->CFGR = ADC_CFGR_DMAEN | (0b00 << ADC_CFGR_RES_Pos) | ADC_CFGR_DMACFG | ADC_CFGR_CONT | (1 << ADC_CFGR_OVRMOD_Pos);

  //                12 bit resolution right     continous conv    overwrite
  // ADC1 -> CFGR = (0b00 << ADC_CFGR_RES_Pos) | ADC_CFGR_CONT | (1 << ADC_CFGR_OVRMOD_Pos);

  //set sample time to 61.5 ADC clock cycles on all ADC2 channels
  ADC1->SMPR1 = 0b00110111110110111110110110110000;  //
  ADC1->SMPR2 = 0b00000110110110110110110110110110;  //181.5 ADC clock cycle sample time

  //                  1 cont. conversions           internal temperature s           amps PC5 ch11           Phase A in PC2          Phase B in PC4
  ADC1->SQR1 |= (0b0000 << ADC_SQR1_L_Pos) | (16 << ADC_SQR1_SQ1_Pos);  // | (11 << ADC_SQR1_SQ2_Pos) | (8 << ADC_SQR1_SQ3_Pos) | (5 << ADC_SQR1_SQ4_Pos);
  //                      TPS PC0 ch6                MAP PC1 ch7
  //ADC1 -> SQR2 |= (6 << ADC_SQR2_SQ5_Pos) | (7 << ADC_SQR2_SQ6_Pos);

  ADC1->CR |= ADC_CR_ADSTART;  //start scanning
}  //ADC1 setup done

void ADC2_setup(void) {
  RCC->CFGR2 |= 0x00000170;    //ADC1,2 clock source 72 MHZ/ 16 .
  GPIOC->MODER |= 0x00000FFF;  //PC0,1,2,3,4,5 set as analog inputs
  ADC2->CR = 0x00000000;       //turn on Analog voltage regulator sequence
  ADC2->CR |= 0x10000000;
  delay(1);
  ADC2->CR |= 0x90000000;  //calibrate ADC2
  delay(1);
  ADC2->CR |= 0x00000001;  //enable ADC2
  delay(1);

  DMA2_Channel1->CPAR = (uint32_t) & (ADC2->DR);  //transfer from here  "&" means -> the address of ADC2's Data Register
  DMA2_Channel1->CMAR = (uint32_t)&AD2result[0];  //to here
  DMA2_Channel1->CNDTR = 6;                       //6 samples to transfer

  //                           16 bit transfers        PERPH to MEMory               MEM INC       enabled      circular buff
  DMA2_Channel1->CCR = (0b01 << DMA_CCR_MSIZE_Pos) | (0b01 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_EN | DMA_CCR_CIRC;

  //*******setup ADC part 2 ********
  //               DMA enabled       12 bit resolution right   DMA circular      continous conv    overwrite
  ADC2->CFGR = ADC_CFGR_DMAEN | (0b00 << ADC_CFGR_RES_Pos) | ADC_CFGR_DMACFG | ADC_CFGR_CONT | (1 << ADC_CFGR_OVRMOD_Pos);

  //set sample time to 61.5 ADC clock cycles on all ADC2 channels
  ADC2->SMPR1 = 0b00110111110110111110110110110000;  //
  ADC2->SMPR2 = 0b00000110110110110110110110110110;  //181.5 ADC clock cycle sample time

  //                  6 cont. conversions           volts PC3 ch 9            amps PC5 ch11           Phase A in PC2          Phase B in PC4
  ADC2->SQR1 |= (0b0101 << ADC_SQR1_L_Pos) | (9 << ADC_SQR1_SQ1_Pos) | (11 << ADC_SQR1_SQ2_Pos) | (8 << ADC_SQR1_SQ3_Pos) | (5 << ADC_SQR1_SQ4_Pos);
  //                      TPS PC0 ch6                MAP PC1 ch7
  ADC2->SQR2 |= (6 << ADC_SQR2_SQ5_Pos) | (7 << ADC_SQR2_SQ6_Pos);

  ADC2->CR |= ADC_CR_ADSTART;  //start scanning
}  //ADC2 setup done

void getSensors(void) {  //comes from Inj manager()  conversions

  //get  ADC results from last conversion on previous cycle

  // static uint32_t lastTPS;
  static uint16_t oldTPS, oldMAP;

  TPS = AD2result[4];
  MAP = AD2result[5];

  TPS = oldTPS + constrain(((TPS - oldTPS) / 8), -30, 40);  // read TPS value and intergrate (smoothing) K = .125
  oldTPS = TPS;                                             // ramp TPS up or down to prevent jerking.
  TPSv = float(TPS) / 615;


  MAP = oldMAP + ((MAP - oldMAP) / 4);
  MAP = constrain(MAP, 0, 4095);
  oldMAP = MAP;
  MAPv = float(MAP) / 615;   //sensor volts
  BOOST = (MAP - 350) / 45;  //PSI
  BOOST = constrain(BOOST, 0, 60);

  idle_valid = (digitalRead(PA11));  //check idle valid signal. wired to PA11 5v when resting.

}  //get sensors done

void threshold(void)
// zero crossing dector circuit threshold adjustment, phase A and phase B from the VP44 toothwheel sensor.
{  //slowly adjust thresholds, Ball park by Phase A,B signals >> very low pass filter (DC component) >> ADC scan + averaging
  static int Ki = 16;
  static uint16_t ThrA = lastGoodThrA, ThrB = lastGoodThrB, lastThrA, lastThrB;
  static uint32_t lastThres;

  if (millis() - lastThres > 4) {
    lastThres = millis();
    (crank) ? Ki = 16 : Ki = 8;
    ThrA = ThrA + ((AD2result[2] - lastThrA) / Ki);
    lastThrA = ThrA;
    ThrB = ThrB + ((AD2result[3] - lastThrB) / Ki);
    lastThrB = ThrB;
    //load DAC every 20 revolutions, reduces jitter
    //if ((RevCounter > 3 && RevCounter < 30) || !(RevCounter % 20)) {
    DAC->DHR12R1 = ThrA;
    DAC->DHR12R2 = ThrB;
    if (RPM > 750 && RPM < 1000) {
      lastGoodThrA = ThrA;
      lastGoodThrB = ThrB;
    }
    //}
  }
}  //threshold done

void SetAmps(void) {
  //increase duty cycle with battery V decrease, fine tune amps with feedback
  //max duty is 600 which is fully on = max current
  int amps;
  //static int duty4AutoAdj;
  float i = 0;
  if (amps_hold2 < 1700) {
    amps_hold2 = amps_hold1;  //incase of very short duration
  }

  BATv = float(SOL) / 203;  //219 for 2.2k resistor, 203 for 2k. R38
  BATv = BATv - 0.12;       //cal factor +.1 ,or -.12
  BATv = constrain(BATv, 0, 20);

  if (RPM < 600 || RevCounter < 100) {
    i = 1.0;
  }
  //fine adjust
  ampErr = ((amps4Manual + i - AMPS));  //porportional error and then average with last amp error.
  if (AMPS < 7.0) {
    duty4AutoAdj = 0;  //prevent windup
    ampErr = 0;
  }
  duty4AutoAdj = duty4AutoAdj + (ampErr * 6);
  duty4AutoAdj = constrain(duty4AutoAdj, -40.0, 40.0);

  //fast coarse adjustment current
  //Il = (BATv * dutycycle) / Rl
  //600 is max duty = 100%
  //sol. resistance = .3 ohm plus .1 ohm for wiring and losses

  int offset = ((BATv - 12.0) * 20) + 55;

  int duty2Batv = sqrt((((amps2Manual + i) * .20) / BATv)) * 600;
  int duty4Batv = sqrt((((amps4Manual + i) * .20) / BATv)) * 600;

  duty4 = duty4Batv - offset + duty4AutoAdj;
  duty2 = duty2Batv - offset + duty4AutoAdj;
  duty2 = constrain(duty2, 220, 500);
  duty3 = constrain(duty3Manual, 0, 300);  //no auto for duty 3 usually 150
  duty4 = constrain(duty4, 130, 500);
}  //SetAmps done

void AmpsReduce(void) {  //reduces current slightly after injection solenoid stable
  int amps;
  if (event == 4 && TIM2->CNT - SOI > 1000)  //750 microseconds after SOI
  {
    amps = AD2result[1] - ((amps_ref1 + amps_ref2) / 2);
    if (RPM == 0) {
      amps = AD2result[1] - 1535;  //incase engine stopped use the typical amp ref 1535. the amp IC bias
    }
    AMPS = (AMPS + float(amps) / 63) / 2;
    AMPS = constrain(AMPS, 0, 20);
    TIM3->CCR1 = duty4;
    Amps_Reduced = 1;  //reset by index tooth
  }
}  //AmpsReduce done

void SetDeadtime2(void) {  //second method uses self inductance formula VL = L*(di/dt)
  //with BatV applied to VL how long is dt for di to = 18 amps?
  //inj. sol. L = 250 microHenry resting, ~320 plunger pulled in.
  //dt = 18 Amps / (BATv / 250)  //because L is microHenry, t will be microseconds: faster math.
  //deadTime1Man is a trial and error value manually entered for best response on oscilloscope.

  deadTime1Bat = float(18 / (BATv / 300));
  deadTime1Bat = constrain(deadTime1Bat, 200, 1000);  //micro seconds
  deadTime1 = deadTime1Bat + deadTime1Man;
  deadTime1 = constrain(deadTime1, 200, 1200);

  deadTime2 = deadTime2Man + float((deadTime1Bat * 0.7));  //half the effect for dead time 2, softens plunger landing
  deadTime2 = constrain(deadTime2, 100, 500);

  deadTime3 = constrain(deadTime3Man, 25, 150);  //coast plunger to landing then holding current
}

void sanityCheck(void) {
  //ugh, this function needs a complete redo!
  //Some original OBDII codes are no longer applicable with the new controller
  //0230, 0232
  static int errCnt1, errCnt2;
  static int DTCpending, LastDTC;
  static uint32_t lastSanity_time;
  if (millis() - lastSanity_time > 500) {
    lastSanity_time = millis();

    LastDTC = DTC;
    //DTC = 0;
    if (TPSv < 0.25) {
      DTCpending = 122;  //APPS voltage to Low
      errCnt1 += 2;
    }
    if (TPSv > 4.0) {
      DTCpending = 123;  //APPS voltage to High
      errCnt1 += 2;
    }
    if (TPSv > 1.5 && idle_valid) {
      DTCpending = 121;  //P0121 APPS Sensor Volts Do Not Agree Idle Validation Signal
      errCnt1 += 1;
    }
    if (BATv > 18.0) {
      DTCpending = 563;  //Battery voltage too High
      errCnt1 += 1;
    }
    if (BATv < 3.0 && LastDTC != 215) {
      DTCpending = 215;  //No voltage from fuel system relay / fuse
      errCnt1 += 1;
    }
    if (amps_ref1 > 2000)  //raw ADC reading
    {
      DTCpending = 251;  //P0251 VP44 Pump Fuel Valve Feedback Circuit Fuel valve current detected when there sould be none
      errCnt1 += 3;
    }
    if (AMPS < 2.0 && okToInj && BATv > 11.0 && RPM > 100 && RPM < 1000) {
      DTCpending = 253;  //P0253 Fuel Injection Pump Fuel Valve Open Circuit
      errCnt1 += 1;
    }
    if (AMPS > 15.0 && RPM > 800) {
      DTCpending = 254;  //P0254 VP44 Fuel Valve Current Too High
      errCnt1 += 2;
    }
    if (TimingRetrys > 5) {
      DTC = 216;         //actual timing not match commanded timing, >= 2 degrees mismatch and 5 re-try.
      TimingRetrys = 0;  //comes from TCV servo function
    }
    if (MAPv < 0.1 && LastDTC != 237) {
      DTCpending = 237;  //MAP sensor too low
      errCnt1 += 1;
    }
    if (BOOST > 5 && RPM < 900 && Load < 10) {
      DTCpending = 238;  // MAP sensor too high when it sould be low.
      errCnt1 += 1;
    }
    if (injMalfunction > 1) {
      DTCpending = 1688;  //injection drive malfunction, inj. event counter incorect.
      errCnt1 += 1;
    }
    if (errCnt1 > 3) {
      DTC = DTCpending;  //a pending error happend too many times, pending code becomes in effect
      errCnt1 = 0;
      DTCpending = 0;
      injMalfunction = 0;
    }
    if (!DTCpending) {
      errCnt1 = 0;  //happy
    }
    //pulsing sensors checked every 0.5 seconds, is engine rotating?
    if (VPsync || CKPsync || CamSync) {
      errCnt2++;  //errCnt2 increments fast or slow based on priority
      if (VPsync && !CKPsync && errCnt2 > 1) {
        //DTC = 336;   //P0336 Crankshaft Position Sensor Signal
      }
      if (VPsync && !CamSync && errCnt2 > 1) {
        //DTC = 341;   //P0341 Camshaft Position Sensor Signal Missing
      }
      if (CKPsync && !VPsync && errCnt2 > 1) {
        DTC = 370;  //P0370 VP44 Speed/Position Sensor Signal Lost
      }
      if (VPsync && CKPsync && CamSync) {
        errCnt2 = 0;  //rotation sensors are happy so reset error counter
      }
    }

    if (!eepromValid) {
      DTC = 1691;  //P1691 VP44 Controller Calibration Error.
    }
    if (WasIWDreset) {
      DTC = 1688;  //prog crashed and internal watchdog reset CPU; P1688 Internal Fuel Injection Pump Failure
    }

    if (DTC == 122 || DTC == 123 || DTC == 216 || DTC == 238 || DTC == 336) {  //run but power limited
      if (RPM > 800 && RevCounter > 60) {                                      //some stabalizing time (60 crankshaft revolutions) to prevent false limp when started
        //Limp = 1;                           //limp does not clear until ignition recycled, TODO
      }
    }
    if (DTC == 563 || DTC == 251 || DTC == 254) {  //critical errors shut off fuel system relay or risk burning inj. solenoid
      shutOff();
    }
  }
}  //sanity done

void shutOff(void) {
  TIM2->CCMR1 = FORCE_LOW;  //FORCE low to drive inj sol off
  pinMode(PC12, OUTPUT);    // Fuel system relay drive; end high Z state which allows hardware timeout and force off by software
  digitalWrite(PC12, LOW);  // Turn off Fuel system relay. may already be off by hardware fault detection circuit
  Serial.print(" shut off DTC P0");
  Serial.println(DTC);  //show why it shut off
}

void ignitionOff(void) {
  //igniton key state. Store threashold values, DTC to emulated EEPROM when key turned off.
  //write sequence takes too long if engine running and it dies.
  //extend the watchdog.
  if (digitalRead(PC7)) {
    return;  //key on do nothing
  }
  TIM2->CCMR1 = FORCE_LOW;  //FORCE low to drive inj sol off
  pinMode(PC12, OUTPUT);    // Fuel system relay drive; end high Z state which allows hardware timeout and force off by software
  digitalWrite(PC12, LOW);  // Turn off Fuel system relay. may already be off by hardware fault detection circuit
  Serial.println(" Saving data ");
  delay(10);
  //          EEPROM.put(200, lastGoodThrA);
  //         EEPROM.put(202, lastGoodThrB);
  //        EEPROM.put(204, DTC);
  //       EEPROM.put(208, lastGoodThrA + lastGoodThrB); //simple checksum
  Serial.println(" Off ");
  IWatchdog.set(5000000);  //should power down in less then 5 seconds
  IWatchdog.reload();
  // while(!digitalRead(PC7));               //while key off wait here for power down
}  //ignitionOff done

void EEPROM_save(void) {
  Serial.println(" Saving data ");
  delay(10);
  EEPROM.put(212, TPS_rest);
  EEPROM.put(216, TPS_floored);
  EEPROM.put(220, kp);
  EEPROM.put(224, ki);
  EEPROM.put(228, droop);
  //EEPROM.put(232, kd);      //not critical
  EEPROM.put(232, FuelBias);


  //        EEPROM.put(204, DTC);
  //       EEPROM.put(208, lastGoodThrA + lastGoodThrB); //simple checksum
  //Serial.println(" Off ");
  IWatchdog.set(5000000);  //should power down in less then 5 seconds
  IWatchdog.reload();
}

void OPAMP2_set(void) {  //This is the injection solenoid current blip detector

  GPIOA->MODER |= 0x00003000;  //OPAMP2 set pin PA6 out to  AF
  GPIOB->MODER |= 0x30000000;  //OPAMP2 set pin PB14 (VIN plus)to AF.
  GPIOC->MODER |= 0x00000C00;  //OPAMP2 set pin PC5 (VIN  minus) to AF
  //no AFR needed OPAMP2 out is direct connected to PA6
  OPAMP2->CSR |= 0x00000005;  //PB14 + in, PC5 - in, PA6 out, enabled
}

void OPAMP3_set(void) {        //This is the phase A input zero crossing detector
  GPIOB->MODER |= 0x0000003F;  //OPAMP3 set pins to AF.
  //no AFR needed OPAMP3 out is direct connected to PB1
  OPAMP3->CSR |= 0x0000002D;  //PB0 + in, PB2 - in, PB1 out, enabled
}

void OPAMP4_set(void) {        //This is the phase B input zero crossing detector
  GPIOB->MODER |= 0x03F00000;  //OPAMP4 set pins to AF.
  //no AFR needed OPAMP4 out is direct connected to PB12
  OPAMP4->CSR |= 0x00000005;  //PB10 - in, PB11 + in, PB12 out, enabled
}

void DAC_set(void) {      //this is the threshold source(s) wired to phase A, B opamps
  DAC->CR |= 0x00010001;  //enable DAC1, 2 and output buffers
  //DAC->DHR12R1 = 0x0000;      //ch1 PA4 12 bit reolution right aligned
  //DAC->DHR12R2 = 0x0000;      //ch2 PA5 12 bit resolution right aligned
  DAC->DHR12R1 = lastGoodThrA;  //phase A threshold starting value
  DAC->DHR12R2 = lastGoodThrB;  //phase B threshold
}  //DAC_setup done

void REC() {
  //process any incoming bytes from the serial port.
  // read the incoming characters
  //separate numbers from Alpha
  //build number or alpha string from incoming stream
  //end when newline <ENTER> is received

  static char incomingChar;
  static int numbersIn;
  static String instring, lettersIn, inNum;

  while (Serial.available() > 0) {
    incomingChar = Serial.read();
    if (isDigit(incomingChar)) {
      inNum += incomingChar;  //concatonate this is a string not interger
    }
    if (isAlpha(incomingChar)) {
      instring += incomingChar;
    }
    if (incomingChar == '?') {  //help prompt//TODO a new help menu using "F macro" to save on RAM
      // help();
    }
    if (incomingChar == '\n')  //<enter> end of input
    {
      lettersIn = instring;
      numbersIn = (inNum.toInt());  //now string is converted to interger
      instring = "";
      inNum = "";
      Serial.print(lettersIn);
      Serial.print(" ");
      Serial.println(numbersIn);
    }
  }  //end of while serial

  if (lettersIn == "rel") {
    releaseTime = numbersIn;
    releaseTime = constrain(releaseTime, 0, 2000);
    //EEPROM.put(200, releaseTime);
  }
  if (lettersIn == "da") {
    duty1 = numbersIn;
  }
  if (lettersIn == "ib") {
    numbersIn = constrain(numbersIn, 95, 170);  // 9.5A to 17A
    amps2Manual = float(numbersIn / 10.0);
  }
  if (lettersIn == "dc") {
    duty3Manual = numbersIn;
  }
  if (lettersIn == "id") {
    numbersIn = constrain(numbersIn, 75, 120);  // 7.5A to 12A
    amps4Manual = float(numbersIn / 10.0);
  }

  if (lettersIn == "ta") {
    deadTime1Man = numbersIn;
  }
  if (lettersIn == "tb") {
    deadTime2Man = numbersIn;
  }
  if (lettersIn == "tc") {
    deadTime3Man = numbersIn;
  }
  if (lettersIn == "bn") {
    bounceTime = numbersIn;
    bounceTime = constrain(bounceTime, 0, 6000);
  }
  if (lettersIn == "cor") {
    correction = numbersIn - 200;
    correction == constrain(correction, -200, 200);
  }
  if (lettersIn == "tpsl") {
    Serial.println(TPS);
    TPS_rest = TPS + 25;
  }
  if (lettersIn == "tpsh") {
    Serial.println(TPS);
    TPS_floored = TPS;
  }
  if (lettersIn == "kp") {
    kp = numbersIn;
  }
  if (lettersIn == "ki") {
    ki = numbersIn;
  }
  if (lettersIn == "kd") {
    kd = numbersIn;
  }
  if (lettersIn == "droop") {
    droop = numbersIn;
  }
  if (lettersIn == "fuelbias") {
    FuelBias = constrain(numbersIn, 1300, 1900);
  }
  if (lettersIn == "save") {
    EEPROM_save();
  }
  lettersIn = "";
  numbersIn = 0;

}  //REC done

void printOut(void) {  //split up printing so It won't delay things to much
  //caution avoid excess printing
  static uint32_t LastPrint;
  static int prtGrp;

  if (millis() - LastPrint > 500) {
    if (prtGrp == 0) {
      Serial.print(" RPM ");
      Serial.print(RPM);
      Serial.print(" fuel ");
      Serial.print(fuelGov2);
      //Serial.print(" tim ");
      //Serial.print(TrefTime);// - phaseB[19]);
      //Serial.print(" correction ");
      //Serial.print(correction);
    }
    if (prtGrp == 1) {
      // Serial.print(" bounce ");
      // Serial.print(bounceTime);
      Serial.print(" BOOST ");
      Serial.print(BOOST);
      //Serial.print(" TPS ");
      //Serial.print(TPS);
      //Serial.print(" RpmFuel ");
      //Serial.print(RpmFuelLimit);
      //Serial.print(" ampref ");
      //Serial.print(amps_ref1);
      Serial.print(" BATv ");
      Serial.print(BATv, 1);
    }
    if (prtGrp == 2) {
      //Serial.print(" timingDuty ");
      //Serial.print(timingDuty);
      //Serial.print(" desiredT ");
      //Serial.print(desiredTim, 1);
      //Serial.print(" Tps ");
      // Serial.print(TPS);
      //Serial.print(" deadTime1Bat ");
      //Serial.print(deadTime1Bat);
      //Serial.print(" MSR ");
      //Serial.print(CAN->MSR, HEX);
      Serial.print(" AMPS ");
      Serial.print(AMPS, 1);
      Serial.print(" Timing ");
      Serial.print(Timing, 1);
      Serial.print(" TPS scale ");
      Serial.print(TPS_scale);
      Serial.print(" Load ");
      Serial.println(Load, 1);

      // Serial.print(" CAN ");
      // Serial.println(CAN->sFIFOMailBox[1].RDLR, HEX);

      if (DTC > 0 && DTC < 999) {
        //Serial.print(" DTC P0");       //show  error or why it shutdown.
        //Serial.println(DTC);
        DTC = 0;
      } else if (DTC > 0 && DTC > 999) {  //print without a leading 0
        //Serial.print(" DTC P");       //show  error or why it shutdown.
        //Serial.println(DTC);
        DTC = 0;
      }

      if (!digitalRead(PC13))  //user push button on nucleo checked in print function as a simple debouncer
      {
        //something to do when button pressed
        //EEPROM_save();
      }

      LastPrint = millis();
    }

    prtGrp++;
    if (prtGrp > 2) {
      prtGrp = 0;
    }
  }

}  // printOut done

void CANsetup(void) {

  GPIOB->AFR[1] |= 0x00000099;  //AF9;  re-map CANRX - PB8, CANTX - PB9
  GPIOB->MODER |= 0x000A0000;   //portB  pins 8 & 9 selected as alternate functions
  CAN->MCR = 1 << RESET;        //reset the CAN perp
  delay(1);
  CAN->MCR = 0x00000011;  //abom on, no sleep init, NART one time transmit
  CAN->BTR = 0x401C0008;  //250K baud pre=8, ts1=13, ts2=2, sjw=1, loopback mode

  //filters
  // CAN->FM1R |= 0x1C << 8;
  CAN->FMR |= 0x1UL;  //init mode all filters inactive
  CAN->FA1R &= ~(0x1UL);
  CAN->FS1R |= 0x1UL;
  CAN->sFilterRegister[0].FR1 = 0x0UL;  //don't care mask all msg ID's pass thru
  CAN->sFilterRegister[0].FR2 = 0x0UL;  //don't care mask
  CAN->FM1R &= ~(0x1UL);                //mask mode
  CAN->FFA1R |= 0x1UL;                  //asign filter #0 to FIFO 1
  CAN->FA1R |= 0x1UL;                   //filter #0 activate
  CAN->FMR &= ~(0x1UL);                 //filter(s) on - enabled

  CAN->MCR = 0x00000040;  //activate CAN in loopback mode//this doesn't control anything yet.

  //***** turn on CAN interrupt ******
  CAN->IER |= 0x10UL;  //RX1 FIFO 1 message pending interrupt enable
  NVIC_SetPriority(CAN_RX1_IRQn, 4);
  NVIC_EnableIRQ(CAN_RX1_IRQn);
}

void CANsend(void) {
  static uint16_t datLow;
  volatile int CANtimeOut;
  CAN->sTxMailBox[0].TDLR = datLow++;  //dummy data
  CAN->sTxMailBox[0].TDHR = 0x05060708;
  CAN->sTxMailBox[0].TDTR = 0x00000008;  //DLC 8
  CAN->sTxMailBox[0].TIR = 0x00400000;   //std id 02

  CAN->sTxMailBox[0].TIR |= 0x1UL;  //RTS tx mail box 0
  while ((CAN->sTxMailBox[0].TIR & 0x1UL) && CANtimeOut++ < 1000000)
    ;
}

void CANrec(void) {
  while (CAN->RF1R & 0x3UL) {
    //Serial.print(" CAN ");
    //Serial.println(CAN->sFIFOMailBox[1].RDLR, HEX);
    CAN->RF1R |= 0x20UL;  //indicate to mailbox it has been read by software
  }
}



int32_t tempCalcInternal(uint16_t Tsample) {

  int32_t Temperature = (int32_t)(((110.0f - 30.0f) / ((float)(temp110) - (float)(temp30))) * ((float)(Tsample) - (float)(temp30)) + 30.0f);
  Temperature = Temperature - 10.0f;
  return Temperature = ((Temperature * 9) / 5) + 32;
}

//********* end of program ********
