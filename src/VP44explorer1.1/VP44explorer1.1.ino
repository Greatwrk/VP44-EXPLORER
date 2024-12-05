

#include <EEPROM.h>
#include <IWatchdog.h>
#include "VP44tables.h"

/*
  VP44 explorer
  ver 1.1
  by Ray Malcuit
  Dec. 2024
  pump controller. Predict SOI time by kinematics -> pump shaft velocity, acceleration method
  arm core nucleo 64 pin STM32f303RE board
  requires STM32duino core, NOT Maple core. Not for cube or HAL.
  higher resolution timer 2 is now 0.5 microsecond per tick.
  Sensors ADC now with DMA, less CPU overhead
  Phase A captured by Timer 8 now.
  Diagnostic trouble codes added
  Internal watchdog timer added
  DTC stored
  Added keyed igniton shutdown and save.
  Rapid injection amps adjustment based on battery voltage.
  Inj sol deadtime based on self induced voltage
  Limp home added
  Injection line delay correction added
  improved idle speed governor with variable kp gain based on load
  works in drive and neutral better. less jumps with AC clutch.
  Added EEPROM storage and defaults for first time use
  Added a cylinder de-activation test.
  moved look-up tables and default params. to VP44tables.h and VP44tables.cpp
  fixed help menu
  added SOE and EOE, cleaned up timer compare register loading. 
  expanded timer 2 and edge counters for future 4 cylinder.

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
   PB13 inj solenoid drive current to ADC3 ch5 in
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

#define ON 0X3110;                         //Timer 2 compare Ch. 1 register pin states on match high
#define OFF 0X3120;                        // TIM2->CCMR1 = off, pin goes low when match occurs...
#define NO_CHANGE 0X3100;                  //cause interrupt only pin remains the same
#define FORCE_LOW 0x3140;                  //forces tim2 ch1 pin low
#define _50HZ 1440;                        //TIM16 prescaler
#define _60HZ 1200;                        //
#define scopeLow GPIOB->BRR = 0X8000;      // PB15 off - fast method O-scope goes low
#define scopeHigh GPIOB->BSRR = 0X8000;    //PB15 sync to O-scope goes high
#define scopeToggle GPIOB->ODR ^= 0X8000;  //PB15 sync to O-scope toggle
#define TACHhigh GPIOB->BSRR = 0X0080;     //Tach signal output PB7 high inverted at connector;
#define TACHlow GPIOB->BRR = 0X0080;       // PB7 Tach signal low - inverted at conn;

volatile uint32_t phaseA[60];
volatile uint32_t phaseB[60], phaseBpredict[60];
volatile uint32_t BLIPtime[10], TrefTime;
volatile uint32_t lastA, lastB;
volatile int cycle, oldCycle, okToInj, injMalfunction;
volatile int phaseA_edge, phaseB_edge, old_phaseB_edge, CKPsync, CamSync, CKP_cnt1, BLIPcnt;
volatile uint32_t velocity, SOE, SOI, EOI; 
volatile int32_t EOE;
volatile uint16_t velocityA, oldVelocityA;
volatile int16_t accelA;
volatile int accel, event, predictErr, VPsync, timeout;
uint16_t timingDuty, RecTimingDuty;
int32_t AmpErr;
int TrefEdge, deadTime1Bat, DTC, storedDTC, TimingRetrys;
int TCerror, lastTC, TimError, active_cyl, kill = 0;
int TPS, MAP, SOL, idle_valid, crank = 2;
int RPM, RPMavg, printPauseT;
float Timing, Timing2, oldTiming, desiredTim, desiredTimNL, desiredTimFL, TerrorP, amps, amps1;
float TPSv, MAPv, BATv, AMPS, ampErr, Load;
float milliLiters;
uint16_t AD2result[6], AD1result[6];  //ADC2 global var
int help;
int amps_ref1;
int amps_ref2;
int amps_hold1;
int amps_hold2;
int duty4AutoAdj;
uint16_t RevCounter, checksum;
bool Limp, WasIWDreset, Amps_Reduced;
int RpmFuelLimit = 4095, MapFuelLimit = 4095, BOOST;
int fuelGov2 = 2000;
int TempInt, coldStart, offset = 300;
uint TPS_scaled;
uint32_t printInterval = 500;

/****** Get the factory calibration values from system memory to improve the internal temperature sensor accuracy****/
uint32_t VREF_INTERNAL = 0x1FFFF7BA;
uint16_t Vrefi = *((uint32_t *)(VREF_INTERNAL));
uint32_t TEMP30 = 0X1FFFF7B8;
uint16_t temp30 = *((uint32_t *)(TEMP30));
uint32_t TEMP110 = 0x1FFFF7C2;
uint16_t temp110 = *((uint32_t *)(TEMP110));
float TS = 1710;

//Firimg order
const int FiringOrd[]{ 0, 1, 5, 3, 6, 2, 4 };   //4 cyl 0, 1, 3, 4, 2

const uint8_t
  phaseB_in = PA1,
  CMP = PA8,
  idle = PA11,
  phaseA_in = PB3,
  TACH = PB7,
  O_scope = PB15,
  IGN = PC7,
  failsafe = PC12,
  button = PC13;

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
  IWatchdog.begin(2000000);  //internal watchdog resets CPU after 2 second if execution freezes or go off track

  Serial.begin(115200);  //start serial PA2 TX. PA3 RX. to st-Link which is USB converter
  delay(100);            //let USB get going

  if (WasIWDreset) {
    //Serial.println(" DTC P1688 ");  //internal fuel pump controller failure
  }
  // get data previosly stored in emmulated EEprom (flash last page)
  if (!WasIWDreset) {
    EEPROM_read();
  }

  pinMode(phaseB_in, INPUT);  //b phase in
  pinMode(idle, INPUT);       //idle validation in
  pinMode(CMP, INPUT);        //early 24 valve cam sensor, single notch
  pinMode(phaseA_in, INPUT);  //phase A in
  pinMode(TACH, OUTPUT);      //Tachometer out
  pinMode(O_scope, OUTPUT);   //O'scope debug out
  pinMode(IGN, INPUT);        //Ignition key on sens
  pinMode(failsafe, OUTPUT);  //failsafe relay
  pinMode(button, INPUT);     //user push button sw on nucleo board

  //******** Speed sensor and injection valve events: timer 2 is fixed at 0.5 uS per tick, capture tooth wheel edges on CH 2 ******
  //********** Main Timer 2 setup************
  HardwareTimer *MainTim2 = new HardwareTimer(TIM2);  //this instance is required for attachinterrupt and ISR handler to work-
  //could possibly be done with low level - NVIC controller
  // Timer 2 is 32 bits wide but has no default pins so pins have to be remapped as ALT FUNC.
  // Timer 2 see datasheet table 14
  GPIOA->AFR[0] |= 0x00000011;  //AF1; PA0 Timer 2 ch 1, PA1 ch 2
  GPIOA->AFR[1] |= 0x00000AA0;  //AF10; PA9 Timer 2 ch 3, PA10 ch 4
  GPIOA->MODER |= 0x00280F0A;   //PA0, PA1 and PA9, PA10 pins set mode alt function.

  TIM2->CR1 = 0X0004;  //INTERNAL CLK, UPCOUNTER, UEV INTERRUPT ENABLED
  TIM2->PSC = 35;      //0.5 micro sec per tick
  TIM2->ARR = 600000;  //300 mS overflows below 90 RPM
  TIM2->CNT = 0;
  TIM2->CCER = 0;        //some timer registers are only writeable when this is disabled
  TIM2->CCMR1 = 0x3130;  //T2C1 force comp out low. T2C2 input capture. WAS 40
  TIM2->CCMR2 = 0X3131;  //input capture ch 3 N8 filter , ch 4 capture
  //the two previous steps have to be done before enabling the channels in the next register below
  TIM2->CCER = 0X3BB1;  // ch1 output compare to pin PA0, ch2 either edge, ch4 capture - edge, ch3 capture any edge was13B1
  TIM2->CCR1 = 600001;  //COMPARE TIMER TO 600001 micro seconds un-reachable

  //****** timer2 interrupts enable *********
  MainTim2->attachInterrupt(T2_OVF);        //Tim2 timeout 300 mS. about 90 RPM
  MainTim2->attachInterrupt(1, T2C1_comp);  //Solenoid driver pin PA0
  MainTim2->attachInterrupt(2, T2C2_capt);  //VP speed sensor phase B
  MainTim2->attachInterrupt(3, T2C3_capt);  //Crank Pos sensor capture PA9
  MainTim2->attachInterrupt(4, T2C4_capt);  //solenoid current 'BLIP' detector

  //*******start timer 2 **********
  TIM2->CR1 |= 0X0001;  //start TIMER2

  //******** Speed sensor phase A timer 8 is fixed at 0.5 uS per tick, capture tooth wheel edges on CH 4 ******
  //********** Main Timer 8 setup************
  HardwareTimer *MainTim8 = new HardwareTimer(TIM8);  //this instance is required for attachinterrupt and ISR handler to work-
  // Timer 8 is 16 bits wide but has no default pins so pins have to be remapped as ALT FUNC.
  // Timer 8 see datasheet table 14
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

  //***** TIMER 3 SETUP FOR PWM ON CH 1, 3 *********
  //*******controls fuel solenoid current, buck converter
  GPIOC->AFR[0] |= 0x02000000;  //AF2; PC6 mapped to Timer 3 ch 1 compare
  GPIOC->AFR[1] |= 0x00000002;  //AF2; PC8 mapped to Timer 3 ch 3 compare, not used
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
  attachInterrupt(digitalPinToInterrupt(CMP), Camshaft_handler, CHANGE);

  //********** set up analog systems *******
  ADC1_setup();
  ADC2_setup();
  DAC_set();
  OPAMP2_set();
  OPAMP3_set();
  OPAMP4_set();
  delay(10);  //let the ADCs get some conversions into DMA

  if (AD2result[0] > 750) {  //If solenoid has battery over 3.5V then fuel system relay fault
    //DTC = 1285;              //should not be turned on yet.
    Serial.print(" DTC P");
    Serial.println(DTC);
  }
  //fuel solenoid will be ruined if left on too long. WARNING! do not defeat the failsafe relay funtion.
  pinMode(failsafe, OUTPUT);     // fuel system relay drive also failsafe shutdown incase software or hardware malfunction
  digitalWrite(failsafe, HIGH);  // Turn on fuel system relay
  delay(200);                    // so the relay has time to engage
  pinMode(failsafe, INPUT);      //fuel system relay should latch on by hardware now. goto high Z state

  Serial.println(" Enter ? for help menu ");
  CANsetup();       //config the CAN periph
  if (tempCalcInternal(AD1result[0]) < 60) { coldStart = 1; }

}  //setup done

//*********** ISR handlers start here *************************
extern "C" void CAN_RX1_IRQHandler(void) {  //STM32duino does not support CAN. Using CMISS LL
  //Serial.println(" isr fired ");
  CANrec();
}

void T8C4_capt(void) {
  //********** vp tooth wheel sensor phase A signal input******
  //record all the tooth edge times as they pass by sensor A
  // 4 cyl edge 19 changes to 29

  if (TIM2->CNT - lastA > 100) {  //50 uS noise filter
    phaseA_edge++;
    //scopeHigh;    //PB15 sync to O-scope goes high
    phaseA[phaseA_edge] = TIM8->CCR4;  //Timer2 ran out of capture channels so using TIM8
    lastA = TIM2->CNT;

    if (phaseA_edge > 4 && phaseA_edge < 34) {  //ignore the index tooth region when calculating the following //4 cyl 34 >> 54
      //Not fractional degrees per microsecond that will require a lot of long math.
      //velocity and accel expressed as the TIME (in microseconds) per 6 degrees of rotation
      //calculated on + and - edges so velocity is updated every 6 degrees
      //pulses are not guaranteed to be 50% duty cycle so use period
      velocityA = phaseA[phaseA_edge] - phaseA[phaseA_edge - 2];  // steady velocity (current period)
      velocityA = (velocityA + oldVelocityA) / 2;                 //rolling average velocity
      accelA = (accel + (velocityA - oldVelocityA)) / 2;          //average acceleration as the diff in velocities
      oldVelocityA = velocityA;

      int cnt1 = phaseB_edge + 2;  //predict the next B edge time from the previous known B time based on leading phase A
      int cnt2 = phaseB_edge + 1;  //velocity and accelleration, predict error is calculated later and used to improve prediction
      phaseBpredict[cnt1] = phaseB[phaseB_edge] + velocityA + accelA + predictErr;
      phaseBpredict[cnt2] = phaseB[phaseB_edge - 1] + velocityA + accelA + predictErr;

      while (cnt1 < 32)  //compute some remaining Phase B edge times from leading edge A    //4 cyl 22 >> 32
      //math has time to complete between the lag of B from A
      {
        phaseBpredict[cnt1 + 2] = phaseBpredict[cnt1] + velocityA + accelA / 2;
        phaseBpredict[cnt2 + 2] = phaseBpredict[cnt2] + velocityA + accelA / 2;
        cnt1 += 2;
        cnt2 += 2;
      }

      //**** SOI, start of injection, SOE, start of energized. EOI, end of injection. EOE end of energised
       
      SOI = phaseBpredict[19 + crank] + offset - 300;            //we want SOI to start on tooth edge 19 or edge 21 if starting engine
      EOI = ((float)fuelGov2 / 512) * velocityA;  //length of inj. not absolute end time
      EOE = EOI - releaseTime * 2;
      if (EOE < 1) { EOE = 0; }  //EOE is signed prevent less than 0
      if (EOE > 32) {            //don't bother with injection less the 16 uS
        okToInj = 1;
      }
      if (okToInj && VPsync && event == 0 && phaseA_edge > 7) {
        SOE = SOI - (deadTime1 * 2 + deadTime2 * 2 + deadTime3 * 2);  //copensate for the sol. plunger inertia (dead time)
        if (TIM2->CNT + 10 < SOE) {                                   //timer is too close to SOE to change it, use previous prediction
          TIM2->CCR1 = SOE;                                           //chan 1 compare match value <think> set alarm time on stopwatch
          TIM2->CCMR1 = ON;                                           //T2C1 pin active high on match
        }
      }
      if (event == 4) {
        if (TIM2->CNT + 10 < (EOE + SOI)) {
          TIM2->CCR1 = EOE + SOI;
          TIM2->CCMR1 = OFF;
        }
      }
    }
    if (phaseA_edge > 58) {  //restart tooth edge counter //was 38
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

    if (phaseB_edge == 19) {  //4 cyl>> 29
      // GPIOB->BRR = 0X8000;  // PB15 off - fast method scope trigger
      scopeLow;
    }
    lastB = TIM2->CNT;
    phaseB[phaseB_edge] = TIM2->CCR2;  //record the time of this edge, <think> Lap button on stopwatch

    //************ index tooth detector ********
    // when    PA1 is high  AND    PC9 is low: this is an index tooth
    if (GPIOA->IDR & 0x0002 && !(GPIOC->IDR & 0x0200))  // when phase B(PA1) goes high and phase A(PC9) is low at the same time then trigger
    {                                                   //init a new cycle, <think> reset the stopwatch
      //PB15 sync to O-scope goes high
      scopeHigh;
      TIM2->EGR = 0x0001;  // reset timer 2 cntr and all capture, compare events
      phaseA_edge = 0;
      if (phaseB_edge == 36) {  //4 cyl >> 56
        VPsync = 1;
      } else {
        VPsync = 0;
        Serial.println(" sync loss ");
      }
      phaseB_edge = 0;
      phaseB[0] = 0;
      SOE = 600001;
      SOI = 600001;  //set timer2 compares to un-reachable numbers
      EOE = 600001;
      EOI = 600001;
      cycle++;
      timeout = 0;
      if (okToInj && event != 5) {  //should never be anything except 5 timer2 match events happened when index tooth occurs
        injMalfunction++;
      }
      event = 0;
      BLIPcnt = 0;
      okToInj = 0;
    }

    if (phaseB_edge > 58) {             //edge counter normally does not get this high while engine is running //was 38
      phaseB[0] = phaseB[phaseB_edge];  //typicaly 36 edges for 6 cylinder or 54? for 4 cylinder
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
      TIM2->CCMR1 = ON;      //T2C1 pin on
      TIM2->DIER |= 0x0010;  //enable blip detector interrupt
      break;
    case 4:                     //SOI
      TIM3->CCR1 = duty4 + 20;  //medium holding current until the end of injection
      TIM2->CCR1 = TIM2->CNT + EOE + 2;
      TIM2->CCMR1 = OFF;  //T2C1 pin active low on next match
      break;
    case 5:                     //comes here at end of injection drive pin will be low by hardware compare match
      TIM2->CCR1 = 600001;      //un-reachable
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
      TrefTime = TIM2->CCR3;  // <<-- this is what we are looking for in order to compute timing advance
      //TACHlow;    //for debugging
    }

    if (CKP_cnt1 == 19 || CKP_cnt1 == 43 || CKP_cnt1 == 67) {
      //TACHhigh;   //for debugging
    }

    if (CKP_cnt1 == 30 || CKP_cnt1 == 66) {  //provides a RPM*2 signal to pcm
      //if (CKP_cnt1 == 19 || CKP_cnt1 == 43 || CKP_cnt1 == 67 ){  //was 30 and 66 use for o scope timing analasys
      //Tach signal output PB7 high inverted at connector
      TACHhigh;
    }
    if (CKP_cnt1 == 12 || CKP_cnt1 == 48) {
      //if (CKP_cnt1 == 12 || CKP_cnt1 == 36 ||CKP_cnt1 == 60) {  //was 12 and 48 use for o scope timing analasys
      // PB7 Tach signal low - inverted at conn
      TACHlow;
    }
    CKPpWidth = micros() - lastCKPTime;
    lastCKPTime = micros();

    //*********** CKP missing tooth detector ***********
    if (CKPpWidth > lastCKPpWidth * 3 && !(GPIOA->IDR & 0x0200))  //if PA9 went low and notch was 3X as wide as the previous notch
    {
      CKP_armed = CKP_cnt1;  //armed on this edge. The very next tooth width should be narrower than the last notch
    }
    if (CKP_cnt1 - CKP_armed == 1 && CKPpWidth < lastCKPpWidth / 2)  //was this the next tooth AND half as wide as the previous?
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
  //scopeToggle;
  BLIPcnt++;
  BLIPcnt = constrain(BLIPcnt, 0, 9);
  BLIPtime[BLIPcnt] = TIM2->CCR4;  //captured by T2C4 in microseconds
  //scopeLow;
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
  SOE = 600001;
  SOI = 600001;  //set timer2 compare registers to un-reachable numbers
  EOE = 600001;
  EOI = 600001;
}

void Camshaft_handler(void)  //***** Cam sensor used to sync cylinder counter.
{
  if (digitalRead(CMP) == 0)  //signal falling edge sync up cylinder counter
  {
    //scopeToggle;
    (cycle == 1) ? CamSync = 1 : CamSync = 0;
    cycle = 1;
  }
}  //camshaft done

//********* end of ISR handlers ***************************

//********* main loop ************************************

void loop() {
  //call these functions on every phase B tooth edge
  if (phaseB_edge != old_phaseB_edge) {
    old_phaseB_edge = phaseB_edge;
    injManager();  //this fuction computes RPM, and splits up tasks and events
  }

  //call these fuctions on every cylinder injection cycle
  if (cycle != oldCycle) {
    if (cycle > 6) {  //4 cyl>> 4
      cycle = 1;
    }
    oldCycle = cycle;
    active_cyl = FiringOrd[cycle];
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
  if (help) {
    Help();
    printInterval = 10000;  //delay the stats and RPM printing so help menu can be read
  }
  printOut();
  ignitionOff();

  //call these functions if engine is stopped
  if (timeout) {
    getSensors();        //convert raw ADC samples to TPS, MAP, etc
    SOL = AD2result[0];  //to calc volts with sol. resting
    SetAmps();           //auto adjust the inj sol. amps
    SetDeadtime2();
    governor();  //not turning, call governor so it updates fuel for starting
    TempInt = tempCalcInternal(AD1result[0]);
  }

  //time interval functions
  //CAN is in loopback mode and not really doing anything.
  static uint32_t lastCAN_send;
  if (millis() - lastCAN_send > 500) {
    lastCAN_send = millis();
    CANsend();
  }

  IWatchdog.reload();  //if stack got here feed the watchdog

}  //Main loop done

void injManager(void)  //******* this executes on every B tooth edge *******
{
  //***** tasks for tooth edges and split up the CPU load *****
  static int lastRPM;
  if (phaseB_edge == 1) {
    amps_ref1 = AD2result[1];
    //deadTime1 = deadTime1 - ((BLIPtime[1] - phaseB[19])/8);
    deadTime1 = constrain(deadTime1, 300, 1000);
  }
  if (phaseB_edge == 3) {  //*** calc RPM ***
    //RPMavg = (RPMavg + (40000000 / phaseB[36])) / 2;
    RPM = 40000000 / phaseB[36];
    RPMavg = lastRPM + ((RPM - lastRPM) / 4);
    RPM = constrain(RPM, 0, 6000);
    RPMavg = constrain(RPMavg, 0, 6000);
    lastRPM = RPM;
    // 4 cyl. use RPM = 60000000 / phaseB[56];
  }
  if (phaseB_edge == 4) {
    timingCompute();
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
  }
  if (phaseB_edge == 8)  //  calculate the engine load.
  {
    getSensors();
    Load = LoadCalc(fuelGov2);
    MapFuelLimit = MapFuelLookup(BOOST);
  }
  if (phaseB_edge == 21 + crank) {  //4 cyl 31
    amps_hold1 = AD2result[1];
    RpmFuelLimit = RPMFuelLookup(RPMavg);
  }
  if (phaseB_edge == 22 + crank)  //ADC conversion should be done now //4 cyl 32
  {
    amps_hold2 = AD2result[1];
  }
  if (phaseB_edge >= 34) {    // 4 cyl 44
    TIM2->CCMR1 = FORCE_LOW;  //T2C1 force low. never inject past edge 34.
    TempInt = tempCalcInternal(AD1result[0]);
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
  static float fuelGov, lastFuelGov;
  static float lastRPM;
  static int TempComp;
  static int fuelp, fueli, fueld;
  int SetPoint = 800;  //desired idle speed
  //int FuelBias = 1570;  //changed type to global var. typical fuel level to cause idle speed at 800 RPM
  const int MaxFuel = 4095;                  //4095 is the max allowable fueling level. 3600 is stock-ish
  const int SoftRev = 3000, HardRev = 3200;  //Rev limiter
  float kpL = kp / 100.0, kiL = ki / 100.0, kdL = kd / 100.0;
  float LF = Load - 3.0; 
  LF = constrain(Load, 0.0 , 2.0);
  kpL = mapf(LF, 0.0, 2.0, kpL, kpL/5);
  TPS_rest = constrain(TPS_rest, 200, 300);
  TPS_floored = constrain(TPS_floored, 1700, 2300);
  FuelBias = constrain(FuelBias, 1200, 1800);

  if (coldStart) {
    TempComp = 180 - (RevCounter);  //sets fuel bias a bit higher for smoother cold startup
    TempComp = constrain(TempComp, 0, 400);
  }

  TPS_scaled = map(TPS, TPS_rest, TPS_floored, (FuelBias + TempComp), 4095);
  TPS_scaled = constrain(TPS_scaled, FuelBias, 4095);
  SetPoint = SetPoint + (TempComp / 4) + ((TPS_scaled - FuelBias) / 5);  //was 5 determines going off idle feel. 5 is lazier than 4
  fuelp = SetPoint - RPM;
  fuelp = constrain(fuelp, -150, 150);
  fueli = fueli + (fuelp * kiL);
  fueli = constrain(fueli, -250.0, droop);
  fueld = RPM - RPMavg;//lastRPM;
  lastRPM = RPM;
  if (fueli > 20 && RPM > 820) { fueli = fueli / 3; }   //control integral wind-up.
  if (fueli < -20 && RPM < 780) { fueli = fueli / 3; }
  fuelGov = fuelp * kpL + fueli - fueld * kdL;
  fuelGov = constrain(fuelGov, -300.0, droop);
  fuelGov = fuelGov + TPS_scaled;
  //********* Enforce Table limits ******
  //fuel vs RPM
  if (RpmFuelLimit <= MapFuelLimit) {  //enforce the most restrictive results from fuel tables
    //fuelGov = constrain(fuelGov, 0, RpmFuelLimit);  //un-comment to enable table
  }
  //fuel vs BOOST
  else {
    //fuelGov = constrain(fuelGov, 0, MapFuelLimit);  //un-comment to enable table
  }

  //********** Anti stall ****** mostly un-tested need a manual transmission.
  if (RPM < 740) {
    //fuelGov = fuelGov + 50; //quick response
    fuelGov = fuelGov + constrain(((740 - RPM) * 3), 0, 300);  //porpotional response
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
  fuelGov = (fuelGov + lastFuelGov) / 2.0;  //rolling average fuelGov output for smoother response
  lastFuelGov = fuelGov;

  //*********turn off a cylinder for testing*****
  if (RPM > 600 && kill > 0 && kill == active_cyl) { fuelGov = 0; }
  fuelGov2 = constrain(fuelGov, 0, MaxFuel);  //Atomic fuelGov2 (output) may get destroyed in EOI funtion
  //fuelGov2 = TPS_scaled; //un-comment to disable gov.
}  //govenor done


//******** load calculator **********
//calc the fuel plungers stroke %,  comp for the non linear ramp of the cam donut. Sinusoidal for now.
float LoadCalc(uint32_t fuel) {
  float stroke;
  float _load;
  for (int i; i < 9; i++) {
    if (fuel >= strokeCorrection[i]) {
      stroke = (12.5) * i;
      float r = (fuel - strokeCorrection[i]) / (float)(strokeCorrection[i + 1] - strokeCorrection[i]);
      stroke = (12.5 * r) + stroke;
    }
  }
  //fuel plungers are 7.5 mm dia. x 3.5 mm stroke x 3 plungers. volume 124 ml = 3*((7.5 / 2) * pi * 3.5)
  milliLiters = (124 * stroke) / 100.0;  //injected quanity per cylinder  //Nissan 4 cyl 77 mL ?

  //cam lobe Load correction from above now scaled to 0 every 200 RPM when at no load in neutral
  int l = RPM / 200;  //table entry for every 200 RPM
  l = l - 5;
  l = constrain(l, 0, 20);  //1000 to 4000 RPM
  int base_fuel = BaseFuel[l];
  if (RPM % 200 && l < 20) {  //if odd RPM then split the difference
    base_fuel += (BaseFuel[l + 1] - BaseFuel[l]) / 2;
  }
  _load = mapf(fuel, base_fuel, 4095.0, 0.0, 100.0);
  _load = constrain(_load, 0.0, 100.0);  //used for Timing advance table
  return _load;
}

int MapFuelLookup(int boost) {
  //Boost based fueling
  int Limit;
  int j = boost / 2;        //table entry for every 2 psi
  j = constrain(j, 0, 22);  //44 psi max
  Limit = MapFuel[j];
  if (boost % 2 && j < 22) {  //if odd psi then split the difference
    Limit += (MapFuel[j + 1] - MapFuel[j]) / 2;
    return Limit;
  }
}

int RPMFuelLookup(int rpmAvg) {
  int i = (rpmAvg / 100) - 5;  //table indexs every 100 RPM staring at 500 to 4000
  i = constrain(i, 0, 35);
  //float r = (float(rpmAvg % 100) / 100);  //remainder, percent between table entries used for interpolation
  return RpmFuel[i];  //Fuel Vs RPM limiter
}

//***************************** Timing section ******************************
void timingCompute(void) {  //this fuction is called on every cycle
  //******** compute injection start relative to top dead center, in degrees.*******
  //this is the phase relationship between the cam donut and pump shaft
  static float oldTiming;
  //Timing = (TrefTime - (phaseB[19] + bounceTime)) / (float)phaseB[36];
  //Timing = (TrefTime - (phaseB[19] + 1100)) / (float)phaseB[36];
  //1100 ticks or 550 uS line delay. Speed of sound in diesel about 32 inches from pump to injector.
  //Timing = Timing * 120;        //one index to index tooth is 120 degrees phaseB[36] is the period
  //Timing = Timing - 20.0;       //because the timing referece signal happens at 20 degrees after TDC.
  //Timing = constrain(Timing, 0, 40);
  //Timing = (Timing + oldTiming) / 2.0;
  //oldTiming = Timing;                       //average timing for some smoothing

  //********** new method ***********
  if (TrefTime < phaseB[19]) {                                                   //TrefTime occured after the Index tooth. Happens at high REVs and advanced timing
    Timing = (phaseB[19] + 1100 - TrefTime) / (float)(phaseB[23] - phaseB[21]);  //timer 2 reset / rolled over
  } else {
    Timing = (TrefTime - (phaseB[19] + 1100)) / (float)(phaseB[23] - phaseB[21]);  //timer 2 normal
  }
  Timing = Timing * 6.0;  //6 degrees per tooth, notch period
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
  } else if (TimError > 12 && RPM > 1800) {
    timingDuty = 600;
  }
  if (RPM > 0 && RPM < 600) {
    timingDuty = 990;  //keep timing at minimum while starting
  }
  if (RPM > 0 && RevCounter < 100) {
    //timingDuty = 700; //let the timing control system stabilize.
  }
  if (RecTimingDuty > 0) { timingDuty = RecTimingDuty; }  //for testing use serial rec "timduty xxx" to specify a duty cycle
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
  if (in_max == in_min) {                                                      //catch divide by 0
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
  DMA1_Channel1->CMAR = (uint32_t)&AD1result[0];  //to here in memory
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
  DMA2_Channel1->CMAR = (uint32_t)&AD2result[0];  //to here in memory
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

  idle_valid = (digitalRead(idle));  //check idle valid signal. wired to PA11 5v when resting.

}  //get sensors done

void threshold(void)
// zero crossing dector circuit threshold adjustment, phase A and phase B from the VP44 toothwheel sensor.
{  //slowly adjust thresholds, Ball park by Phase A,B signals >> very low pass filter (DC component) >> ADC scan + averaging
  static int Ki = 16;
  static uint16_t ThrA, ThrB, lastThrA, lastThrB;
  static uint32_t lastThres;

  if (millis() - lastThres > 4) {
    lastThres = millis();
    (crank) ? Ki = 16 : Ki = 8;
    ThrA = ThrA + ((AD2result[2] - lastThrA) / Ki);
    lastThrA = ThrA;
    ThrB = ThrB + ((AD2result[3] - lastThrB) / Ki);
    lastThrB = ThrB;
    DAC->DHR12R1 = ThrA;
    DAC->DHR12R2 = ThrB;
  }
}  //threshold done

void SetAmps(void) {
  //increase duty cycle with battery V decrease, fine tune amps with feedback
  //max duty is 600 which is fully on = max current
  int amps;
  float i = 0;
  amps2Manual = constrain(amps2Manual, 10.0, 13.0);
  amps4Manual = constrain(amps4Manual, 7.0, 10.0);

  if (amps_hold2 < 1700) {
    amps_hold2 = amps_hold1;  //incase of very short duration
  }

  BATv = float(SOL) / 203;  //219 for 2.2k resistor, 203 for 2k. R38
  BATv = BATv - 0.12;       //cal factor +.1 ,or -.12
  BATv = constrain(BATv, 0, 20);

  if (RPM < 600 || RevCounter < 50) {  //a little extra at startup
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
  if (event == 4 && TIM2->CNT - SOI > 1000)  //500 microseconds after SOI
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
      DTCpending = 122;  //APPS voltage too Low
      errCnt1 += 2;
    }
    if (TPSv > 4.0) {
      DTCpending = 123;  //APPS voltage too High
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

    //   if (!eepromValid) {
    //     DTC = 1691;  //P1691 VP44 Controller Calibration Error.
    //    }
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
  TIM2->CCMR1 = FORCE_LOW;      //FORCE low to drive inj sol off
  pinMode(failsafe, OUTPUT);    // Fuel system relay drive; end high Z state which allows hardware timeout and force off by software
  digitalWrite(failsafe, LOW);  // Turn off Fuel system relay. may already be off by hardware fault detection circuit
  Serial.print(" shut off DTC P0");
  Serial.println(DTC);  //show why it shut off
}

void ignitionOff(void) {
  //this doesn't work right. 5 volts dies off too fast to save
  //igniton key state. Store threashold values, DTC to emulated EEPROM when key turned off.
  //write sequence takes too long if engine running and it dies.
  //extend the watchdog.
  if (digitalRead(IGN)) {
    return;  //key on do nothing
  }
  TIM2->CCMR1 = FORCE_LOW;      //FORCE low to drive inj sol off
  pinMode(failsafe, OUTPUT);    // Fuel system relay drive; end high Z state which allows hardware timeout and force off by software
  digitalWrite(failsafe, LOW);  // Turn off Fuel system relay. may already be off by hardware fault detection circuit
  //Serial.println(" Saving data ");
  //Serial.println(" Off ");
}  //ignitionOff done

void EEPROM_read() {
  int A2M, A4M;
  EEPROM.get(208, checksum);
  //Serial.println(checksum);
  if (checksum == 0 || checksum == (2 ^ 32) - 1) {
    Serial.println(" Blank EEPROM, using defaults. ");
    return;
  }
  EEPROM.get(204, storedDTC);
  EEPROM.get(212, TPS_rest);
  EEPROM.get(216, TPS_floored);
  EEPROM.get(220, kp);
  EEPROM.get(224, ki);
  EEPROM.put(228, kd);
  EEPROM.get(232, droop);
  EEPROM.get(236, FuelBias);
  EEPROM.get(240, A2M);
  EEPROM.get(244, A4M);
  amps2Manual = float(A2M / 10.0);
  amps4Manual = float(A4M / 10.0);
  if (checksum != TPS_rest + TPS_floored) {
    Serial.println(" invalid checksum, using defaults ");
    while (1)
      ;  //wait here for watchdog to reset and use values from var. declarations
  }
}

void EEPROM_save(int defaults) {
  if (defaults == 2) {
    Serial.println(" Restoring to defaults, please wait. ");
    EEPROM.put(208, 0);  //set the checksum to zero and force a watchdog reset.
    while (1)
      ;
  }
  Serial.println(" Saving data ");
  EEPROM.put(212, TPS_rest);
  EEPROM.put(216, TPS_floored);
  EEPROM.put(220, kp);
  EEPROM.put(224, ki);
  EEPROM.put(228, kd);
  EEPROM.put(232, droop);
  EEPROM.put(236, FuelBias);
  int A2M = amps2Manual * 10;
  int A4M = amps4Manual * 10;
  EEPROM.put(240, A2M);
  EEPROM.put(244, A4M);
  if (defaults == 3) {
    DTC = 0;
    Serial.println(" Erasing DTCs ");
  }
  EEPROM.put(204, DTC);
  EEPROM.put(208, TPS_rest + TPS_floored);  // a simple checksum
}

void OPAMP2_set(void) {        //This is the injection solenoid current blip detector
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
}  //DAC_setup done

void REC() {
  //process any incoming bytes from the serial port.
  // read the incoming characters
  //separate numbers from Alpha
  //build number or alpha string from incoming stream
  //end when newline <ENTER> is received

  static char incomingChar;
  static int numbersIn, saveLevel;
  static String instring, lettersIn, inNum;

  while (Serial.available() > 0) {
    incomingChar = Serial.read();
    if (isDigit(incomingChar)) {
      inNum += incomingChar;  //concatonate this is a string not interger
    }
    if (isAlpha(incomingChar)) {
      instring += incomingChar;
    }
    if (incomingChar == '?') {  //print the help menu
      help = 1;
    }
    if (incomingChar == '\n' && !help)  //<enter> end of input
    {
      lettersIn = instring;
      numbersIn = (inNum.toInt());  //now string is converted to interger
      Serial.print(lettersIn);
      Serial.print(" ");
      Serial.println(inNum);
      instring = "";
      inNum = "";
    }
  }  //end of while serial

  if (lettersIn == "rel") {
    releaseTime = numbersIn;
    releaseTime = constrain(releaseTime, 0, 2000);
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
    printInterval = 10000;
    Serial.println(" Save? Y or N ");
    saveLevel = 1;
  }
  if (lettersIn == "restore") {
    printInterval = 10000;
    Serial.println(" Restore? Y or N ");
    saveLevel = 2;
  }
  if (lettersIn == "erase") {
    printInterval = 10000;
    Serial.println(" Erase? Y or N ");
    saveLevel = 3;
  }
  if (lettersIn == "cylinder") {
    kill = numbersIn;
  }
  if (lettersIn == "timduty") {
    RecTimingDuty = numbersIn;
  }
  if(lettersIn == "offset"){
    offset = constrain(numbersIn, 0, 600);
  }
  if (lettersIn == "y") {
    EEPROM_save(saveLevel);
  } else if (lettersIn == "n") {  //abort
    saveLevel = 0;
    Serial.println(" Aborted ");
  }
  lettersIn = "";
  numbersIn = 0;

}  //REC done

void Help() {
  //printing all this at once can cause an engine mis fire so break it up.
  static int group;
  if (group == 0) {
    Serial.println();
    Serial.println(" Syntax; letters <space> numbers <enter> ");
    Serial.println(" ta or tb or tc; manual deadtine. EX: ta 400 sets 1st dead time to 400 microseconds. ");
    Serial.println(" kp or ki or kd; govorner pid parameters times 100. EX: kp 220 sets porpotional gain to 2.2 ");
  }
  if (group == 1) {
    Serial.println(" droop 0 to 500; allows the idle to droop under load. 0 max droop 500 min droop. ");
    Serial.println(" fuelbias; nominal fuel value for stable idle in neutral. 1500 default ");
    Serial.println(" ib or id; amps times 10 for solenoid part b or part d (holding). EX: id 75 sets holding current to 7.5A. ");
    Serial.println(" timduty 200 thru 900; manually setting valve duty cycle. lower value advances. EX timduty 600 is typical. 0 is auto. ");
  }
  if (group == 2) {
    Serial.println(" cylinder 0 thru 6; stop injection on cylinder for testing. EX: cylinder 0 normal mode. ");
    Serial.println(" tpsl or tpsh; tpsl records  the resting throttle pos. tpsh records WOT. key on engine off. ");
    Serial.println(" offset 0 thru 600 fine tune SOI bounce with edge 19, 300 is centered. ");
    Serial.println(" save; saves parameters to EEPROM ");
    Serial.println(" restore; sets parameters to defaults ");
    Serial.println(" erase; clears stored DTCs. ");
    Serial.println();
  }
  group++;
  if (group > 2) {
    group = 0;
    help = 0;
  }
}

void printOut(void) {  //split up printing so It won't delay things to much
  //caution avoid excess printing
  static uint32_t lastPrint, lastHelp;
  static int prtGrp;

  if (millis() - lastPrint > printInterval) {
    if (prtGrp == 0) {
      Serial.print(" RPM ");
      Serial.print(RPMavg);
      Serial.print(" BOOST ");
      Serial.print(BOOST);
      Serial.print(" BATv ");
      Serial.print(BATv);
      //Serial.print(" kpL ");
      //Serial.print(kpll);
    }
    if (prtGrp == 1) {

      //Serial.print(" TPS ");
      //Serial.print(TPS);
      //Serial.print(" RpmFuel ");
      //Serial.print(RpmFuelLimit);
      //Serial.print(" ampref ");
      //Serial.print(amps_ref1);
      //Serial.print(" BATv ");
      //Serial.print(BATv, 1);
    }
    if (prtGrp == 2) {
      //Serial.print(desiredTimNL);
      //Serial.print(" ");
      //Serial.print(desiredTimFL);
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
      Serial.print(" Load ");
      Serial.println(Load);
      //Serial.print(" TPS scaled ");
      //Serial.print(TPS_scaled);

       //Serial.print(" CAN ");
       //Serial.println(CAN->sFIFOMailBox[1].RDLR, HEX);

      if (DTC > 0 && DTC < 999) {
        //Serial.print(" DTC P0");       //show  error or why it shutdown.
        //Serial.println(DTC);
        DTC = 0;
      } else if (DTC > 0 && DTC > 999) {  //print without a leading 0
        //Serial.print(" DTC P");       //show  error or why it shutdown.
        //Serial.println(DTC);
        DTC = 0;
      }
    }
    prtGrp++;
    if (prtGrp > 2) {
      prtGrp = 0;
      printInterval = 500;
      lastPrint = millis();
    }
  }
  if (millis() - lastHelp > 15000) {
    Serial.println(" Enter ? for help menu ");
    lastHelp = millis();
  }
}  // printOut done

void CANsetup(void) {
  GPIOB->AFR[1] |= 0x00000099;  //AF9;  re-map CANRX - PB8, CANTX - PB9
  GPIOB->MODER |= 0x000A0000;   //portB  pins 8 & 9 selected as alternate functions
  CAN->MCR = 1 << RESET;        //reset the CAN perp
  delay(1);
  CAN->MCR = 0x00000011;  //abom on, no sleep init, NART one time transmit
  CAN->BTR = 0x401C0008;  //250K baud pre=8, ts1=13, ts2=2, sjw=1
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

  CAN->MCR = 0x00000040;  //activate CAN in ****LOOPBACK MODE ******//this doesn't control anything yet.

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
  //internal CPU core temp, used to detect engine bay temp on cold days
  int32_t Temperature = (int32_t)(((110.0f - 30.0f) / ((float)(temp110) - (float)(temp30))) * ((float)(Tsample) - (float)(temp30)) + 30.0f);
  Temperature = Temperature - 10.0f;
  return Temperature = ((Temperature * 9) / 5) + 32;
}

//********* end of program ********