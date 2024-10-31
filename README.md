# VP44 EXPLORER
My project replaces the control board that sits on top of the pump with a remote board.
The project uses a STM32f303 Nucleo dev. board and uses Arduino IDE.
A custom PCB was made in two parts one is main logic and sensor inerface. The second part is the 
solenoid drivers and power supply. The current driver board uses high speed switching buck converter
to operate the fuel solenoid. A linear drive board is under development.
The linear method is the OEM style. It generates a lot of heat and is cooled by the fuel flowing
under it in the pump housing.
The switching style results in significantly less heat. The downside is the switching signal can
mask the current monitoring of the solenoid. As the armature plunger moves it causes a change
in inductance which causes a small current blip. The blip is helpful to determine the solenoid
deadtime.

The VP44 shaft has a tooth wheel the spacing is 3 degrees of crankshaft rotation. Every 60 degrees (120 crank) is a pair of staggered teeth these I refer to index tooth. A dual megneto-resistive sensor
senses the teeth as they pass by and creates two sinewaves one leads I refer to as phase A the other 
follows approx. 90 degrees I refer to as phase B. Both signal are zero cross detected by internal opAmps in the STM32 and then fed to timer captures. Software detects the index tooth and resets a tooth edge counter to zero. On the tooth edge 19 of the B phase the radial injection plungers are at their maximum extension and begin following the cam ring into compression. At exactly this position the injection solenoid valve must transitioning to fully closed position. This is start of injection SOI after a variable amount degrees the valve de-enrgizes this is the end of injection EOI. Since the solenoid has deadtime on both turn on and turn off the switching must occur before edge 19 actually happens. This is done by cumputing shaft postion, velocty, and acceleration to predict the relative time from
a current tooth edge to edge 19. Obviously the closer we are to the edge 19 event the better the pedicton will be, but at higher RPMs the deadtime of the solenoid results in required predicton to
occur many degrees in advance else the valve will be closed to late or early. That results in late injection and reduced fuel delivery and hazy smoke. The valve must be de-energized before the moment of desired EOI because of mechanical delay I call this release time, it is about 850 micro seconds. The deadtime is mostly dependant on the battery voltage and must be caculated. A full current is applied to get the valve moving then the current is reduced to prevent the valve from slamming on its seat and bouncing. Finally the current is reduced to a safe level for the majority of the injection event. The switch off voltage spike must be clamped at a higher voltage to allow for a fast release so
simple clamping / recirculating methods are too slow. Some manual time and current values must be 
determined with an ossciloscope and are stored.

Anyone who has kick started or rope started a small engine has expirenced the rapid fluctuations in 
crankshaft velocity. These also occur in a running engine but are greatly reduced by the flywheel mass
and inertia. These variations have to be accounted for in both the SOI time and EOI time if not the
engine will start to run rough which makes the variations worse and rusults in timing and delivery
errors and possible stalling or run away.

The timing advance is determined by the phase relationship of the cam ring to crankshaft postion.
The ECU decodes the crankshaft postion sensor and cam sensor and creates a reference signal that
occurs at 20 degrees after top dead center for each cylinder. In my project the decoding is done internally and the ECU is not required. The cam ring moves by a hydraulic piston which is controlled by another solenoid valve. It is PWM controlled by the micro. The actual timing is calculated by the 
VP tooth signal in relation to the CKP derived signal. A software servo contol loop adjust the PWM to
to match the computed timing with the desired timing contained in usual look up tables.

For now the TPS, MAP and CKP are all thats required. A CAN interface is on the board. The CAN message
format and protocol for the VP44 comms has never been fully documented but most of the Fueling and
timing instuctions are well known. That could be a future addition.

The Schematic and PCB were made with KiCad. The enclosure was designed with FreeCad. The code has 
some low level stuff for some of the periphials that at the time were not supported by STM32duino.



