# VP44 EXPLORER

My project replaces the control board that sits on top of the pump with a remote board.
The project uses an STM32f303 Nucleo dev. Board and I used Arduino IDE.
A custom PCB was made in two parts. One is the main logic and sensor interface. The second part is the 
solenoid drivers and power supply. The current driver board uses a high-speed switching buck converter
to operate the fuel solenoid. A linear drive board is under development.
The linear method is the OEM style. It generates a lot of heat and is cooled by the fuel flowing
under it in the pump housing. The switching method results in significantly less heat. The downside is the switching signal can
mask the current monitoring of the solenoid. As the armature plunger moves it causes a change in inductance which causes a small current blip.
The blip is useful to determine the solenoid dead time.

The VP44 shaft has a toothed wheel, the tooth spacing is 3 degrees of crankshaft rotation.
Every 60 degrees (120 at crankshaft) is a pair of staggered teeth.
These I refer to as the index tooth.
A dual magneto-resistive sensor with a back bias magnet senses the teeth as they pass by and creates two sinewaves.
The one that leads I refer to as phase A. The second follows by approximately 90 degrees, I refer to it as phase B.
Both signals are adaptive zero crossing detected by internal opAmps and the DACs in the STM32 and then fed to timer captures.
The software detects the index tooth region and resets a tooth edge counter to zero.
On the tooth edge 19 of the B phase the radial injection plungers are at their maximum extension.
As they follow the cam ring into compression it's at this position the injection solenoid valve must be fully transitioned to the closed position.
This is the start of injection SOI. After a variable amount of degrees, the valve de-energizes this is the end of injection EOI.

Since the solenoid has deadtime on both turn on and turn off the switching must occur before edge 19 or EOI actually happens.
This is done by computing shaft position, velocity, and acceleration to predict the relative time from a current tooth edge to edge 19.
Obviously, the closer we are to the edge 19 event the better the prediction will be, but at higher RPMs the deadtime of the solenoid results in the required prediction to
occur many degrees in advance else the valve will be closed too late or early.
That results in late injection, reduced fuel delivery and hazy smoke.
The valve must be de-energized before the moment of desired EOI because of mechanical delay I call this release time, which is about 850 microseconds.
The dead time is mostly dependent on the battery voltage and must be calculated often. 
A full current is applied to get the valve moving then the current is reduced to prevent the valve from slamming on its seat and bouncing.
Finally, the current is reduced to a safe level for the majority of the injection event.
The switch-off induction spike at EOI must be clamped at a higher voltage and low current for a fast release.
The magnetic field is determined largely by A x N turns, so keeping current low speeds the armature release.
Simple clamping / recirculating methods are too slow. A zener diode RC circuit is used. 
Some manual adjustments to times and current values must be determined with an oscilloscope and are stored.

Anyone who has kick-started or rope-started a small engine has experienced the rapid fluctuations that occur in 
crankshaft velocity. These also occur in a running engine but are greatly reduced by the flywheel mass
and inertia. These variations have to be accounted for in both the SOI time and EOI time if not the
engine will start to run rough which makes the variations worse and results in timing and delivery
errors and possible stalling or running away.

The timing advance is determined by the phase relationship of the cam ring to the crankshaft position.
The ECU decodes the crankshaft position sensor and cam sensor to create a reference signal that
occurs at 20 degrees after top dead center for each cylinder. Later engines removed the CKP and changed the camshaft gear. 
With my project, the decoding is done internally and the ECU is not required. 
The cam ring moves by a hydraulic piston which is controlled by another solenoid valve. It is PWM controlled by the MCU. 
The actual timing is calculated by the VP tooth signal phase relationship to the CKP-derived signal. 
A software servo control loop adjusts the PWM to match the computed timing with the desired timing contained in the usual look-up tables.

For now, the TPS, MAP and CKP are all that's required. The CMP is optional. A CAN interface is on the board and a CAN interface is being developed.
The Schematic and PCB were made with KiCad. The code has some low-level stuff for some of the peripherals that at the time were not supported by STM32duino.

If you build this I suggest using a 2 ohm 50 W resistor in place of the fuel solenoid for testing with a scope. The resistor can later be placed
in series with the solenoid to protect it on the bench while sweep testing at RPMs and Fuel levels. Listen for any malfunctions.
The power transistors must be isolated from the heatsink with silicon insulators. Always test isolation with an DVM.

# See it run [HERE](https://www.youtube.com/watch?v=DHwnkv7ZLJk)