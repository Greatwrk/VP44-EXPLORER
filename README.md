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



