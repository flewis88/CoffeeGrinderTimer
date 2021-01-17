# GRINDY BOI - Coffee Grinder Timer
Arduino based Timer for grinding coffee, with estimated dose weight

- Rotary encoder rotation adjusts time duration.
- Pushing encoder powers the relay for set duration, and stores changed duration to EEPROM
- Holding Calibration button allows rotary encoder to adjust the dose weight for the given duration, changing the time/weight ratio (calibration is a single reference point, and assumes dose weight is linerarly proportional to grind time)
- Releasing the Calibration button stores dose weight to EEPROM

Fritzing diagram coming soon
Arduino Nano
LCD 16x2 over I2C
Rotary Encoder connections
 -Clock -> D2
 -DT  -> D3
 -SWITCH -> D4
 Calibration Button -> D5
 Relay -> D6
 
 Relay wired to switch active line of extension cable, between wall socket and coffee grinder (left in on position)

Commenting is limited, and code is kinda cobbled together. I may refine it.
Video coming after final assembly.
