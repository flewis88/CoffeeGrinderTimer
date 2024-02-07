# GRINDY BOI - Coffee Grinder Timer
Arduino based Timer for grinding coffee, with estimated dose weight

- Rotary encoder rotation adjusts time duration.
- Pushing encoder powers the relay for set duration, and stores changed duration to EEPROM
- Holding Calibration button allows rotary encoder to adjust the dose weight for the given duration, changing the time/weight ratio (calibration is a single reference point, and assumes dose weight is linerarly proportional to grind time)
- Releasing the Calibration button stores dose weight to EEPROM

Fritzing diagram coming soon
Arduino Uno
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

![IMG-20240118-WA0000](https://github.com/flewis88/GRINDY-BOI-Coffee-Grinder-Timer/assets/52615288/c7b83013-9a97-42ed-88c7-4977b2e4ee06)
![IMG-20240118-WA0001](https://github.com/flewis88/GRINDY-BOI-Coffee-Grinder-Timer/assets/52615288/6b2da7fa-3a97-42bf-a082-b7e2dea2ac8f)
![IMG-20240118-WA0002](https://github.com/flewis88/GRINDY-BOI-Coffee-Grinder-Timer/assets/52615288/979723b0-ca8e-4404-ba08-3fab4080f3a5)
![IMG-20240118-WA0003](https://github.com/flewis88/GRINDY-BOI-Coffee-Grinder-Timer/assets/52615288/bc3a29c2-f113-4d07-962a-4dd8b1f22b81)
![IMG-20240118-WA0004](https://github.com/flewis88/GRINDY-BOI-Coffee-Grinder-Timer/assets/52615288/3f26beb0-966e-407f-b16a-378030318756)

