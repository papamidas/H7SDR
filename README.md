# H7SDR
Software defined radio on a STM32H7 microcontroller.

Primitive, but working!

compiled with STM32CubeIDE 1.6.0

- connect an antenna (i.e. a piece of wire, a few meters long) to a bandpass filter for 6070 kHz
- connect bandpass filter output to ADC input on PF11
- connect a loudspeaker amplifier to DAC output PA4
- wait for RADIO DARC transmission, sunday 11:00 CEST and listen (reception of Radio DARC should be possible throughout Europe and beyond) 
- if neccessary, increase volume by increasing global variable dac_gain

for further information: {www.dm1cr.de}
