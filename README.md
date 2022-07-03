# H7SDR
Software defined radio on a STM32H7 microcontroller.
This is a complete AM shortwave radio receiver using only the built-in peripherals of a STM32H743 microcontroller, a passive bandpass-filter AND NOTHING MORE!
The internal ADC of the STM32H743 is used for digitizing the antenna signal and the internal DAC of the STM32H743 is used for putting out the audio signal.
Everything in between is plain digital signal processing (DSP).
The default settings are suited for the reception of radio DARC in the 49-m-band at 6070 kHz 

Primitive, but working (and very instructive, as I think)!

compiled with STM32CubeIDE 1.6.0

- connect an antenna (i.e. a long piece of wire, maybe a few meters long) to a bandpass filter for 6070 kHz
- connect bandpass filter output to ADC input on PF11
- connect a loudspeaker amplifier to DAC output PA4
- wait for a RADIO DARC transmission, sunday 11:00 CEST and listen (reception of Radio DARC should be possible throughout Europe and beyond)
- or wait for some other transmission at 6070 kHz
- if neccessary, increase volume by increasing global variable dac_gain

for further information: {www.dm1cr.de}
