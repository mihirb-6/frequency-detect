# frequency-detect

The culmination of an Electronics Measurement Techniques course at FIT.
An electronics project combining concepts like gain, filters, operational amplifiers, and frequency detection.

Project goals:
- Assemble a band-pass filter and apply filtering concepts to calculate the correct frequency cut-off for low and high cutoffs.
        - The band-pass will filter out the noise from a less-than-ideal signal
- Assemble an inverting amplifier circuit to boost the original signal for easy peak detection, aim to achieve a gain of 10.
- Write a C++ script that does the following:
        - Obtain a signal from an analog input (could be a phone, laptop, etc., playing a musical note(s))
        - Conduct a Fast Fourier Transform using the ArduinoFFT library
        - Identify peak frequencies, account for multiple peaks in the case of multiple notes played simultaneously
        - Activate a red, green, or blue LED depending on the octave of the musical note, ensuring multiple LEDs can activate simultaneously           in the case of multiple notes played simultaneously
