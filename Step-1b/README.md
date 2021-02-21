# Callibrate Arduino Oscillators via Radio Signals (wireless)
These codes are used to find the ratio between the frequencies of quartz oscillators in the transmittor and receiver Arduinos (which is ideally expected to be 1:1, but would deviate slighly in actual boards), to enter the counter compensation factor in code, before Step-2, so that the counter time in one of the boards does not lead/lag in practice.

This callibration is done wirelessly with Radio Signals, by using 433 MHz RF Transmitter & Receiver Modules.

Note: Step-1a is not needed if this step is performed and vice versa.
