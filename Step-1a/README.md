# Callibrate Arduino Oscillators via Cable (wired)
These codes are used to find the ratio between the frequencies of quartz oscillators in the transmittor and receiver Arduinos (which is ideally expected to be 1:1, but would deviate slighly in actual boards), to enter the counter compensation factor in code, before Step-2, so that the counter time in one of the boards does not lead/lag in practice.

This callibration is done by physically connecting both the boards by means of jumper wired, for callibration signal transmission.

Note: Step-1b is not needed if this step is performed and vice versa.
