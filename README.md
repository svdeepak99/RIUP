# RIUP: RF Synced Indoor Ultrasonic Positioning & Navigation
This project aims to achieve indoor positioning and navigation, employing **Time of Flight (TOF) measurements** using **Ultrasonic Transmitter and Receiver** Modules.

**Note: The README files explaining each step can be inside the step folders present in this root directory.**

The setup would consist of a Transmission Station, whose operation is similar to the GPS Satellites orbiting on the Geostationary Orbits above the Earth's Surface. 
Then there will be a mobile Receiver Station, similar to any GPS Device on the Earth's Surface, which would obtain signals from the GPS Satellites and estimate its own position through trilateration.

However, unlike GPS Satellites, the Transmission Station would have ultrasound transmitters that would continually transmit ultrasonic pulses, which would then be captured by the Receiver Station, for TOF measurements. Likewise, the Receiver Stations would posses Ultrasound receiver modules, unlike conventional GPS Devices.
In addition to these, the Transmitter Station would have a **433MHz Radio Frequency (RF)** transmitter module, and the Receiver Station would have a 433MHz RF receiver module. These RF transducers serve the role of syncing the clock between the transmitter and receiver stations, in order to perform the TOF measurements on the Ultrasound signals. This is done since radio signals travel a lot faster than ultrasound signals. Hence, it can be used to instantly start the TOF clock on the receiver station when the transmitter station beams an ultrasonic pulse.

The microcontroller boards used throughout the project is an Arduino UNO board for the transmitter station and another Arduino UNO board for the receiver station. **I wrote the firmware entirely in Embedded C** to make full use of the hardware and improve the execution speeds to handle time-sensitive operations such as TOF measurements, time syncing using RF Signals, trilaterations and navigations simultaneously on the same board. This also reduced the cost of the required hardware, by a significant margin.

The components used while making this prototype are as follows:
1) 2 x Arduino UNO R3 Boards                    - https://store.arduino.cc/usa/arduino-uno-rev3
2) 4 x US-100 Ultrasonic Distance Sensor Module - https://robu.in/product/us-100-ultrasonic-sensor-distance-measuring-module-temperature-compensation/
3) 1 x 433MHz Transmitter & Receiver Modules    - https://www.amazon.in/Robocraze-Wireless-Transmitter-Receiver-Quadcopter/dp/B07MY67BDB/
4) 1 x L298N Motor Driver Module                - https://robu.in/product/l298n-2a-based-motor-driver-module-good-quality/
5) 1 x HC-05 Bluetooth Module                   - https://www.amazon.in/xcluma-Wireless-Bluetooth-Transceiver-Arduino/dp/B071HWFYJP/
