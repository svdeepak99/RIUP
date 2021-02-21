# Wireless 2D Positioning with 2 Ultrasound transmitters, by triangulation
This code successfully estimates the (x,y) coordinate of a bot/receiver station, when the coordinates of the 2 Ultrasound transmitters are provided. The time is synced by using RF signals similar to Step-3, and an RF signal is transmitted before Ultrasound is transmitted from each of the sensors from the transmitter station.

The stationary transmitter station has the transmitter Arduino connected to the RF transmitter, and 2 Ultrasound transmitters which are placed perpendicular near the ceiling of the room facing the mobile receiver station downwards.

The mobile receiver station has the receiver Arduino, connected to an Ultrasound receiver and the RF receiver. This mobile station calculates the distance from the Ultrasound transmitter, using the TOF values obtained and then uses these values to calculate it's (x,y) coordinates on the floor through triangulation, thereby achieving 2D positioning.
