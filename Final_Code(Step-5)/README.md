# Navigation: Drawing Shapes with Receiver Station
This step has the same set-up for the Transmitter Station as in Step-4, with multiple configuration changes to the Receiver Station.

The Receiver station is now mounted on a Differential drive robot, with the Receiver Arduino controlling the bot's motor drivers. The Receiver Arduino is connected to 2 Ultrasonic Receivers placed on the head and tail of the Robot seperated by a distance of about 20 to 30cm. This 2 Ultrasonic receivers will not only help obtain the 2D position of the bot with better accuracy, but its orientation as well.

With this functionality successfully established, the bot can now navigate to any point on the ground, when the (x, y,2D_angle) coordinates & the bot's heading angle are provided.

Henceforth, the codes for the receiver station tracing a square forwards & backwards, and writing the letters 'RMI' (the name of my Robotics Club) on the floor are provided in this folder. The common code to be uploaded to the transmitter station in all the cases, is also provided.
