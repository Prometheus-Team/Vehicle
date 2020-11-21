# Vehicle
Contains the navigation component of the system
1. Vehicle API - connects the arduino with the computer/Raspberry
2. Manual control - allows the user to control the vehicle manually
3. Localization - estimate the location of the vehicle from sensor readings
4. Map builder - creates a 2D map of the area for navigation from range sensor
5. Shortest and smooth path - find the shortest and smooth path between two given points

Dependency
- ROS
- Python serial - to connect with Arduino

To start the system write the following on the terminal

`roslaunch vehicle_control control.launch`