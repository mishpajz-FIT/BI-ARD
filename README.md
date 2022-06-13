# BI-ARD

Interactive applications with Arduino course at FIT CTU.

The repository contains the main project of the course for Arduino (UNO).

## Project

### Description
**Indoor air quality mesurement station**

Poorly ventilated rooms often accumulate harmful substances and dust that can cause health problems. 
Elevated CO2 concentrations lead to headaches and reduce concentration, which is a problem in heavily insulated bedrooms, for example. House dust, in turn, can cause allergic reactions and impair breathing. The quality of the air we breathe is also affected by its humidity.

The aim of this project is to create a platform that can measure these variables.

The indoor device will be able to measure the CO2 concentration in the room, the amount of dust, temperature and humidity. These variables will be measured periodically and displayed on the device's display. At the same time, the device will indicate the overall measurement status from the viewpoint by means of an LED. Last but not least, the device will have internet connectivity and will be able to send this measured information so that it can be easily collected.

### Structure

#### Documentation
The used components, wiring and settings are described in the file [docs/hw-documentation.pdf](docs/hw-documentation.pdf).

The actual control of the device is described in the file [docs/user-manual.pdf](docs/user-manual.pdf).

#### Code

All code executable on the Arduino can be found in the *src* directory.

#### Libraries

The libraries used in this project are available in the *libs* directory. All libraries are provided with a license that allows their use in this project.


