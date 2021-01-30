# AD5293 for Arduino
A library to operate AD5293 digital potentiometer.

Uses SPI library for communication with the device.

*by Grzegorz Wielgoszewski, http://nanometrologia.wielgoszewski.pl*

# General information
The **AD5293** library provides essential functions to operate the 10-bit AD5293 digital potentiometer from an Arduino device via SPI interface.

AD5293 (see: https://www.analog.com/en/products/ad5293.html) is a single-channel, 1024-position (10-bit) digital potentiometer with calibrated 1-% nominal resistor tolerance, available with nominal resistance of 20 kΩ, 50 kΩ and 100 kΩ, manufactured by Analog Devices. According to device datasheet, the nominal wiper resistance is 60 Ω, which is included in calculations performed within the **AD5293** library.

Each physical chip, connected to Arduino, should be using a separate **AD5293** instance. 
The **AD5293Class** provides methods to control the potentiometer as a voltage divider or a rheostat (‘in-line’ variable resistor), allowing obviously to change the wiper position arbitrarily as well. 

Daisy-chain connected devices can also be handled using **AD5293** library. *To be checked, though.*

# Hardware compatibility
Any Arduino device with SPI capabilities should be enough. Please remember, however, that you may need a free digital output line per each AD5293 (for the *!SYNC* pin).
In my applications, I was controlling the AD5293 potentiometer from an Arduino Zero and an Arduino Micro.
