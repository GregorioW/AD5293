/*! \file  SimpleAD5293.ino
 *  \brief  Example of usage of the AD5293 library.
 * 
 *  \details  Basic example of using AD5293 library to control AD5293 digital potentiometers.
 *
 *  \date Created: 30 Jan 2021
 *  \author Grzegorz Wielgoszewski \<Nanometrologia@Wielgoszewski.pl\>
 */

#include <AD5293.h>

// Pin configuration.
// These pins were used once the AD5293 library was used with Arduino Micro.
const uint8_t pinCS = 8;      //!< Arduino pin connected to AD5293 !SYNC (!CS) pin.
const uint8_t pinRDY = 9;     //!< Arduino pin connected to AD5293 RDY pin.
const uint8_t pinRESET = 10;  //!< Arduino pin connected to AD5293 !RESET pin.

const uint8_t potNomResistance = 100;   //!< Nominal resistance of the potentiometer, in kiloohms.
float potResistance = 50000.0;          //!< Potentiometer resistance.

AD5293Class Potentiometer;  //!< Potentiometer instance.
// Definitions of static variables in AD5293 library.
uint8_t   AD5293Class::chainedDevices = 0;
uint16_t* AD5293Class::allWipers = NULL;

void setup() {  
  // put your setup code here, to run once:
  while (!SerialUSB);

  // Starts communication.
  SerialUSB.begin(115200);
  SPI.begin();

  // Initialises !SYNC pin and !RESET pin.
  pinMode(pinCS, OUTPUT);
  digitalWrite(pinCS, HIGH);
  pinMode(pinRESET, OUTPUT);
  digitalWrite(pinRESET, HIGH);

  // Initialises the potentiometer instance in basic parallel-driven mode (no daisy-chain connection).
  Potentiometer.begin(pinCS, potNomResistance, CONN_PARALLEL);
  Potentiometer.setReadyPin(pinRDY);
  Potentiometer.setResetPin(pinRESET);
  Potentiometer.reset();

  // For the purpose of this example, at the stage of initialisation the wiper is set to centre position
  // (unless the above potResistance value is changed).
  Potentiometer.writeOhms(potResistance);

  SerialUSB.println("Potentiometer conigured.");
  SerialUSB.print("Current resistance: ");
  SerialUSB.print(Potentiometer.getCurrentResistance(), 1);
  SerialUSB.print(" ohms.");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while (SerialUSB.available() > 0) {
    // Look for the next valid integer in the incoming serial stream.
    uint16_t newResistance = SerialUSB.parseInt();

    // Limit the new wiper setting to the available range.
    // (Only for the example, it is done in the library with each call of relevant methods.)
    newResistance = constrain(newResistance, 0, 1023);
    
    SerialUSB.print("Wiper to be set to ");
    SerialUSB.print(newResistance, DEC);
    SerialUSB.print("... ");

    Potentiometer.write(newResistance);

    SerialUSB.println("done!");
    SerialUSB.print("New resistance: ");
    SerialUSB.print(Potentiometer.getCurrentResistance(), 1);
    SerialUSB.println(" ohms.");
  }
}
