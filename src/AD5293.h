/*! \file	AD5293.h
 *	\brief	AD5293 class header
 * 
 *	\details	Handling of AD5293 digital potentiometer.
 *
 *	\pre	SPI is required to operate the AD5293 converter.
 *	\pre	Therefore, use of <b>SPI</b> library is needed.
 *
 *	\date	Created: 30.11.2016 18:43:23
 *  \author	Grzegorz Wielgoszewski \<Arduino@Wielgoszewski.pl\>
 */

#ifndef _AD5293_h
#define _AD5293_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <SPI.h>
#if defined(ARDUINO_ARCH_AVR)
  #include <AVR_status_codes.h>
#else defined(ARDUINO_ARCH_SAM)
  #include <status_codes.h>
#endif

/*!
 *	\brief	Enumeration of connection modes.
 */
enum SPIConnectionMode {
	CONN_PARALLEL,		//!< Converter operates as a single device (dedicated <tt>!CS</tt> pin).
	CONN_DAISYCHAIN		//!< Converter operates as one of daisy-chained devices (shared <tt>!CS</tt> pin).
};

#define POT_BITS				(10u)		//!< Number of bits.
#define POT_WIPER_RESISTANCE	(60.0)		//!< Wiper resistance (ohms).

/*!
 *	\brief	Class for the AD5293 digital potentiometer.
 */
class AD5293Class
{
 private:
	SPISettings* m_commSettings;	//!< SPI communication settings.
	SPIConnectionMode m_connMode;	//!< Connection mode.
	boolean m_useMISO = false;		//!< Indicates if data is sent from SDO to microcontroller MISO.
 
	uint8_t m_pinCS;			//!< !SYNC (!CS) pin of the device (Arduino pin).
	uint8_t m_pinRDY = 255;		//!< RDY pin of the device (Arduino pin).
	uint8_t m_pinRST = 255;		//!< !RESET pin of the device (Arduino pin).
	
	uint16_t m_wiper;			//!< Current wiper position.
	float m_topRefVoltage;		//!< Top reference voltage value (V).
	float m_bottomRefVoltage;	//!< Bottom reference voltage value (V).

	float m_topResistance;		//!< Largest resistance (ohms).
	float m_bottomResistance;	//!< Wiper resistance (ohms).
	
	const uint8_t m_bits = POT_BITS;	//!< Number of wiper setting bits.

	uint8_t m_chainOrder = 0;			//!< Order in chain of devices.
	AD5293Class* previousADC = NULL;	//!< Pointer to previous device in chain.
	AD5293Class* nextADC = NULL;		//!< Pointer to next device in chain.
	
 protected:

 	static uint8_t chainedDevices;	//!< Number of daisy-chained devices.
 	static uint16_t* allWipers;		//!< Array with all current wiper positions.

 public:
  
/*!
 *	\brief	Configures the potentiometer (symmetric bipolar).
 *
 *	\param	pinCS		\c !SYNC pin number (Arduino pin).
 *	\param	refVoltage	Reference voltage (V).
 *	\param	connMode	Multiple devices connection mode.
 *
 *	\note	Use this function if the reference voltage is symmetric and bipolar, e.g. -1.024 V to +1.024 V.
 *
 *	\returns	\see_status_code
 */ 
enum status_code begin(uint8_t pinCS, float refVoltage, SPIConnectionMode connMode);

/*!
 *	\brief	Configures the potentiometer (two separate voltages).
 *
 *	\param	pinCS				!SYNC pin number (Arduino pin).
 *	\param	refVoltageBottom	Lower reference voltage (V).
 *	\param	refVoltageTop		Upper reference voltage (V).
 *	\param	connMode			Multiple devices connection mode.
 *
 *	\note	Use this function if the top and bottom reference voltage is unrelated, e.g. 0 to 4.096 V.
 *
 *	\returns	\see_status_code
 */ 
enum status_code begin(uint8_t pinCS, float refVoltageBottom, float refVoltageTop, SPIConnectionMode connMode);

/*!
 *	\brief	Configures the potentiometer (rheostat).
 *
 *	\param	pinCS			\c !SYNC pin number (Arduino pin).
 *	\param	nomResistance	Nominal resistance (kiloohms).
 *	\param	connMode		Multiple devices connection mode.
 *
 *	\note	Use this function if the potentiometer acts as a variable resistor in series.
 *
 *	\returns	\see_status_code
 */ 
enum status_code beginRheo(uint8_t pinCS, uint8_t nomResistance, SPIConnectionMode connMode);

/*!
 *	\brief	Configures position in a chain of AD5293s.
 *
 *	\param	chainNo		Position in chain of converters (the device to which MOSI is connected is at the <b>last</b> position).
 *	\param	prevADC		Pointer to instance of the preceding converter in chain.
 *	\param	nextADC		Pointer to instance of the following converter in chain.
 *
 *	\note	Use <tt>NULL</tt> as appropriate pointers for the first and last converters in chain.
 *
 *	\returns	\see_status_code
 */ 
enum status_code configureChain(uint8_t chainNo, AD5293Class* prevADC, AD5293Class* nextADC);

/*!
 *	\brief	Sets hardware usage of \c SDO pin.
 *
 *	\param	isDataOutUsed	Set to \c true if SPI data is read in the microcontroller via \c MISO pin.
 */
void useMISO(boolean isDataOutUsed);

/*!
 *	\brief	Assigns Arduino pin to \c RDY function of AD5293.
 *
 *	\param	pinRDY	\c RDY pin number (Arduino pin).
 *
 *	\note	\c RDY pin is an optional functionality of AD5293 and requires a suitable board - one of Arduino digital pins needs to be connected to the \c RDY pin.
 */
void setReadyPin(uint8_t pinRDY);

/*!
 *	\brief	Assigns Arduino pin to <tt>!RESET</tt> function of AD5293.
 *
 *	\param	pinRST	<tt>!RESET</tt> pin number (Arduino pin).
 *
 *	\note	Software <tt>!RESET</tt> may be considered optional in many designs, 
 *	\note	therefore this functionality requires a suitable board - one of Arduino digital pins needs to be connected to the <tt>!RESET</tt> pin of AD5293.
 */
void setResetPin(uint8_t pinRST);

/*!
 *	\brief	Reads the state of \c RDY pin.
 *	
 *	\note		\c RDY pin is an optional functionality of AD5293. Please consider proper handling of boards with no connection to \c RDY pin. 
 *
 *	\returns	\c RDY pin state if present, \c true otherwise.
 */
boolean isReady(void);

/*!
 *	\brief	Performs hardware reset.
 */
void reset(void);

/*! 
 *	\brief	Combines command code and data.
 *
 *	\param	commandCode	Command to be executed.
 *	\param	data		Data for the command.
 *
 *	\returns	Ready-to-send half word.	
 */
uint16_t combine(uint8_t commandCode, uint16_t data);

/*!
 *	\brief	Send command to switch \c SDO pin to high impedance.
 */
void placeSDOinHighZ(void);

/*!
 *	\brief	Resets wiper to midscale.
 *
 *	\note	If in daisy-chain mode, sets all devices to midscale.
 */
enum status_code resetMidscale(void);

/*!
 *	\brief	Sends the wiper setting as raw value.
 *
 *	\param	wiperSetting	Integer value from 0 to 1023 (10-bit potentiometer).
 *
 *	\returns	\see_status_code
 */
enum status_code write(uint16_t wiperSetting);

/*!
 *	\brief	Sends the wiper setting as goal value.
 *
 *	\param	outResistance	Floating-point number representing resistance to be set (ohms).
 *
 *	\returns	\see_status_code
 */
enum status_code writeOhms(float outResistance);

/*!
 *	\brief	Sends the wiper setting as goal value.
 *
 *	\param	outVoltage	Floating-point number representing voltage to be set (V).
 *
 *	\returns	\see_status_code
 */
enum status_code writeVolts(float outVoltage);

/*!
 *	\brief	Reads wiper position from the device returns it.
 *
 *	\returns	Current wiper position (raw).
 */
uint16_t readWiper(void);

/*!
 *	\brief	Returns current wiper position.
 *
 *	\param	iterate		When daisy-chained, iterate over all devices (\c false by default).
 *
 *	\returns	Current wiper position (raw).
 */
uint16_t getWiper(boolean iterate);

/*!
 *	\brief	Returns current resistance value.
 *
 *	\returns	Current resistance value (ohms).
 */
float getCurrentResistance(void);

/*!
 *	\brief	Returns current voltage value.
 *
 *	\returns	Current voltage value (V).
 */
float getCurrentVoltage(void);

};

// Registers definitions.
#define POT_CTRL_C2		(0x2u)									//!< Calibration enable.
#define		POT_CTRL_CAL_RESPERF		(0x0u << POT_CTRL_C2)	//!< Resistor performance mode (default).
#define		POT_CTRL_CAL_NORMAL			(0x1u << POT_CTRL_C2)	//!< Normal mode.
#define POT_CTRL_C1		(0x1u)									//!< \c RDAC register write protect.
#define		POT_CTRL_WIPER_LOCK_ACTIVE	(0x0 << POT_CTRL_C1)	//!< Wiper position locked digitally.
#define		POT_CTRL_WIPER_ALLOW_WRITE	(0x1 << POT_CTRL_C1)	//!< Allows update of wiper position.

// Command definitions.
#define	POT_CMD_NOP				(0x00u)		//!< No operation.
#define POT_CMD_WRITE			(0x01u)		//!< Write to \c RDAC.
#define POT_CMD_READ			(0x02u)		//!< Read from \c RDAC.
#define POT_CMD_RESET			(0x04u)		//!< Software reset; refresh with midscale code.
#define	POT_CMD_WR_CTRL			(0x06u)		//!< Write to control register.
#define POT_CMD_RD_CTRL			(0x07u)		//!< Read control register.
#define POT_CMD_SW_POWER_DOWN	(0x08u)		//!< Software power-down.
#define POT_CMD_SDO_HIGHZ		(0x8001u)	//!< \c SDO into high-Z state.

// Other definitions
#define POT_CMD_Pos				(0xAu)		//!< Command position in the serial data.
#define POT_CMD_Msk				(0x3C00u)	//!< Command mask.
#define POT_DATA_Msk			(0x03FFu)	//!< Wiper setting mask.

#endif

