/*! \file	AD5293.cpp
 *	\brief	AD5293 class code
 * 
 *	\details	Handling of AD5293 digital potentiometer.
 *
 *	\pre	SPI is required to operate the AD5293 converter.
 *	\pre	Therefore, use of <b>SPI</b> library is needed.
 *
 *	\date	Created: 30.11.2016 18:43:23
 *  \author	Grzegorz Wielgoszewski \<Arduino@Wielgoszewski.pl\>
 */

#include "AD5293.h"

/*!
 *	\details	Calls the #begin() function for bipolar reference voltage - repeats \a refVoltage as negative and positive value.
 */
enum status_code AD5293Class::begin(uint8_t pinCS, float refVoltage, SPIConnectionMode connMode = CONN_PARALLEL) {
	return this->begin(pinCS, -refVoltage, refVoltage, connMode);
};

/*!
 *	\details	Assigns Arduino pin for SPI operation of the AD5293 and sets the voltages for potentiometer input (used in output calculations).
 *				SPI parameters (#m_commSettings) are also set within this function.
 */
enum status_code AD5293Class::begin(uint8_t pinCS, float refVoltageBottom, float refVoltageTop, SPIConnectionMode connMode = CONN_PARALLEL) {
	this->m_bits = 10;
	
	this->m_pinCS = pinCS;
	this->m_bottomRefVoltage = refVoltageBottom;
	this->m_topRefVoltage = refVoltageTop;
	this->m_wiper = 0;
	
	pinMode(this->m_pinCS, OUTPUT);
	digitalWrite(this->m_pinCS, HIGH);
	
	this->m_commSettings = new SPISettings(500000, MSBFIRST, SPI_MODE1);
	this->m_connMode = connMode;

	this->chainedDevices += 1;

	this->placeSDOinHighZ();

	return STATUS_OK;
};

/*!
 *	\details	
 */
enum status_code AD5293Class::configureChain(uint8_t chainNo, AD5293Class* prevADC, AD5293Class* nextADC) {
	this->m_chainOrder = chainNo;
	this->previousADC = prevADC;
	this->nextADC = nextADC;

	// If not yet created, creates array with all current wiper positions.
	if (this->allWipers == NULL) 
		this->allWipers = new uint16_t(this->chainedDevices);

	return STATUS_OK;
};

/*!
 *	\details	Assigns Arduino pin for SPI operation of the AD5293 and sets the resistances for potentiometer operation (used in value calculations).
 *				SPI parameters (#m_commSettings) are also set within this function.
 */
enum status_code AD5293Class::beginRheo(uint8_t pinCS, uint8_t nomResistance, SPIConnectionMode connMode = CONN_PARALLEL) {
	this->m_bits = 10;

	this->m_pinCS = pinCS;
	this->m_bottomResistance = POT_WIPER_RESISTANCE;

	switch (nomResistance) {
		case 20:
			this->m_topResistance = 20000.0;
			break;

		case 50:
			this->m_topResistance = 50000.0;
			break;

		case 100:
			this->m_topResistance = 100000.0;
			break;

		default:
			this->m_topResistance = 0.0;
	}
	this->m_topResistance += POT_WIPER_RESISTANCE;

	this->m_bottomRefVoltage = 0.0;
	this->m_topRefVoltage = 0.0;

	this->m_wiper = 0;
	
	pinMode(this->m_pinCS, OUTPUT);
	digitalWrite(this->m_pinCS, HIGH);
	
	this->m_commSettings = new SPISettings(500000, MSBFIRST, SPI_MODE1);
	this->m_connMode = connMode;

	this->chainedDevices += 1;

	this->placeSDOinHighZ();

	return STATUS_OK;
};


/*!
 *	\details	Assigns Arduino pin for identification of read/write completion.
 */
void AD5293Class::setReadyPin(uint8_t pinRDY) {
	this->m_pinRDY = pinRDY;
	
	// By default, the device is ready -> pull up.
	pinMode(this->m_pinRDY, INPUT_PULLUP);
}

/*!
 *	\details	Assigns Arduino pin for hardware reset.
 */
void AD5293Class::setResetPin(uint8_t pinRST) {
	this->m_pinRST = pinRST;
	
	pinMode(this->m_pinRST, OUTPUT);
	digitalWrite(this->m_pinRST, HIGH);
}

/*!
 *	\details	If \c RDY pin is not assigned, \c true is returned after a delay of 10 ms.
 */
boolean AD5293Class::isReady() {
	if (this->m_pinRDY != 255) {
		return ((digitalRead(m_pinRDY) == 1) ? true : false);
	} else {
		delay(10);
		return true;
	}
}

/*!
 *	\details	After reset, \c SDO pin is switched to high-impedance.
 */
void AD5293Class::reset() {
	if (this->m_pinRST != 255) {
		digitalWrite(this->m_pinRST, LOW);
		while (!(digitalRead(this->m_pinRST) == LOW));
		digitalWrite(this->m_pinRST, HIGH);
	}

	this->resetMidscale();

	this->placeSDOinHighZ();
}

/*!
 *	
 */
uint16_t AD5293Class::combine(uint8_t command, uint16_t data) {
	return (((uint16_t)command << POT_CMD_Pos) | (data & POT_DATA_Msk));
}

/*!
 *	\details	See Table 10 in AD5293 datasheet.
 */
void AD5293Class::placeSDOinHighZ() {
	SPI.beginTransaction(*this->m_commSettings);
	digitalWrite(this->m_pinCS, LOW);
	SPI.transfer16(POT_CMD_SDO_HIGHZ);
	digitalWrite(this->m_pinCS, HIGH);
	while (this->isReady() == false);
	digitalWrite(this->m_pinCS, LOW);
	SPI.transfer16(this->combine(POT_CMD_NOP, 0));
	digitalWrite(this->m_pinCS, HIGH);
	SPI.endTransaction();
}

/*!
 *	
 */
enum status_code AD5293Class::resetMidscale() {
	uint8_t count = 0;
	uint16_t receivedData = 0;

	switch (this->m_connMode) {
		case CONN_PARALLEL:
			this->write((uint16_t)1 << (this->m_bits - 1));
			break;

		case CONN_DAISYCHAIN:
			SPI.beginTransaction(*this->m_commSettings);

			digitalWrite(this->m_pinCS, LOW);
			for (count = 0; count < this->chainedDevices; count++) {
				receivedData = SPI.transfer16(this->combine(POT_CMD_WR_CTRL, POT_CTRL_WIPER_ALLOW_WRITE));
			}
			digitalWrite(this->m_pinCS, HIGH);

			while (this->isReady() == false);
			
			digitalWrite(this->m_pinCS, LOW);
			for (count = 0; count < this->chainedDevices; count++) {
				receivedData = SPI.transfer16(this->combine(POT_CMD_WRITE, ((uint16_t)1 << (this->m_bits - 1))));
			}
			digitalWrite(this->m_pinCS, HIGH);
			SPI.endTransaction();

			while (this->isReady() == false);

			break;

		default:
			;
	}

	return STATUS_OK;
};

/*!
 *	\details	See Table 9 in AD5293 datasheet.
 */
enum status_code AD5293Class::write(uint16_t wiperSetting) {

	uint16_t receivedData = 0;

	this->placeSDOinHighZ();

	SPI.beginTransaction(*this->m_commSettings);

	digitalWrite(this->m_pinCS, LOW);
	
	receivedData = SPI.transfer16(this->combine(POT_CMD_WR_CTRL, POT_CTRL_WIPER_ALLOW_WRITE));
	digitalWrite(this->m_pinCS, HIGH);

	while (this->isReady() == false);
	
	digitalWrite(this->m_pinCS, LOW);
	receivedData = SPI.transfer16(this->combine(POT_CMD_WRITE, wiperSetting));

	digitalWrite(this->m_pinCS, HIGH);
	SPI.endTransaction();

	while (this->isReady() == false);

	this->m_wiper = this->getWiper() & POT_DATA_Msk;
	
	return STATUS_OK;
};

/*
 *	\details	The \a outResistance is trimmed if out of reference range <tt>[#m_bottomResistance, #m_topResistance]</tt>.
 */
enum status_code AD5293Class::writeOhms(float outResistance) {
	float outVal = constrain(outResistance, this->m_bottomResistance, this->m_topResistance);
	uint16_t outWiper;
	
	outWiper = (uint16_t)floorf(((1 << this->m_bits)*(outResistance - this->m_bottomResistance))/(this->m_topResistance - this->m_bottomResistance));
	outWiper = constrain(outWiper, 0, (1 << this->m_bits) - 1);
	
	return this->write(outWiper);
};

/*
 *	\details	The \a outVoltage is trimmed if out of reference range <tt>[#m_bottomRefVoltage, #m_topRefVoltage]</tt>.
 */
enum status_code AD5293Class::writeVolts(float outVoltage) {
	float outVal = constrain(outVoltage, this->m_bottomRefVoltage, this->m_topRefVoltage);
	uint16_t outWiper;
	
	outWiper = (uint16_t)floorf(((1 << this->m_bits)*(outVoltage - this->m_bottomRefVoltage))/(this->m_topRefVoltage - this->m_bottomRefVoltage));
	outWiper = constrain(outWiper, 0, (1 << this->m_bits) - 1);
	
	return this->write(outWiper);
};

/*!
 *	\details	Within the function, #m_wiper is updated based on the actual read-out from the device. 
 */
uint16_t AD5293Class::readWiper() {
	uint16_t receivedData = 0;
	
	SPI.beginTransaction(*this->m_commSettings);
	
	digitalWrite(this->m_pinCS, LOW);
	receivedData = SPI.transfer16(this->combine(POT_CMD_READ, 0x155));
	digitalWrite(this->m_pinCS, HIGH);

	//  SPI.endTransaction();
	while (this->isReady() == false);
	// SPI.beginTransaction(*this->m_commSettings);
	digitalWrite(this->m_pinCS, LOW);
	receivedData = SPI.transfer16(this->combine(POT_CMD_NOP, 0x155));
	digitalWrite(this->m_pinCS, HIGH);

	SPI.endTransaction();

	this->m_wiper = receivedData & POT_DATA_Msk;

	this->placeSDOinHighZ();
	
	return receivedData;
};

/*!
 *	\details	Within the function, #m_wiper is updated based on a previous actual read-out for all daisy-chained devices. 
 */
uint16_t AD5293Class::getWiper() {
	uint16_t rawWiper = 0;

	switch (this->m_connMode) {
		case CONN_DAISYCHAIN:
			this->m_wiper = this->allWipers[this->m_chainOrder];

		case CONN_PARALLEL:
			rawWiper = this->m_wiper;
			break;

		default:
			;
	}
	
	return rawWiper;
};

/*!
 *	\details	Read-out from #getWiper() function is recalculated according to #m_topResistance and #m_bottomResistance values.
 */
float AD5293Class::getCurrentResistance() {
	float wiper = float(this->getWiper());
	return (this->m_bottomResistance + (wiper * (this->m_topResistance- this->m_bottomResistance))/(1 << this->m_bits));
};

/*!
 *	\details	Read-out from #getWiper() function is recalculated according to #m_topRefVoltage and #m_bottomRefVoltage values.
 */
float AD5293Class::getCurrentVoltage() {
	float wiper = float(this->getWiper());
	return (this->m_bottomRefVoltage + (wiper * (this->m_topRefVoltage - this->m_bottomRefVoltage))/(1 << this->m_bits));
};

