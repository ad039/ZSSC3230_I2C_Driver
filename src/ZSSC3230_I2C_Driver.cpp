#include "ZSSC3230_I2C_Driver.h"

/*
@brief initialise the sensor

@param deviceAddress		i2c address of the device, non shifted, default 0x48
@param wirePort				pointer to the TwoWire object used to run i2c, default "Wire"

@return true if connection has been made and , false if sensor is not connected
*/
bool ZSSC3230::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
	//Get user's options
	_i2cPort = &wirePort;
	_deviceAddress = deviceAddress;

	// if no device is connected, return false
	if (_is_connected() == false)
	{
		if (_debug_simple == true) {
			Serial.println(F("No device connected"));
		}
		return false;
	}

	soft_reset();   //Issue command to do a software reset
	delay(50);		//Wait for device to come back online.

	if (sleep_mode() != 0)	// enter sleep mode as a defualt
	{
		if (_debug_simple == true) {
			Serial.println(F("Startup sleep mode entry failed"));
		}
		return false;
	}

	delay(10); // give the sensor time to write to memory

	// set the sample time for the sensor
	if (_read_zssc3230_config() == false) {
		if (_debug_simple == true) {
			Serial.println(F("Configuration read failed"));
		}
		return false;
	}

	delay(10);	// give the sensor time to write to memory

	return (true); //All done!
}

/*
@brief check to see if the sensor can communicate through i2c

@return		true if sensor ACKs, false if sensor did not ACK
*/
bool ZSSC3230::_is_connected(void)
{
	_i2cPort->beginTransmission(_deviceAddress);
	if (_i2cPort->endTransmission() != 0)
		return (false); //Sensor did not ACK
	return (true);    //All good
}

/*
@brief set the mode of the zssc3230 to sleep

@return 0 if I2C transfer successful
*/
uint8_t ZSSC3230::sleep_mode(void) {
	_i2cPort->beginTransmission(_deviceAddress);  // Transmit to device with address _deviceAddress
	_i2cPort->write(ZSSC3230_START_SLEEP);          // Sends value byte
	return _i2cPort->endTransmission();			// Stop transmitting and return 0 if set to sleep
}

/*
@brief set the mode of the zssc3230 to cyclic

@return 0 if I2C transfer successful
*/
uint8_t ZSSC3230::cyclic_mode(void) {
	_i2cPort->beginTransmission(_deviceAddress);  // Transmit to device with address _deviceAddress
	_i2cPort->write(ZSSC3230_START_CYC);             // Sends value byte
	return _i2cPort->endTransmission();			// Stop transmitting and return 0 if set to cyclic
}

/*
@brief set the mode of the ZSSC32320 to command

@return 0 if I2C transfer successful
*/
uint8_t ZSSC3230::command_mode(void) {
	soft_reset();
	delay(50);		// wait for device to come back online
	_i2cPort->beginTransmission(_deviceAddress);  // Transmit to device with address _deviceAddress
	_i2cPort->write(ZSSC3230_START_CMD);             // Sends value byte
	return _i2cPort->endTransmission();			// Stop transmitting and return 0 if set to command
}

/*
@brief	soft reset the ZSSC3230 
*/
void ZSSC3230::soft_reset(void) 
{
	write_zssc3230(0xFF, 0x0000);
}

/*
@brief read the ssc capacitance
Note: this delays the microcontroller

@return	ssc capacitance as a floating point number 
*/
float ZSSC3230::read_ssc_cap(void)
{
	write_zssc3230(ZSSC3230_SSC_MEASUREMENT, 0);	// ask the sensor for a ssc measurment

	uint8_t buffer[4]; // create a buffer to write to

	delayMicroseconds(_sampleDelay);	// delay to allow the sensor to complete its mathematics

	read_zssc3230(buffer, 4);		// read the ssc measurment

	uint32_t ssc_cap = ((uint32_t)buffer[1] << 16 )| ((uint32_t)buffer[2] << 8) | ((uint32_t)buffer[3]);

	return (ssc_cap * 2 * _capRange / 16777216.0f) + (_capOffset - _capRange);
}

/*
@brief read the ssc capacitance when the sensor is in cyclic mode

@return ssc capacitance as a floating point number
*/
float ZSSC3230::read_ssc_cap_cyc(void)
{
	uint8_t buffer[4]; // create a buffer to write to

	read_zssc3230(buffer, 4);		// read the ssc measurment

	uint32_t ssc_cap = ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | ((uint32_t)buffer[3]);	// convert the i2c buffer to a 32 bit unsigned int
	return (ssc_cap * 2 * _capRange / 16777216.0f) + (_capOffset - _capRange);	// return the ssc capacitance
}

/*
@brief read the temperature through the ADC
Note: this delays the microcontroller

@return temperature as a floating point number
*/
float ZSSC3230::read_raw_temp(void)
{
	write_zssc3230(ZSSC3230_RAW_TEMP_MEASUREMENT, 0);
	
	uint8_t buffer[4];

	delayMicroseconds(_sampleDelay);	// delay to allow the sensor to complete its mathematics

	read_zssc3230(buffer, 4);		// read the raw temp measurment

	int temp_raw = (((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | ((uint32_t)buffer[3])) & 0x3FFF;

	return _mapf(temp_raw, 0, (0x3FFF - 1), -40, 125);
}

/*
@brief read the raw ADC capacitance value
Note: this delays the microcontroller

@return raw ADC capacitance as a signed 32 bit integer
*/
int32_t ZSSC3230::read_raw_cap(void)
{
	write_zssc3230(ZSSC3230_RAW_CAP_MEASUREMENT, 0);

	uint8_t buffer[4]; // create a buffer to write to

	delayMicroseconds(_sampleDelay);	// delay to allow the sensor to complete its mathematics

	read_zssc3230(buffer, 4);		// read the raw cap measurment

	uint32_t cap_raw = (((uint32_t)buffer[1] << 16) & 0x00FF0000) | (((uint32_t)buffer[2] << 8) & 0x0000FF00) | (((uint32_t)buffer[3]) & 0x000000F);
	return _sign_extend_24_32(cap_raw);
}

/*
@brief function to sign extend 24 bit to 32 bit

@return 32 bit signed integer
*/
int32_t ZSSC3230::_sign_extend_24_32(uint32_t x) 
{
	const int bits = 24;
	uint32_t m = (uint32_t)1u << (bits - 1);
	return (x ^ m) - m;
}

/*
@brief read an nvm register from the ZSSC3230

@param buffer		a pointer to a buffer
@param regAdd		the memory register you want to read
*/
bool ZSSC3230::read_nvm_reg(uint8_t *buffer, uint8_t regAdd) {

	if (regAdd < 0x00 || regAdd > 0x18) {
		// it is outside the nvm register bounds
		return false;
	}

	write_zssc3230(regAdd, 0);

	delay(1);	// delay to allow the sensor to access its memory

	read_zssc3230(buffer, 3);		// read the raw cap measurment and place onto the buffer

	return true;
}


/*
@brief read the zssc3230 through i2c.
Can make a measurement reading, Raw ADC reading or read from NVM memory

@param *buffer	a pointer to a buffer which stores the i2c bytes
@param len		the number of bytes recieved from the sensor

@return true if the number of bytes in the i2c transfer is equal to the desired length
*/
bool ZSSC3230::read_zssc3230(uint8_t *buffer, uint8_t len)
{	
	// request information from the sensor
	_i2cPort->requestFrom(_deviceAddress, len);  // Request 4 bytes from slave device @ address _deviceAddress

	uint8_t i = 0;
	while (_i2cPort->available()) {
		buffer[i++] = _i2cPort->read();  // Receive a byte & push it into the buf
	}

	if (_debug_advanced == true) {
		Serial.print(F("I2C Data Transfer: "));
		uint8_t x = 0;
		while (x < len - 1) {
			Serial.print(buffer[x++], BIN);
			Serial.print(' ');
		}
		Serial.println(buffer[x], BIN);
	}

	if (i == len)
		return true;
	return false; //Error
}

/*
@brief write to zssc3230 over i2c
Can write to a NVM register or ask the zssc3230 for a capacitance or temperature reading

@param regADD	the register address to send over i2c. This is a command to access NVM memory or ask for a capacitance sample
@param data		if writing to NVW memory, this is where the data is placed. Default (0x0000)

@return			a status number indicating if the i2c transfer was succesfful (0) or not (1,2,3,4,5)
*/
uint8_t ZSSC3230::write_zssc3230(uint8_t regAdd, uint16_t data = (uint16_t)0x0000)
{
	// write to sensor requesting status data

	_i2cPort->beginTransmission(_deviceAddress);  // Transmit to device with address _deviceAddress
	_i2cPort->write(regAdd);             // Load value byte
	_i2cPort->write((data & 0xFF00) >> 8);
	_i2cPort->write(data & 0x00FF);
	return _i2cPort->endTransmission();      // Send value byte and stop transmitting
}


/*
@brief function to map a number from one range to another range

@param x			input number
@param in_min		the minimum of the current range
@param in_max		the maximum of the current range
@param out_min		the minimum of the desired range
@param out_max		the maxinum of the desired range

@return	the range shifted value of x as a floating point number
*/
float ZSSC3230::_mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
@brief configure the sensor_config register in the NVM this handles the capacitance range,
capacitance zero offset, noise mode, adc bits, sensor leakage cancellation and sensecap type
we also set the sample delay time in this function which allows the mathematics to take place 
between a sample request from the master and a sample sent from the slave

@param sct				sensecap type (0 = Defferential mode, 1 = single end sensor mode)
@param slc				sensor leakage cancellation (0 = off, 1 = on)
@param cap_range		range of the capacitance (0.5 to 16pF)
@param noise_mode		noise mode (0 = off, 1 = on)
@param adc_res			analogue to digital comverter resolution
@param cap_offset		Capacitance zero shift offset

@return true if i2c write completed successfully
*/
bool ZSSC3230::configure_sensor(SENSCAP_TYPE sct, SENSOR_LEAKAGE slc, CAP_RANGE cap_range, NOISE_MODE noise_mode, ADC_RES adc_res, CAP_OFFSET cap_offset) {
	
	uint8_t status1 = 0;

	// build the configure register out of the function inputs by shifting bits to there requires place
	uint16_t config = ((sct & 0x01) << 15) | ((slc & 0x01) << 14) | (cap_range & 0x1F) << 9 | (noise_mode & 0x01) << 8 | (adc_res & 0x03) << 6 | (cap_offset & 0x3F);
	
	// write the config to NVM memory location 0x12
	status1 = write_zssc3230(0x32, config);

	delay(10);

	// set the sample delay dependant on the adc resolution and noise mode, the cap offset and the cap range in the library
	_read_zssc3230_config();

	delay(10);
	if (status1 == 0)
		return true;	// i2c write completed successfully

	return false;	// ERROR
}

/*
@brief read the config register to update private library variables _sampleDelay, _capRange and _capOffset
*/
bool ZSSC3230::_read_zssc3230_config(void) {
	// define a buffer to store register 0x12 on
	uint8_t buffer[3];

	// variable to store reuqired parts of the zssc3230 of the zssc3230
	uint8_t noise_mode;
	uint8_t cap_range;
	uint8_t cap_offset;
	uint8_t adc_res;


	if (read_nvm_reg(buffer, 0x12) == false)
	{
		return false;
	}
	else
	{
		noise_mode = buffer[1] & 0x01;
		cap_range = (buffer[1] & 0x3E) >> 1;
		cap_offset = buffer[2] & 0x3F;
		adc_res = (buffer[2] & 0xC0) >> 6;

		if (_debug_advanced == true) {
			Serial.print(F("Noise Mode = "));
			Serial.println(noise_mode, BIN);
			Serial.print(F("Cap Range = "));
			Serial.println(cap_range, BIN);
			Serial.print(F("Cap Offset = "));
			Serial.println(cap_offset, BIN);
			Serial.print(F("ADC Resolution = "));
			Serial.println(adc_res, BIN);
		}
	}

	
	if (noise_mode == NOISE_MODE_OFF) {
		switch (adc_res) {
		case ADC_12_BIT:
			_sampleDelay = ADC_12_BIT_NM_OFF;
			_adcResolution = 12;
			break;
		case ADC_14_BIT:
			_sampleDelay = ADC_14_BIT_NM_OFF;
			_adcResolution = 14;
			break;
		case ADC_16_BIT:
			_sampleDelay = ADC_16_BIT_NM_OFF;
			_adcResolution = 16;
			break;
		case ADC_18_BIT:
			_sampleDelay = ADC_18_BIT_NM_OFF;
			_adcResolution = 18;
			break;
		}
	}
	if (noise_mode == NOISE_MODE_ON) {
		switch (adc_res) {
		case ADC_12_BIT:
			_sampleDelay = ADC_12_BIT_NM_ON;
			_adcResolution = 12;
			break;
		case ADC_14_BIT:
			_sampleDelay = ADC_14_BIT_NM_ON;
			_adcResolution = 14;
			break;
		case ADC_16_BIT:
			_sampleDelay = ADC_16_BIT_NM_ON;
			_adcResolution = 16;
			break;
		case ADC_18_BIT:
			_sampleDelay = ADC_18_BIT_NM_ON;
			_adcResolution = 18;
			break;
		}
	}

	_capRange = (cap_range + 1) / 2.0f;	// convert the input cap_range into a value that can be used for read_ssc_cap and read_ssc_cap_cyc

	// convert the cap_offset into a value that can be used for read_ssc_cap and read_ssc_cap_cyc to give the proper capacitance range and offset
	if (cap_offset == OFFSET_0_pF_0)
		_capOffset = 0;
	else
		_capOffset = cap_offset / 4.0f;

	if (_debug_simple == true) {
		Serial.println(F("Configuration Successful: "));
		Serial.print(F("Noise mode "));
		(noise_mode == 0) ? Serial.println(F("off")) : Serial.println(F("on"));
		Serial.print(F("Sample delay set to "));
		Serial.print(_sampleDelay);
		Serial.println(F("us"));
		Serial.print(F("ADC Resolution set to "));
		Serial.print(_adcResolution);
		Serial.println(F("-bit"));
		Serial.print(F("Capacitance range set to +/-"));
		Serial.print(_capRange);
		Serial.println(F("pF"));
		Serial.print(F("Capacitance offset set to "));
		Serial.print(_capOffset);
		Serial.println(F("pF"));
		delay(1);
	}
	
	return true;
}


/*
@brief function to calibrate the sensor using the coefficients calculated by the ZSSC323x evaluation software available at: https://www.renesas.com/us/en/products/sensor-products/sensor-signal-conditioners-ssc-afe/zssc3230-low-power-high-resolution-capacitive-sensor-signal-conditioner#design_development

@param Offset_S		capacitance sensor offset coefficient
@param Gain_S		capacitance sensor gain coefficient 
@param SOT_S		Second order term for the sensor

@return true if all i2c writes have been successfull
*/
bool ZSSC3230::calibrate_zssc3230(int32_t Offset_S, int32_t Gain_S, int32_t SOT_S) 
{
	uint8_t status1 = 0;

	// we need to write these in sign-magnitude binary format to the zssc3230
	// extract the sign of the number
	int sign_offset_s = (Offset_S < 0) ? 1 : 0;     // this is the same as writing: if (offset_S<0) { sign_offset_s = 1;}\n else {sign_offset_s = 0;}\n
	int sign_gain_s = (Gain_S < 0) ? 1 : 0;
	int sign_SOT_s = (SOT_S < 0) ? 1 : 0;

	// extract the magnitude of the number
	uint32_t Offset_S_Mag = abs(Offset_S);
	uint32_t Gain_S_Mag = abs(Gain_S);
	uint32_t SOT_S_Mag = abs(SOT_S);

	if (_debug_advanced == true) {
		Serial.print(F("Debug Offset S in Sign Mag: "));
		Serial.print(sign_offset_s, BIN);
		Serial.print(F(" - "));
		Serial.println(Offset_S_Mag, BIN);
		Serial.print(F("Debug Gain S in Sign Mag: "));
		Serial.print(sign_offset_s, BIN);
		Serial.print(F(" - "));
		Serial.println(Gain_S_Mag, BIN);
		Serial.print(F("Debug SOT S in Sign Mag: "));
		Serial.print(sign_offset_s, BIN);
		Serial.print(F(" - "));
		Serial.println(SOT_S_Mag, BIN);
	}

	// write bits 15:0 of Offset_S to register 0x03
	status1 += write_zssc3230(0x23, (Offset_S_Mag & 0x00FFFF));

	delay(10);

	// write bits 15:0 of Gain_S to register 0x04
	status1 += write_zssc3230(0x24, (Gain_S_Mag & 0x00FFFF));

	delay(10);

	// write sign bits and MSB of Offset_S and Gain_S to register 0x0D
	status1 += write_zssc3230(0x2D, (sign_offset_s << 15 | (Offset_S_Mag & 0x7F0000) >> 8 | sign_gain_s << 7 | (Gain_S_Mag & 0x7F0000) >> 16));

	delay(10);

	// write bits 15:0 of SOT_S to register 0x09
	status1 += write_zssc3230(0x29, (SOT_S_Mag & 0x00FFFF));

	delay(10);
	
	// write sign bit and MSB of SOT_S to register 0x10
	status1 += write_zssc3230(0x30, ((sign_SOT_s << 15 | (SOT_S_Mag & 0x7F0000) >> 8)) & 0xFF00);

	delay(10);

	if (status1 == 0)
		return true;	// calibration complete

	return (false); // ERROR
}


/*
@brief set the i2c address of the zssc3230 in NVM

@param i2cAddress	the desired 7 bit i2c address

@return true if i2c write has been successful
*/
bool ZSSC3230::set_i2c_address(uint8_t i2cAddress) {
	// nvm register 0x02 contains the i2c address
	// 0x02 stores the address in bits 6:0, whilst other interface configurations are set in the rest of the register
	// to write a new address without changing any other configuration, first we need to read the register

	write_zssc3230(0x02, 0);	// write to the zssc3230 asking for the nvm 0x02 register data

	uint8_t buffer[3];

	delay(1);	// delay 1ms to allow the register to read

	read_zssc3230(buffer, 3);	// read the i2c data with the 0x02 register data

	// gather the bytes into a variable
	uint16_t interfaceConfig = ((uint16_t)buffer[1] << 8) | ((uint16_t)buffer[2]);

	// take the previous config stored in bits 15:7 of 0x02 and combine it with the new address
	interfaceConfig = (interfaceConfig & 0xFF80) | (i2cAddress & 0x7F);

	// update the register 0x02
	uint8_t status1 = write_zssc3230(0x22, interfaceConfig);

	// update the device address in ZSSC3230 class
	_deviceAddress = i2cAddress;

	if (status1 == 0)
		return true;	// Address change successfull

	return false;	// ERROR
}


void ZSSC3230::enable_debugging(void)
{
	_debug_simple = true;
}

void ZSSC3230::disable_debugging(void)
{
	_debug_simple = false;
}


