/*
  This is an Arduino library written for the ZSSC3230 Capacitance to digital converter by Renesas.
  By Alex Dunn, Jan 23rd, 2024

  This code was developed using the ZSSC3230 Datasheet:
  https://www.renesas.com/us/en/document/dst/zssc3230-datasheet?r=465426
  However, it does not include all the functionality of the chip

  Note: The current version of this library uses delays to wait for the ZSSC3230 to complete its math 
  functions. This delay time depends upon the ADC resolution and the internal noise smoothing mode. 
  In future updates to this library, this issue will be addressed as it halts all other operations too.

*/



#ifndef ZSSC3230_I2C_Driver_h
#define ZSSC3230_I2C_Driver_h


#include "Arduino.h"
#include <Wire.h>


// i2c write definitions for the ZSSC3230
typedef enum {
	ZSSC3230_CALC_NVM_CHECKSUM		= 0x90,
	ZSSC3230_RAW_CAP_MEASUREMENT	= 0xA2,
	ZSSC3230_RAW_TEMP_MEASUREMENT	= 0xA6,
	ZSSC3230_START_SLEEP			= 0xA8,
	ZSSC3230_START_CMD				= 0xA9,
	ZSSC3230_SSC_MEASUREMENT		= 0xAA,
	ZSSC3230_START_CYC				= 0xAB,
	ZSSC3230_OVERSAMPLE_2			= 0xAC,
	ZSSC3230_OVERSAMPLE_4			= 0xAD,
	ZSSC3230_OVERSAMPLE_8			= 0xAE,
	ZSSC3230_OVERSAMPLE_16			= 0xAF,
	ZSSC3230_BROKEN_CHIP			= 0xB0,
	ZSSC3230_STOP_PDM				= 0xB4,
	ZSSC3230_SOFT_RESET				= 0xFF,

} ZSSC3230_I2C_COMMANDS;


// capacitance sensor type
typedef enum {
	TYPE_DIFFERENTIAL = 0,
	TYPE_SINGLE_END_SENSOR,
} SENSCAP_TYPE;


// sensor leakage mode on or off
typedef enum {
	LEAKAGE_CANCELLATION_OFF = 0,
	LEAKAGE_CANCELLATION_ON,
} SENSOR_LEAKAGE;


// Capacitance range options, the range is +/- the value indicated
typedef enum {
	RANGE_0_pF_5 = 0,
	RANGE_1_pF_0,
	RANGE_1_pF_5,
	RANGE_2_pF_0,
	RANGE_2_pF_5,
	RANGE_3_pF_0,
	RANGE_3_pF_5,
	RANGE_4_pF_0,
	RANGE_4_pF_5,
	RANGE_5_pF_0,
	RANGE_5_pF_5,
	RANGE_6_pF_0,
	RANGE_6_pF_5,
	RANGE_7_pF_0,
	RANGE_7_pF_5,
	RANGE_8_pF_0,
	RANGE_8_pF_5,
	RANGE_9_pF_0,
	RANGE_9_pF_5,
	RANGE_10_pF_0,
	RANGE_10_pF_5,
	RANGE_11_pF_0,
	RANGE_11_pF_5,
	RANGE_12_pF_0,
	RANGE_12_pF_5,
	RANGE_13_pF_0,
	RANGE_13_pF_5,
	RANGE_14_pF_0,
	RANGE_14_pF_5,
	RANGE_15_pF_0,
	RANGE_15_pF_5,
	RANGE_16_pF_0,
} CAP_RANGE;


// noise mode on or off
typedef enum {
	NOISE_MODE_OFF = 0,
	NOISE_MODE_ON,
} NOISE_MODE;


// ADC resolution options
typedef enum {
	ADC_12_BIT = 0,
	ADC_14_BIT,
	ADC_16_BIT,
	ADC_18_BIT,
} ADC_RES;


// Capacitance Zero Shift offset
typedef enum {
	OFFSET_0_pF_0 = 0,
	OFFSET_0_pF_25,
	OFFSET_0_pF_50,
	OFFSET_0_pF_75,
	OFFSET_1_pF_0,
	OFFSET_1_pF_25,
	OFFSET_1_pF_50,
	OFFSET_1_pF_75,
	OFFSET_2_pF_0,
	OFFSET_2_pF_25,
	OFFSET_2_pF_50,
	OFFSET_2_pF_75,
	OFFSET_3_pF_0,
	OFFSET_3_pF_25,
	OFFSET_3_pF_50,
	OFFSET_3_pF_75,
	OFFSET_4_pF_0,
	OFFSET_4_pF_25,
	OFFSET_4_pF_50,
	OFFSET_4_pF_75,
	OFFSET_5_pF_0,
	OFFSET_5_pF_25,
	OFFSET_5_pF_50,
	OFFSET_5_pF_75,
	OFFSET_6_pF_0,
	OFFSET_6_pF_25,
	OFFSET_6_pF_50,
	OFFSET_6_pF_75,
	OFFSET_7_pF_0,
	OFFSET_7_pF_25,
	OFFSET_7_pF_50,
	OFFSET_7_pF_75,
	OFFSET_8_pF_0,
	OFFSET_8_pF_25,
	OFFSET_8_pF_50,
	OFFSET_8_pF_75,
	OFFSET_9_pF_0,
	OFFSET_9_pF_25,
	OFFSET_9_pF_50,
	OFFSET_9_pF_75,
	OFFSET_10_pF_0,
	OFFSET_10_pF_25,
	OFFSET_10_pF_50,
	OFFSET_10_pF_75,
	OFFSET_11_pF_0,
	OFFSET_11_pF_25,
	OFFSET_11_pF_50,
	OFFSET_11_pF_75,
	OFFSET_12_pF_0,
	OFFSET_12_pF_25,
	OFFSET_12_pF_50,
	OFFSET_12_pF_75,
	OFFSET_13_pF_0,
	OFFSET_13_pF_25,
	OFFSET_13_pF_50,
	OFFSET_13_pF_75,
	OFFSET_14_pF_0,
	OFFSET_14_pF_25,
	OFFSET_14_pF_50,
	OFFSET_14_pF_75,
	OFFSET_15_pF_0,
	OFFSET_15_pF_25,
	OFFSET_15_pF_50,
	OFFSET_15_pF_75,
} CAP_OFFSET;



typedef enum {
	ADC_12_BIT_NM_OFF = 1770,
	ADC_14_BIT_NM_OFF = 2310,
	ADC_16_BIT_NM_OFF = 3390,
	ADC_18_BIT_NM_OFF = 5520,
	ADC_12_BIT_NM_ON = 2550,
	ADC_14_BIT_NM_ON = 3850,
	ADC_16_BIT_NM_ON = 6440,
	ADC_18_BIT_NM_ON = 11630,
} SAMPLE_RATE_DELAY;


class ZSSC3230
{
public:
	// Methods
	bool begin(uint8_t deviceAddress = (uint8_t)0x48, TwoWire &wirePort = Wire);
	void soft_reset(void);
	float read_ssc_cap(void);
	float read_ssc_cap_cyc(void);
	float read_raw_temp(void);
	int32_t read_raw_cap(void);
	bool read_nvm_reg(uint8_t *buffer, uint8_t regAdd);
	bool read_zssc3230(uint8_t *buffer, uint8_t len);
	uint8_t write_zssc3230(uint8_t regAdd, uint16_t data);
	uint8_t sleep_mode(void);
	uint8_t cyclic_mode(void);
	uint8_t command_mode(void);
	bool calibrate_zssc3230(int32_t Offset_S, int32_t Gain_S, int32_t SOT_S);
	bool configure_sensor(SENSCAP_TYPE sct, SENSOR_LEAKAGE slc, CAP_RANGE cap_range, NOISE_MODE noise_mode, ADC_RES adc_res, CAP_OFFSET cap_shift);
	bool set_i2c_address(uint8_t i2cAddress);
	void enableDebugging(void);
	void disableDebugging(void);

private:
	// Variables
	TwoWire *_i2cPort;							// stores the requested i2c port
	uint8_t _deviceAddress = 0x48;				// stores the unshifted 7-bit i2c address. Default is 0x48 
	SAMPLE_RATE_DELAY _sampleDelay;				// delay in microseconds to allow the zssc3230 to proccess the ssc mathematics, this halts the microcontroller
	float _capRange;				// capaitance range stored to use for the read_ssc_cap and read_ssc_cap_cyc functions. is updated in the configure_sensor function
	float _capOffset;				// capaitance offset stored to use for the read_ssc_cap and read_ssc_cap_cyc functions. is updated in the configure_sensor function
	uint8_t _adcResolution;			// the resolution of the adc
	bool _debug_simple = false;		// simle user activated debugging
	bool _debug_advanced = false;	// advanced debugging - Shows i2c data transfer and binary register values

	// Methods
	bool _read_zssc3230_config(void);
	bool _is_connected(void);
	float _mapf(float x, float in_min, float in_max, float out_min, float out_max);
	int32_t _sign_extend_24_32(uint32_t x);
};

#endif