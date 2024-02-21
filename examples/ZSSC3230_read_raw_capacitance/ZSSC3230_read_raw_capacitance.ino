/*
Reading the ZSSC3230 capacitance to digital converter from Renesas
By: Alex Dunn @ UOW
Date: January 22, 2024

This example shows how to read the raw analog to digital converted (ADC) capacitance value from the ZSSC3230
It also shows how to adjust the range, zero shift offset and ADC resolution
*/

#include <ZSSC3230_I2C_Driver.h>
#include <Wire.h>

// create a ZSSC3230 object
ZSSC3230 zssc3230;

void setup() {
  // begin serial communication
  Serial.begin(115200);
  while (!Serial)
     delay(10);

  // begin wire communication
  Wire.begin();

  // enable debugging
  //zssc3230.enable_debugging();

  // initialise the zssc3230 sensor
  while(1) {
	  if (zssc3230.begin()) {
		Serial.println("ZSSC3230 Detected");
		break;
	  }
	  else {
		Serial.println("ZSSC3230 Not Detected. Trying Again..."); // if not detected freeze
		delay(500);
	  }
  }
  

  // configure the sensor data aquisition - this must be run after .begin(), as .begin() configures the sensor to a default range, zero offset and adc resolution

  /* the zssc3230 can operate at different ranges, offsets, adc resolution and more: https://www.renesas.com/us/en/document/dst/zssc3230-datasheet?r=465426 (pg 31 to see more)
  
  The range is written as "RANGE_integer-part_pF_fractional-part" as the 3rd argument in the configure_sensor function
  so a range of +/- 12.5 pf is written as RANGE_12_pF_5 into the configure_sensor function
  available ranges are from +/-0.5pF to +/-16pF in increments of 0.5pF

  The ADC has 4 resolutions: 12 bit, 14 but, 16 bit and 18 bit
  this is written as ADC_resolution_BIT as the 5th argument in the configure_sensor

  The zero shift offset is written as "OFFSET_integer-part_pF_fractional-part" as the 6th argument in the configure_sensor function
  so an offset of 4.75pF from 0pF is written as OFFSET_4_pF_75
  available offsets are from 0pF to 15.75pF in increments of 0.25pF


  */

  // set sensor configuration to be differential mode, sensor leakage cancellation off, Range = +/- 15pF, noise mode off, 18 bit ADC and 15pF zero shift offset
  zssc3230.configure_sensor(TYPE_DIFFERENTIAL, LEAKAGE_CANCELLATION_OFF, RANGE_15_pF_0, NOISE_MODE_OFF, ADC_18_BIT, OFFSET_15_pF_0);
}

void loop() {
  // read the raw ADC capacitance as a signed integer with range [2^(N-1), 2^(N-1)-1] where N is the bit resolution of the ADC
  Serial.println(zssc3230.read_raw_cap());
}
