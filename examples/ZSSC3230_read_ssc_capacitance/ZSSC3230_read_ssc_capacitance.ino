/*
Reading the ZSSC3230 capacitance to digital converter from Renesas
By: Alex Dunn @ UOW
Date: January 22, 2024

This example shows how to read the signal conditioned capacitance from the ZSSC3230
Before running this example and getting meaninful results, the sensor should be calibrated
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
  
  
}

void loop() {
  // read the sensor signal conditioned (SSC) capcitance value
  // this should only be done after proper callibration of the sensor
  Serial.println(zssc3230.read_ssc_cap());
}
