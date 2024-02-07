/*
Reading the ZSSC3230 capacitance to digital converter from Renesas
By: Alex Dunn @ UOW
Date: January 22, 2024

This example shows how to read the signal conditioned capacitance from two ZSSC3230 device
Before running this example and getting meaninful results, the sensor should be calibrated and one of the sensor addresses changed to 0x47 using the ZSSC3230_set_i2c_address example
 */

#include <ZSSC3230_I2C_Driver.h>
#include <Wire.h>

// initialise ZSSC3230 objects
ZSSC3230 zssc3230_1;
ZSSC3230 zssc3230_2;

void setup() {
  // begin serial communication
  Serial.begin(115200);
  while (!Serial)
     delay(10);

  // begin wire communication
  Wire.begin();

  // initialise the first zssc3230 sensor at i2c address 0x47
  while(1) {
	  if (zssc3230_1.begin(0x47)) {
		Serial.println("ZSSC3230_1 Detected");
		break;
	  }
	  else {
		Serial.println("ZSSC3230_2 Not Detected. Trying Again..."); // if not detected freeze
		delay(500);
	  }
  }
  

  // initialise the second zssc3230 sensor at i2c addres 0x48 (this is the default address)
  while(1) {
	  if (zssc3230_2.begin(0x48)) {
		Serial.println("ZSSC3230_2 Detected");
		break;
	  }
	  else {
		Serial.println("ZSSC3230_2 Not Detected. Trying Again..."); // if not detected freeze
		delay(500);
	  }
  }

  
}

void loop() {
  // record the current zssc3230_1 sensor signal conditioned capacitance
  float zssc3230_1_cap = zssc3230_1.read_ssc_cap();

  // record the current zssc3230_2 sensor signal conditioned capacitance
  float zssc3230_2_cap = zssc3230_2.read_ssc_cap();

  
  // build a buffer to store the text format to the serial terminal
  char buf[40];
  // generate the text format, with the recorded data from two different sensors
  sprintf(buf, "ZSSC3230_1 = %.2f, ZSSC3230_2 = %.2f", zssc3230_1_cap, zssc3230_2_cap);

 // write the buffer to the serial terminal
  Serial.println(buf);
  
}
