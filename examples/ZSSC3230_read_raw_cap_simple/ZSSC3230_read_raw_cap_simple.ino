/*
Reading the ZSSC3230 capacitance to digital converter from Renesas
By: Alex Dunn @ UOW
Date: January 22, 2024

This example shows how to read the raw analog to digital converted (ADC) capacitance value from the ZSSC3230
The values are read at the default range +/-15pF, default zero shift offset +15pF and default ADC resolution 12 bit
*/

#include <ZSSC3230.h>
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
  // read the raw ADC capacitance as a signed integer with range [2^(N-1), 2^(N-1)-1] where N is the bit resolution of the ADC
  Serial.println(zssc3230.read_raw_cap());
}
