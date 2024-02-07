/*
Reading the ZSSC3230 capacitance to digital converter from Renesas
By: Alex Dunn @ UOW
Date: January 22, 2024

This example shows how to set the sensor in cyclic mode and read its ssc capacitance once every 10ms
Before running this example and getting meaninful results, the sensor should be calibrated
 */

#include <ZSSC3230_I2C_Driver.h>
#include <Wire.h>

// create a ZSSC3230 object
ZSSC3230 zssc3230;

// define a value to store the previous microsecond timestamp for timing
long prevMicros = 0;

void setup() {
  // begin serial communication
  Serial.begin(115200);
  while (!Serial)
     delay(10);

  // begin wire communication
  Wire.begin();   
  Wire.setClock(400000); // increase the i2c clock speed to fast mode (400kHz)

  // initialise the zssc3230 sensor
  while(1) {
    if (zssc3230.begin()) {
      Serial.println("ZSSC3230 Detected");
      break;
    }
    else {
      Serial.println("ZSSC3230 Not Detected. Trying Again"); // if not detected, try again
      delay(500);
    }
  }

  // place the zssc3230 into cyclic mode
  if (zssc3230.cyclic_mode() == 0) {
    Serial.println("Entered cyclic mode successfully");
  }
  else {
    Serial.println("Could not enter cyclic mode. Freezing...");
    while(1)
      ;
  }
    
  
}

void loop() {
  // read the sensor signal conditioned (SSC) capcitance value every 10 ms
  if (micros() - prevMicros > 10000){
    prevMicros = micros();
    Serial.println(zssc3230.read_ssc_cap_cyc());
  }
  
}
