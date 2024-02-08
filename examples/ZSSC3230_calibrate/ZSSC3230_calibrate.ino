/*
Reading the ZSSC3230 capacitance to digital converter from Renesas
By: Alex Dunn @ UOW
Date: January 22, 2024

This example shows how to calibrate the ZSSC3230 sensor using the 
three point, single temperature calibration method
For this you will need three known capacitors within the range you want to measure
You will also need to download the ZSSC323x Evaluation Software available at: https://www.renesas.com/us/en/products/sensor-products/sensor-signal-conditioners-ssc-afe/zssc3230-low-power-high-resolution-capacitive-sensor-signal-conditioner#design_development
 */

#include <ZSSC3230_I2C_Driver.h>
#include <Wire.h>

// create a ZSSC3230 object
ZSSC3230 zssc3230;

// Coeeficient Varibles
int32_t Offset_S, Gain_S, SOT_S;



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
  

  Serial.println();
  Serial.println("This is a calibration guide for the ZSSC3230. Please ensure you have: ");
  Serial.println("  -Three accurate capacitors within your desired range");
  Serial.println("  -Installed the ZSSC323x Evaluation Software");
  Serial.println("  -Configured the ZSSC3230 in the Arduino script to your desired range and offset");
  Serial.println();

  delay(10);

  // Configure the sensor as you desire. This is important as the calibration will be done for this specific configuration
  zssc3230.configure_sensor(TYPE_DIFFERENTIAL, LEAKAGE_CANCELLATION_OFF, RANGE_15_pF_0, NOISE_MODE_OFF, ADC_12_BIT, OFFSET_15_pF_0);
  
  delay(10);

  // record the first capacitance point:
  // ask for capacitor 1 to be placed into the sensor and wait for the user to begin recording
  while (Serial.available())
        Serial.read(); //Trash any incoming chars
  Serial.println("1. Place your first known capacitor into the sensor. Press any key when ready... ");
  delay(10);
  while (!Serial.available()) {
    delay(10);
  }

  // record an averaged adc reading
  Serial.print("The Averaged ADC value for this capacitance is: ");
  Serial.print(average_raw_cap());
  Serial.println(". Please record this value!");

  // record the second capacitance point:
  // ask for capacitor 2 to be placed into the sensor and wait for the user to begin recording
  while (Serial.available())
        Serial.read(); //Trash any incoming chars
  Serial.println("2. Place your second known capacitor into the sensor. Press any key when ready... ");
  while (!Serial.available()) {
    delay(10);
  }

  // record an averaged adc reading
  Serial.print("The Averaged ADC value for this capacitance is: ");
  Serial.print(average_raw_cap());
  Serial.println(". Please record this value!");

  // record the third capacitance point:
  // ask for capacitor 3 to be placed into the sensor and wait for the user to begin recording
  while (Serial.available())
        Serial.read(); //Trash any incoming chars
  Serial.println("3. Place your third known capacitor into the sensor. Press any key when ready... ");
  while (!Serial.available()) {
    delay(10);
  }

  // record an averaged adc reading
  Serial.print("The Averaged ADC value for this capacitance is: ");
  Serial.print(average_raw_cap());
  Serial.println(". Please record this value!");

  Serial.println();
  Serial.println("4. Now open the ZSSC323x Evaluation Software->Calibration tab");
  Serial.println("5. In the 'Type' dropdown, select 3 Points: S(O + G + SOT) and make sure Curve is set to Parabolic");
  Serial.println("6. Enter the percentage of the full scale input for each of your target capacitances.");
  Serial.println("    e.g if range is set as +/-15pF and zero offset is 15pF, a 10pF capacitor would be 33%");
  Serial.println("7. Click 'Calculate Coefficients'");

  delay(1000);

  while(1) {

    while (Serial.available())
        Serial.read(); //Trash any incoming chars
    Serial.println("Please enter your OFFSET_S value:");
    while (Serial.available() == false)
      ; //Wait for user to send character

    Offset_S = Serial.parseInt(); //Get Offset_s from user

    delay(1000);

    while (Serial.available())
        Serial.read(); //Trash any incoming chars
    Serial.println("Please enter your GAIN_S value:");
    while (Serial.available() == false)
      ; //Wait for user to send character

    Gain_S = Serial.parseInt(); //Get Gain_s from user
    delay(1000);

    while (Serial.available())
        Serial.read(); //Trash any incoming chars
    Serial.println("Please enter your SOT_S value:");
    while (Serial.available() == false)
      ; //Wait for user to send character

    SOT_S = Serial.parseInt(); //Get SOT_s from user
    delay(1000);

    Serial.print("Are these the values you entered? OFFSET_S = ");
    Serial.print(Offset_S);
    Serial.print(", GAIN_S = ");
    Serial.print(Gain_S);
    Serial.print(", SOT_S = ");
    Serial.print(SOT_S);
    Serial.println(". Enter y if yes, enter n if no: ");
    while (Serial.available())
        Serial.read(); //Trash any incoming chars
    while (Serial.available() == false)
      ;
    byte incoming = Serial.read();

    // leave the loop if the user is happy with their choice
    if (incoming == 'y')
      break;
  
  }

  delay(1000);

  // run calibrate_sensor function. This takes the Offset_S, the Gain_S and the SOT_S as inputs.  
  if (zssc3230.calibrate_zssc3230(Offset_S, Gain_S, SOT_S)){
    Serial.println("Calibrated successfully!");
  }
  else {
    Serial.println("Calibration failed, please try again");
  }

  delay(1000);
}

void loop() {
  // beign printing out ssc capacitance values
  Serial.println(zssc3230.read_ssc_cap());
}


int32_t average_raw_cap() {
    // take 100 samples of the capacitance and average them
  int32_t sum = 0;

  for(int i = 0; i <= 100; i++) {
    sum += zssc3230.read_raw_cap();
  }

  return sum/100;
}
