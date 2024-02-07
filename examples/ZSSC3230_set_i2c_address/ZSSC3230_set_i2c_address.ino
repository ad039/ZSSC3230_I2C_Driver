/*
Reading the ZSSC3230 capacitance to digital converter from Renesas
By: Alex Dunn @ UOW
Date: January 22, 2024

This example shows how to change the i2c address of the ZSSC3230
*/

#include <ZSSC3230.h>
#include <Wire.h>

ZSSC3230 zssc3230;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
     delay(10);

  Wire.begin();

  //Scan bus looking for a sensor
  byte currentAddress;
  for (currentAddress = 1; currentAddress < 127; currentAddress++)
  {
    currentAddress = findI2CDevice(currentAddress); //Start scanning at last address
    if (currentAddress == 0)
      break; //No device found!
    if (zssc3230.begin(currentAddress) == true)
      break; //Device found!
  }

  if (currentAddress == 0 || currentAddress == 127)
  {
    Serial.println("No Flex Sensors found on the I2C bus. Freezing...");
    while (1)
      ;
  }

  
  if (zssc3230.begin(currentAddress)){
    Serial.print("ZSSC3230 found at address 0x");
    Serial.print(currentAddress, HEX);
    Serial.print(" / ");
    Serial.print(currentAddress); //Print decimal
    Serial.println("(decimal)");

    byte newAddress = 0;
    while (1)
    {
      while (Serial.available())
        Serial.read(); //Trash any incoming chars
      Serial.println("Enter the address you'd like to change to in decimal. Valid is 8 to 119.");
      while (Serial.available() == false)
        ; //Wait for user to send character

      newAddress = Serial.parseInt(); //Get decimal address from user
      if (newAddress >= 8 && newAddress <= 119)
        break; //Address is valid
      Serial.println("Invalid address. Please try again.");
    }

    zssc3230.set_i2c_address(newAddress); //Change I2C address of this device
    //Valid addresses are 0x08 to 0x77 - 111 possible addresses!
    //Device's I2C address is stored to memory and loaded on each power-on

    delay(100); //Time required for device to record address to EEPROM and re-init its I2C

    if (zssc3230.begin(newAddress) == true)
    {
      Serial.print("Address successfully changed to 0x");
      Serial.print(newAddress, HEX);
      Serial.print(" / ");
      Serial.print(newAddress); //Print decimal
      Serial.println("(decimal)");
      Serial.print("Now load another example sketch using .begin(0x");
      Serial.print(newAddress, HEX);
      Serial.println(") to use this ZSSC3230");
      Serial.println("Freezing...");
      while (1)
        ;
    }
  }
  
  //Something went wrong, begin scanning I2C bus for valid addresses
  Serial.println("Address change failed. Beginning an I2C scan.");
  
}

void loop() 
{
  Serial.println("Scanning...");

  byte found = 0;
  for (byte address = 1; address < 127; address++)
  {
    address = findI2CDevice(address); //Scans bus starting from given address. Returns address of discovered device.

    if (address > 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 0x0F)
        Serial.print("0"); //Pretty print
      Serial.print(address, HEX);
      Serial.print(" / ");
      Serial.print(address); //Print decimal
      Serial.println("(decimal)");

      found++;
    }
    else
    {
      if (found == 0)
        Serial.println("No I2C devices found\n");
      break; //Done searching
    }
  }

  delay(5000);
}

//Scans the ICC bus looking for devices
//Start scanning from a given address
byte findI2CDevice(byte startingAddress)
{
  if (startingAddress == 0)
    startingAddress = 1; //Error check

  for (byte address = startingAddress; address < 127; address++)
  {
    Wire.beginTransmission(address);
    byte response = Wire.endTransmission();

    if (response == 0) //A device acknowledged us at this address!
      return (address);
  }

  return (0); //No device found
}
