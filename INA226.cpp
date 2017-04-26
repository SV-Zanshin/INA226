/*******************************************************************************************************************
** INA class method definitions.                                                                                  **
**                                                                                                                **
** Detailed documentation can be found on the GitHub Wiki pages at https://github.com/SV-Zanshin/INA226/wiki      **
**                                                                                                                **
** This program is free software: you can redistribute it and/or modify it under the terms of the GNU General     **
** Public License as published by the Free Software Foundation, either version 3 of the License, or (at your      **
** option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY     **
** WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the   **
** GNU General Public License for more details. You should have received a copy of the GNU General Public License **
** along with this program.  If not, see <http://www.gnu.org/licenses/>.                                          **
**                                                                                                                **
*******************************************************************************************************************/
#include "INA226.h"                                                           // Include the header definition    //
#include <Wire.h>                                                             // I2C Library definition           //
/*******************************************************************************************************************
** Class Constructor instantiates the class but otherwise does nothing, that is performed in "begin"              **
*******************************************************************************************************************/
INA226_Class::INA226_Class() {}                                               //                                  //
/*******************************************************************************************************************
** Class Destructor currently does nothing and is included for compatibility purposes                             **
*******************************************************************************************************************/
INA226_Class::~INA226_Class() {} // of unused class destructor                //                                  //
/*******************************************************************************************************************
** Method begin() does all of the initialization work                                                             **
*******************************************************************************************************************/
void INA226_Class::begin(const uint8_t maxBusAmps, const uint32_t microOhmR) { // Class initializer                //
  Wire.begin();                                                               // Start the I2C wire subsystem     //
  for(_DeviceAddress = 1;_DeviceAddress<127;_DeviceAddress++) {               // Loop for each possible address   //
    Wire.beginTransmission(_DeviceAddress);                                   // See if something is at address   //
    if (Wire.endTransmission() == 0) {                                        // by checking the return error     //
      writeWord(INA_CONFIGURATION_REGISTER,INA_RESET_DEVICE);                 // Force INAs to reset              //
      delay(10);                                                              // Wait for INA to finish resetting //
      if (readWord(INA_CONFIGURATION_REGISTER)==INA_DEFAULT_CONFIGURATION) {  // Yes, we've found an INA226!      //
        _Current_LSB = (uint32_t)maxBusAmps*1000000000/32767;                 // Get the best possible LSB        //
        _Calibration = 51200000/((int64_t)_Current_LSB*microOhmR/100000);      // Compute calibration register     //
        _Power_LSB   = (uint32_t)25*_Current_LSB;                             // Fixed multiplier for INA219      //
        writeWord(INA_CALIBRATION_REGISTER,_Calibration);                     // Write the calibration value      //
        break;                                                                // Stop searching                   //
      } // of if-then we've found an INA226                                   //                                  //
    } // of if-then we have found a live device                               //                                  //
  } // for-next each possible I2C address                                     //                                  //
} // of method begin()                                                        //                                  //
/*******************************************************************************************************************
** Method readByte reads 1 byte from the specified address                                                        **
*******************************************************************************************************************/
uint8_t INA226_Class::readByte(const uint8_t addr) {                          //                                  //
  Wire.beginTransmission(_DeviceAddress);                                     // Address the I2C device           //
  Wire.write(addr);                                                           // Send the register address to read//
  _TransmissionStatus = Wire.endTransmission();                               // Close transmission               //
  delayMicroseconds(I2C_DELAY);                                               // delay required for sync          //
  Wire.requestFrom(_DeviceAddress, (uint8_t)1);                               // Request 1 byte of data           //
  while(!Wire.available());                                                   // Wait until the byte is ready     //
  return Wire.read();                                                         // read it and return it            //
} // of method readByte()                                                     //                                  //
/*******************************************************************************************************************
** Method readWord reads 2 bytes from the specified address                                                       **
*******************************************************************************************************************/
int16_t INA226_Class::readWord(const uint8_t addr) {                          //                                  //
  int16_t returnData;                                                         // Store return value               //
  Wire.beginTransmission(_DeviceAddress);                                     // Address the I2C device           //
  Wire.write(addr);                                                           // Send the register address to read//
  _TransmissionStatus = Wire.endTransmission();                               // Close transmission               //
  delayMicroseconds(I2C_DELAY);                                               // delay required for sync          //
  Wire.requestFrom(_DeviceAddress, (uint8_t)2);                               // Request 2 consecutive bytes      //
  while(!Wire.available());                                                   // Wait until the byte is ready     //
  returnData  = Wire.read();                                                  // Read the msb                     //
  returnData  = returnData<<8;                                                // shift the data over              //
  returnData |= Wire.read();                                                  // Read the lsb                     //
  return returnData;                                                          // read it and return it            //
} // of method readWord()                                                     //                                  //
/*******************************************************************************************************************
** Method writeByte write 1 byte to the specified address                                                         **
*******************************************************************************************************************/
void INA226_Class::writeByte(const uint8_t addr, const uint8_t data) {        //                                  //
  Wire.beginTransmission(_DeviceAddress);                                     // Address the I2C device           //
  Wire.write(addr);                                                           // Send register address to write   //
  Wire.write(data);                                                           // Send the data to write           //
  _TransmissionStatus = Wire.endTransmission();                               // Close transmission               //
} // of method writeByte()                                                    //                                  //
/*******************************************************************************************************************
** Method writeWord writes 2 byte to the specified address                                                        **
*******************************************************************************************************************/
void INA226_Class::writeWord(const uint8_t addr, const uint16_t data) {       //                                  //
  Wire.beginTransmission(_DeviceAddress);                                     // Address the I2C device           //
  Wire.write(addr);                                                           // Send register address to write   //
  Wire.write((uint8_t)(data>>8));                                             // Write the first byte             //
  Wire.write((uint8_t)data);                                                  // and then the second              //
  _TransmissionStatus = Wire.endTransmission();                               // Close transmission               //
} // of method writeWord()                                                    //                                  //
/*******************************************************************************************************************
** Method getBusMilliVolts retrieves the bus voltage measurement                                                  **
*******************************************************************************************************************/
uint16_t INA226_Class::getBusMilliVolts(const bool waitSwitch=false) {        //                                  //
  if (waitSwitch) waitForConversion();                                        // wait for conversion to complete  //
  uint16_t busVoltage = readWord(INA_BUS_VOLTAGE_REGISTER);                   // Get the raw value and apply      //
  busVoltage = (uint32_t)busVoltage*INA_BUS_VOLTAGE_LSB/100;                  // conversion to get milliVolts     //
  if (!bitRead(_OperatingMode,2) && bitRead(_OperatingMode,1)) {              // If triggered mode and bus active //
    int16_t configRegister = readWord(INA_CONFIGURATION_REGISTER);            // Get the current register         //
    writeWord(INA_CONFIGURATION_REGISTER,configRegister);                     // Write back to trigger next       //
  } // of if-then triggered mode enabled                                      //                                  //
  return(busVoltage);                                                         // return computed milliVolts       //
} // of method getBusMilliVolts()                                             //                                  //
/*******************************************************************************************************************
** Method getShuntMicroVolts retrieves the shunt voltage measurement                                              **
*******************************************************************************************************************/
int16_t INA226_Class::getShuntMicroVolts(const bool waitSwitch=false) {       //                                  //
  if (waitSwitch) waitForConversion();                                        // wait for conversion to complete  //
  int16_t shuntVoltage = readWord(INA_SHUNT_VOLTAGE_REGISTER);                // Get the raw value                //
  shuntVoltage = shuntVoltage*INA_SHUNT_VOLTAGE_LSB/10;                       // Convert to microvolts            //
  if (!bitRead(_OperatingMode,2) && bitRead(_OperatingMode,0)) {              // If triggered and shunt active    //
    int16_t configRegister = readWord(INA_CONFIGURATION_REGISTER);            // Get the current register         //
    writeWord(INA_CONFIGURATION_REGISTER,configRegister);                     // Write back to trigger next       //
  } // of if-then triggered mode enabled                                      //                                  //
  return(shuntVoltage);                                                       // return computed microvolts       //
} // of method getShuntMicroVolts()                                           //                                  //
/*******************************************************************************************************************
** Method getBusMicroAmps retrieves the computed current in microamps.                                            **
*******************************************************************************************************************/
int32_t INA226_Class::getBusMicroAmps() {                                     //                                  //
  int32_t microAmps = readWord(INA_CURRENT_REGISTER);                         // Get the raw value                //
  microAmps = (int64_t)microAmps*_Current_LSB/1000;                           // Convert to microamps             //
  return(microAmps);                                                          // return computed microamps        //
} // of method getBusMicroAmps()                                              //                                  //
/*******************************************************************************************************************
** Method getBusMicroWatts retrieves the computed power in milliwatts                                             **
*******************************************************************************************************************/
int32_t INA226_Class::getBusMicroWatts() {                                    //                                  //
  int32_t microWatts = readWord(INA_POWER_REGISTER);                          // Get the raw value                //
  microWatts = (int64_t)microWatts*_Power_LSB/1000;                           // Convert to milliwatts            //
  return(microWatts);                                                         // return computed milliwatts       //
} // of method getBusMicroWatts()                                             //                                  //
/*******************************************************************************************************************
** Method setAveraging sets the hardware averaging for the different devices                                      **
*******************************************************************************************************************/
void INA226_Class::setAveraging(const uint16_t averages = UINT16_MAX) {       // Set the number of averages taken //
  uint8_t averageIndex;                                                       // Store indexed value for register //
  int16_t configRegister = readWord(INA_CONFIGURATION_REGISTER);              // Get the current register         //
  if      (averages>=1024) averageIndex = 7;                                  // setting depending upon range     //
  else if (averages>= 512) averageIndex = 6;                                  //                                  //
  else if (averages>= 256) averageIndex = 5;                                  //                                  //
  else if (averages>= 128) averageIndex = 4;                                  //                                  //
  else if (averages>=  64) averageIndex = 3;                                  //                                  //
  else if (averages>=  16) averageIndex = 2;                                  //                                  //
  else if (averages>=   4) averageIndex = 1;                                  //                                  //
  else                     averageIndex = 0;                                  //                                  //
  configRegister &= ~INA_CONFIG_AVG_MASK;                                     // zero out the averages part       //
  configRegister |= (uint16_t)averageIndex << 9;                              // shift in the averages to register//
  writeWord(INA_CONFIGURATION_REGISTER,configRegister);                       // Save new value                   //
} // of method setAveraging()                                                 //                                  //
/*******************************************************************************************************************
** Method setBusConversion specifies the conversion rate (see datasheet for 8 distinct values) for the bus        **
*******************************************************************************************************************/
void INA226_Class::setBusConversion(uint8_t convTime = UINT8_MAX) {           // Set timing for Bus conversions   //
  if (convTime>7) convTime=7;                                                 // Use maximum value allowed        //
  int16_t configRegister = readWord(INA_CONFIGURATION_REGISTER);              // Get the current register         //
  configRegister &= ~INA_CONFIG_BUS_TIME_MASK;                                // zero out the Bus conversion part //
  configRegister |= (uint16_t)convTime << 6;                                  // shift in the averages to register//
  writeWord(INA_CONFIGURATION_REGISTER,configRegister);                       // Save new value                   //
} // of method setBusConversion()                                             //                                  //
/*******************************************************************************************************************
** Method setShuntConversion specifies the conversion rate (see datasheet for 8 distinct values) for the shunt    **
*******************************************************************************************************************/
void INA226_Class::setShuntConversion(uint8_t convTime = UINT8_MAX) {         // Set timing for Bus conversions   //
  if (convTime>7) convTime=7;                                                 // Use maximum value allowed        //
  int16_t configRegister = readWord(INA_CONFIGURATION_REGISTER);              // Get the current register         //
  configRegister &= ~INA_CONFIG_SHUNT_TIME_MASK;                              // zero out the Bus conversion part //
  configRegister |= (uint16_t)convTime << 3;                                  // shift in the averages to register//
  writeWord(INA_CONFIGURATION_REGISTER,configRegister);                       // Save new value                   //
} // of method setShuntConversion()                                           //                                  //
/*******************************************************************************************************************
** Method waitForConversion loops until the current conversion is marked as finished. If the conversion has       **
** completed already then the flag (and interrupt pin, if activated) is also reset.                               **
*******************************************************************************************************************/
void INA226_Class::waitForConversion() {                                      // Wait for current conversion      //
  uint16_t conversionBits = 0;                                                //                                  //
  while(conversionBits==0) {                                                  //                                  //
    conversionBits = readWord(INA_MASK_ENABLE_REGISTER)&(uint16_t)8;          //                                  //
  } // of while the conversion hasn't finished                                //                                  //
} // of method waitForConversion()                                            //                                  //
/*******************************************************************************************************************
** Method setAlertPinOnConversion configure the INA226 to pull the ALERT pin low when a conversion is complete    **
*******************************************************************************************************************/
void INA226_Class::setAlertPinOnConversion(const bool alertState) {           // Enable pin change on conversion  //
  uint16_t alertRegister = readWord(INA_MASK_ENABLE_REGISTER);                // Get the current register         //
  if (!alertState) alertRegister &= ~((uint16_t)1<<10);                       // zero out the alert bit           //
              else alertRegister |= (uint16_t)(1<<10);                        // turn on the alert bit            //
  writeWord(INA_MASK_ENABLE_REGISTER,alertRegister);                          // Write register back to device    //
} // of method setAlertPinOnConversion                                        //                                  //
/*******************************************************************************************************************
** Method reset resets the INA226 using the first bit in the configuration register                               **
*******************************************************************************************************************/
void INA226_Class::reset() {                                                  // Reset the INA226                 //
  writeWord(INA_CONFIGURATION_REGISTER,0x8000);                               // Set most significant bit         //
  delay(I2C_DELAY);                                                           // Let the INA226 reboot            //
} // of method reset                                                          //                                  //
/*******************************************************************************************************************
** Method setMode allows the various mode combinations to be set. If no parameter is given the system goes back   **
** to the default startup mode.                                                                                   **
*******************************************************************************************************************/
void INA226_Class::setMode(uint8_t mode = 7) {                                // Set the monitoring mode          //
  int16_t configRegister = readWord(INA_CONFIGURATION_REGISTER);              // Get the current register         //
  configRegister &= ~INA_CONFIG_MODE_MASK;                                    // zero out the mode bits           //
  mode = B00001111 & mode;                                                    // Mask off unused bits             //
  configRegister |= mode;                                                     // shift in the mode settings       //
  writeWord(INA_CONFIGURATION_REGISTER,configRegister);                       // Save new value                   //
  _OperatingMode = mode;                                                      // Save the operating mode          //
} // of method setMode()                                                      //                                  //
