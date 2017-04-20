/*******************************************************************************************************************
** Class definition header for the INA226 class. This library allows access to the INA226 High-Side or Low-Side   **
** Measurement, Bi-Directional Current and Power Monitor with I2C Compatible Interface. The datasheet can be      **
** download from Texas Instruments at http://www.ti.com/lit/ds/symlink/ina226.pdf. While there are breakout boards**
** for the little brother INA219 along with sample libraries, I had a need for a device that would take over 28V  **
** and found that this chip could not only handle the higher voltage but was also significantly more accurate.    **
**                                                                                                                **
** Detailed documentation can be found on the GitHub Wiki pages at https://github.com/SV-Zanshin/INA226/wiki      **
**                                                                                                                **
** The INA226 requires an external shunt of known resistance to be placed across the high-side or low-side supply **
** or ground line and it uses the small current generated by the shunt to compute the amperage going through the  **
** circuit.  This value, coupled with the voltage measurement, allows the Amperage and Wattage to be computed by  **
** the INA226 and all of these values can be read using the industry standard I2C protocol.                       **
**                                                                                                                **
** Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated   **
** documentation files (the "Software"), to deal in the Software without restriction, including without limitation**
** the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,   **
** and to permit persons to whom the Software is furnished to do so, subject to the following conditions:         **
** The above copyright notice and this permission notice shall be included in all copies or substantial portions  **
** of the Software.                                                                                               **
**                                                                                                                **
** Although programming for the Arduino and in c/c++ is new to me, I'm a professional programmer and have learned,**
** over the years, that it is much easier to ignore superfluous comments than it is to decipher non-existent ones;**
** so both my comments and variable names tend to be verbose. The code is written to fit in the first 80 spaces   **
** and the comments start after that and go to column 117 - allowing the code to be printed in A4 landscape mode. **
**                                                                                                                **
** This program is free software: you can redistribute it and/or modify it under the terms of the GNU General     **
** Public License as published by the Free Software Foundation, either version 3 of the License, or (at your      **
** option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY     **
** WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the   **
** GNU General Public License for more details. You should have received a copy of the GNU General Public License **
** along with this program.  If not, see <http://www.gnu.org/licenses/>.                                          **
**                                                                                                                **
** Vers.  Date       Developer           Comments                                                                 **
** ====== ========== =================== ======================================================================== **
** 1.0.0  2017-01-10 Arnd@SV-Zanshin.Com Fixed library file name, added constants for setMode() call              **
** 1.0.0  2017-01-09 Arnd@SV-Zanshin.Com Added reset() and setMode() calls                                        **
** 1.0.b2 2017-01-08 Arnd@SV-Zanshin.Com Removed INA219 code, concentrating on only the INA226                    **
** 1.0.b1 2017-01-05 Arnd@SV-Zanshin.Com Created class                                                            **
**                                                                                                                **
*******************************************************************************************************************/
#include "Arduino.h"                                                          // Arduino data type definitions    //
#ifndef INA226_Class_h                                                        // Guard code definition            //
  #define INA226__Class_h                                                     // Define the name inside guard code//
  /*****************************************************************************************************************
  ** Declare constants used in the class                                                                          **
  *****************************************************************************************************************/
  const uint8_t  I2C_DELAY                    =     10;                       // Microsecond delay on write       //
  const uint8_t  INA_CONFIGURATION_REGISTER   =      0;                       // Registers common to all INAs     //
  const uint8_t  INA_SHUNT_VOLTAGE_REGISTER   =      1;                       //                                  //
  const uint8_t  INA_BUS_VOLTAGE_REGISTER     =      2;                       //                                  //
  const uint8_t  INA_POWER_REGISTER           =      3;                       //                                  //
  const uint8_t  INA_CURRENT_REGISTER         =      4;                       //                                  //
  const uint8_t  INA_CALIBRATION_REGISTER     =      5;                       //                                  //
  const uint8_t  INA_MASK_ENABLE_REGISTER     =      6;                       //                                  //
  const uint16_t INA_RESET_DEVICE             = 0x8000;                       // Write to configuration to reset  //
  const uint16_t INA_DEFAULT_CONFIGURATION    = 0x4127;                       // Default configuration register   //
  const uint16_t INA_BUS_VOLTAGE_LSB          =    125;                       // LSB in uV *100 1.25mV            //
  const uint16_t INA_SHUNT_VOLTAGE_LSB        =     25;                       // LSB in uV *10  2.5uV             //
  const uint16_t INA_CONFIG_AVG_MASK          = 0x0E00;                       // Bits 9-11                        //
  const uint16_t INA_CONFIG_BUS_TIME_MASK     = 0x01C0;                       // Bits 6-8                         //
  const uint16_t INA_CONFIG_SHUNT_TIME_MASK   = 0x0038;                       // Bits 3-5                         //
  const uint16_t INA_CONVERSION_READY_MASK    = 0x0080;                       // Bit 4                            //
  const uint16_t INA_CONFIG_MODE_MASK         = 0x0007;                       // Bits 0-3                         //
  const uint8_t  INA_MODE_TRIGGERED_SHUNT     =   B001;                       // Triggered shunt, no bus          //
  const uint8_t  INA_MODE_TRIGGERED_BUS       =   B010;                       // Triggered bus, no shunt          //
  const uint8_t  INA_MODE_TRIGGERED_BOTH      =   B011;                       // Triggered bus and shunt          //
  const uint8_t  INA_MODE_POWER_DOWN          =   B100;                       // shutdown or power-down           //
  const uint8_t  INA_MODE_CONTINUOUS_SHUNT    =   B101;                       // Continuous shunt, no bus         //
  const uint8_t  INA_MODE_CONTINUOUS_BUS      =   B110;                       // Continuous bus, no shunt         //
  const uint8_t  INA_MODE_CONTINUOUS_BOTH     =   B111;                       // Both continuous, default value   //

  /*****************************************************************************************************************
  ** Declare class header                                                                                         **
  *****************************************************************************************************************/
  class INA226_Class {                                                        // Class definition                 //
    public:                                                                   // Publicly visible methods         //
      INA226_Class();                                                         // Class constructor                //
      ~INA226_Class();                                                        // Class destructor                 //
      void     begin(const uint8_t maxBusAmps, const uint32_t nanoOhmR);      // Class initializer                //
      uint16_t getBusMilliVolts(const bool waitSwitch=false);                 // Retrieve Bus voltage in mV       //
      int16_t  getShuntMicroVolts(const bool waitSwitch=false);               // Retrieve Shunt voltage in uV     //
      int32_t  getBusMicroAmps();                                             // Retrieve microamps               //
      int32_t  getBusMicroWatts();                                            // Retrieve microwatts              //
      void     reset();                                                       // Reset the device                 //
      void     setMode(uint8_t mode = 7);                                     // Set the monitoring mode          //
      void     setAveraging(const uint16_t averages = UINT16_MAX);            // Set the number of averages taken //
      void     setBusConversion(uint8_t convTime = UINT8_MAX);                // Set timing for Bus conversions   //
      void     setShuntConversion(uint8_t convTime = UINT8_MAX);              // Set timing for Shunt conversions //
      void     setAlertPinOnConversion(const bool alertState);                // Enable pin change on conversion  //
      void     waitForConversion();                                           // wait for conversion to complete  //
    private:                                                                  // Private variables and methods    //
      uint8_t  readByte(const uint8_t addr);                                  // Read a byte from an I2C address  //
      int16_t  readWord(const uint8_t addr);                                  // Read a word from an I2C address  //
      void     writeByte(const uint8_t addr, const uint8_t data);             // Write a byte to an I2C address   //
      void     writeWord(const uint8_t addr, const uint16_t data);            // Write two bytes to an I2C address//
      uint8_t  _DeviceAddress      = 0;                                       // First I2C address found          //
      uint8_t  _TransmissionStatus = 0;                                       // Return code for I2C transmission //
      uint16_t _Calibration        = 0;                                       // Calibration register value       //
      uint16_t _Configuration      = 0;                                       // Configuration register value     //
      int64_t _Current_LSB        = 0;                                       // Amperage LSB                     //
      uint32_t _Power_LSB          = 0;                                       // Wattage LSB                      //
      uint8_t  _OperatingMode      = B111;                                    // Default continuous mode operation//
  }; // of MicrochipSRAM class definition                                     //                                  //
#endif                                                                        //----------------------------------//
