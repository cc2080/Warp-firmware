/*
        Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
        2018-onwards, see git log.

        All rights reserved.

        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions
        are met:

        *       Redistributions of source code must retain the above
                copyright notice, this list of conditions and the following
                disclaimer.

        *       Redistributions in binary form must reproduce the above
                copyright notice, this list of conditions and the following
                disclaimer in the documentation and/or other materials
                provided with the distribution.

        *       Neither the name of the author nor the names of its
                contributors may be used to endorse or promote products
                derived from this software without specific prior written
                permission.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
        FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
        COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
        INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
        BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
        LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
        LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
        ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
        POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

/*
 *      config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devINA219.h"

extern volatile WarpI2CDeviceState      deviceINA219State;
extern volatile uint32_t                gWarpI2cBaudRateKbps;
extern volatile uint32_t                gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t                gWarpSupplySettlingDelayMilliseconds;

uint32_t ina219_currentMultiplier_uA;
int16_t ina219_powerMultiplier_uW;
uint16_t ina219_calValue;

//must also calubrate based off the Adafruit INA219 library
void setCalibration_cc2080();

void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
        deviceINA219State.i2cAddress                   = i2cAddress;
        deviceINA219State.operatingVoltageMillivolts   = operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
        uint8_t         payloadByte[2], commandByte[1];
        i2c_status_t    returnValue;

        switch (deviceRegister)
        {
		// Not interested in config or calibration registers
		case 0x00: case 0x05:
		{
                        /* OK */
                        break;
                }

                default:
                {
                        return kWarpStatusBadDeviceCommand;
                }
        }

        i2c_device_t slave =
                {
                        .address       = deviceINA219State.i2cAddress,
                        .baudRate_kbps = gWarpI2cBaudRateKbps
		};

        warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
        commandByte[0] = deviceRegister;
        payloadByte[0] = (payload >> 8) & 0xFF; /* MSB first */
        payloadByte[1] = payload & 0xFF;        /* LSB */
	warpEnableI2Cpins(); 
	OSA_TimeDelay(1);
        returnValue    = I2C_DRV_MasterSendDataBlocking(
                0 /* I2C instance */,
                &slave,
                commandByte,
                1,
                payloadByte,
                2,
                gWarpI2cTimeoutMilliseconds);
        if (returnValue != kStatus_I2C_Success)
        {
                return kWarpStatusDeviceCommunicationFailed;
        }

        return kWarpStatusOK;
}
WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t cmdBuf[1] = {0xFF};
	i2c_status_t status1, status2;

	i2c_device_t slave =
		{
			.address       = deviceINA219State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	USED(numberOfBytes);

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	warpEnableI2Cpins();
	if (1)
	{
		/*
			Steps:
			(1)	Write transaction beginning with start condition, slave address,
				and pointer address and 2 bytes of config values to be written  
				(total payload of 3 bytes).
			(2)	Write transaction beginning with start condition, slave address,
				and pointer address. Total payload is just 1 byte. (Trigger measurement)
			(3)	Wait 10ms for conversion to complete.
			(4)	Read transaction beginning with start condition, followed by
				slave address, and read 2 byte payload
		*/

		/*
		 *	Step 1: Trigger measurement
		 */
		cmdBuf[0] = deviceRegister;

		status1 = I2C_DRV_MasterSendDataBlocking(
			0,
			&slave,
			cmdBuf,
			1,
			NULL,
			0,
			gWarpI2cTimeoutMilliseconds);

		/*
		 *	Step 2: Wait for max 1ms for conversion completion
		 */
		OSA_TimeDelay(1);

		/*
		 *	Step 3: Read register
		 */
		status2 = I2C_DRV_MasterReceiveDataBlocking(
			0 /* I2C peripheral instance */,
			&slave,
			NULL,
			0,
			(uint8_t*)deviceINA219State.i2cBuffer,
			numberOfBytes,
			gWarpI2cTimeoutMilliseconds);

		if ((status1 != kStatus_I2C_Success) || (status2 != kStatus_I2C_Success))
		{
			return kWarpStatusDeviceCommunicationFailed;
		}
	}
	else
	{
		cmdBuf[0] = deviceRegister;

		status1 = I2C_DRV_MasterReceiveDataBlocking(
			0 /* I2C peripheral instance */,
			&slave,
			cmdBuf,
			1,
			(uint8_t*)deviceINA219State.i2cBuffer,
			numberOfBytes,
			gWarpI2cTimeoutMilliseconds);

		if (status1 != kStatus_I2C_Success)
		{
			return kWarpStatusDeviceCommunicationFailed;
		}
	}

	return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
        uint16_t        readSensorRegisterValueLSB;
        uint16_t        readSensorRegisterValueMSB;
        int16_t         readSensorRegisterValueCombined;
        WarpStatus      i2cReadStatus;

	// The INA219_REG_... are defined in the devINA219.h file

        warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
        
	i2cReadStatus = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*print
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                warpPrint(" ----,");
        }
        else
        {
                if (hexModeFlag)
                {
                        warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
                }
        }

        i2cReadStatus = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                warpPrint(" ----,");
        }
        else
        {
                if (hexModeFlag)
                {
                        warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
        }
	 
	i2cReadStatus = readSensorRegisterINA219(INA219_REG_POWER, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                warpPrint(" ----,");
        }
        else
        {
                if (hexModeFlag)
                {
                        warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
                }
        }

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                warpPrint(" ----,");
        }
        else
        {
                if (hexModeFlag)
                {
                        warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
                }
        }
}

uint8_t
appendSensorDataINA219(uint8_t* buf)
{
        uint8_t index = 0;

        uint16_t readSensorRegisterValueLSB;
        uint16_t readSensorRegisterValueMSB;
        int16_t readSensorRegisterValueCombined;
        WarpStatus i2cReadStatus;

        warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
        
	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                buf[index] = 0;
                index += 1;

                buf[index] = 0;
                index += 1;
        }
	else
	{
                /*
                 * MSB first
                 */
                buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
                index += 1;

                buf[index] = (uint8_t)(readSensorRegisterValueCombined);
                index += 1;
        }

        i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                buf[index] = 0;
                index += 1;

                buf[index] = 0;
                index += 1;
        }
        else
        {
                /*
                 * MSB first
                 */
                buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
                index += 1;

                buf[index] = (uint8_t)(readSensorRegisterValueCombined);
                index += 1;
        }

	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_POWER, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                buf[index] = 0;
                index += 1;

                buf[index] = 0;
                index += 1;
        }
        else
        {
                /*
                 * MSB first
                 */
                buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
                index += 1;

                buf[index] = (uint8_t)(readSensorRegisterValueCombined);
                index += 1;
        }

	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                buf[index] = 0;
                index += 1;

                buf[index] = 0;
                index += 1;
        }
        else
        {
                /*
                 * MSB first
                 */
                buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
                index += 1;

                buf[index] = (uint8_t)(readSensorRegisterValueCombined);
                index += 1;
        }

        return index;
}

/*!
 *  @brief  Configures to INA219 to be able to measure up to 32V and 2A
 *          of current.  Each unit of current corresponds to 100uA, and
 *          each unit of power corresponds to 2mW. Counter overflow
 *          occurs at 3.2A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void setCalibration_cc2080() 
{
	// By default we use a pretty huge range for the input voltage,
  	// which probably isn't the most appropriate choice for system
  	// that don't use a lot of power.  But all of the calculations
  	// are shown below if you want to change the settings.  You will
  	// also need to change any relevant register settings, such as
  	// setting the VBUS_MAX to 16V instead of 32V, etc.

  	// VBUS_MAX = 16V             (Assumes 32V, can also be set to 16V)  
  	// VSHUNT_MAX = 0.04         (Assumes Gain 1, 320mV, can also be 0.16, 0.08,
  	// 0.04) RSHUNT = 0.1               (Resistor value in ohms)

  	// 1. Determine max possible current
  	// MaxPossible_I = VSHUNT_MAX / RSHUNT
  	// MaxPossible_I = 0.4A

  	// 2. Determine max expected current
  	// MaxExpected_I = 0.1A  of which the OLED accounts for around 25mA

  	// 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  	// MinimumLSB = MaxExpected_I/32767
  	// MinimumLSB = 3,05185e-6              (61uA per bit)
  	// MaximumLSB = MaxExpected_I/4096
  	// MaximumLSB = 24.414e-6              (488uA per bit)

  	// 4. Choose an LSB between the min and max values
  	//    (Preferrably a roundish number close to MinLSB)
  	// CurrentLSB = 0.000007 (7uA per bit), chosen to fit 

  	// 5. Compute the calibration register
  	// Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  	// Cal = 58514 (0xE492)

  	ina219_calValue = 58514;

  	// 6. Calculate the power LSB
  	// PowerLSB = 20 * CurrentLSB
  	// PowerLSB = 0.00014 (2mW per bit)

  	// 7. Compute the maximum current and shunt voltage values before overflow
  	//
  	// Max_Current = Current_LSB * 32767
  	// Max_Current = 0.229369 A before overflow
  	//
  	// If Max_Current > Max_Possible_I then
  	//    Max_Current_Before_Overflow = MaxPossible_I
  	// Else
  	//    Max_Current_Before_Overflow = Max_Current
  	// End If
  	// 
  	// Max_Current_Before_Overflow = Max_Current
  	// Max_Current_Before_Overflow = 0.229369 A
  	//
  	// Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  	// Max_ShuntVoltage = 0.0229369 V
  	//
  	// If Max_ShuntVoltage >= VSHUNT_MAX
  	//    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  	// Else
  	//    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  	// End If
  	//
  	//Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  	//Max_ShuntVoltage_Before_Overflow = 0.0229369 V
  	//
	
	// 8. Compute the Maximum Power
 	// MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
 	// MaximumPower = 0.229369 * 16V
  	// MaximumPower = 3.6699W

  	// Set multipliers to convert raw current/power values
  	ina219_currentMultiplier_uA = 7; // Current LSB = 7uA per bit
  	ina219_powerMultiplier_uW = 140; // Power LSB = 140uW per bit

  	// Set Calibration register to 'Cal' calculated above
  	writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);

  	// Set Config register to take into account the settings above
  	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  
	//  Adafruit_BusIO_Register config_reg =
	//     Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
	//  _success = config_reg.write(config, 2);

 	writeSensorRegisterINA219(INA219_REG_CONFIG, config);
}

int16_t getBusVoltage_raw_INA219() 
{	
	uint16_t value;

	//  Adafruit_BusIO_Register bus_voltage_reg =
	//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_BUSVOLTAGE, 2, MSBFIRST);
	//  _success = bus_voltage_reg.read(&value);
	
	readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2);
	value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1]; 

	// Shift to the right 3 to drop CNVR and OVF and multiply by LSB
	
	return (int16_t)((value >> 3) * 4);
}

/*!
 *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 *  @return the raw shunt voltage reading
 */
int16_t getShuntVoltage_raw_INA219() 
{
	uint16_t value;
	//  Adafruit_BusIO_Register shunt_voltage_reg =
	//     Adafruit_BusIO_Register(i2c_dev, INA219_REG_SHUNTVOLTAGE, 2, MSBFIRST);
	//  _success = shunt_voltage_reg.read(&value);
  	readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2);
	value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];
  
	return value;
}

/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
int16_t getCurrent_raw_INA219() 
{
	uint16_t value;

  	// Sometimes a sharp load will reset the INA219, which will
  	// reset the cal register, meaning CURRENT and POWER will
  	// not be available ... avoid this by always setting a cal
  	// value even if it's an unfortunate extra step
  	
	//Adafruit_BusIO_Register calibration_reg =
      	//Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  	//calibration_reg.write(ina219_calValue, 2);

	writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);

  	// Now we can safely read the CURRENT register!
  	
	// Adafruit_BusIO_Register current_reg =
      	// Adafruit_BusIO_Register(i2c_dev, INA219_REG_CURRENT, 2, MSBFIRST);
  	// _success = current_reg.read(&value);
  	
	readSensorRegisterINA219(INA219_REG_CURRENT, 2);
	value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];
	return value;
}

/*!
 *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
 *  @return raw power reading
 */
int16_t getPower_raw_INA219() 
{
  	uint16_t value;

  	// Sometimes a sharp load will reset the INA219, which will
  	// reset the cal register, meaning CURRENT and POWER will
  	// not be available ... avoid this by always setting a cal
  	// value even if it's an unfortunate extra step
  
  	
  	//Adafruit_BusIO_Register calibration_reg =
  	//    Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  	//calibration_reg.write(ina219_calValue, 2);
	
	writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);
  	
	// Now we can safely read the POWER register!
  	
	//Adafruit_BusIO_Register power_reg =
      	//	Adafruit_BusIO_Register(i2c_dev, INA219_REG_POWER, 2, MSBFIRST);
  	//_success = power_reg.read(&value);
  
	readSensorRegisterINA219(INA219_REG_POWER, 2);
	value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];


	return value;
}

/*!
 *  @brief  Gets the shunt voltage in uV (so +-327000uV)
 *  @return the shunt voltage converted to microvolts
 */
float getShuntVoltage_uV_INA219() {
  	int16_t value;
  	value = getShuntVoltage_raw_INA219();
  	return value * 10;
}

/*!
 *  @brief  Gets the bus voltage in millivolts
 *  @return the bus voltage converted to millivolts
 */
float getBusVoltage_mV_INA219() {
  	int16_t value = getBusVoltage_raw_INA219();
  	return value;
}

/*!
 *  @brief  Gets the current value in uA, taking into account the
 *          config settings and current LSB
 *  @return the current reading convereted to microamps
 */
float getCurrent_uA_INA219() {
  	float valueDec = getCurrent_raw_INA219();
  	valueDec *= ina219_currentMultiplier_uA;
  	return valueDec;
}

/*!
 *  @brief  Gets the power value in mW, taking into account the
 *          config settings and current LSB
 *  @return power reading converted to milliwatts
 */
float getPower_uW_INA219() {
  	float valueDec = getPower_raw_INA219();
  	valueDec *= ina219_powerMultiplier_uW;
  	return valueDec;
}

