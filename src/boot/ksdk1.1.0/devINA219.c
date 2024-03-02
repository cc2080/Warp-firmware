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

