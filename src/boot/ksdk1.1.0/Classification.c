
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

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

#include "devMMA8451Q.h"
#include "Classification.h"

void classificationAlg(){
	uint16_t XMSB, YMSB, ZMSB;
  	uint16_t XLSB, YLSB, ZLSB;
	int32_t XAccel, YAccel, ZAccel;
	uint32_t accelerationSquare;
	WarpStatus i2cReadStatus;
		
    	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6 /* numberOfBytes */);
	
	// warpPrint("\nReading acceleration measurements from MMA8451Q registers %d to %d.\n", kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, kWarpSensorOutputRegisterMMA8451QOUT_X_MSB + 5);

	if (i2cReadStatus != kWarpStatusOK){
    	warpPrint("\nFailed to read acceleration measurements.\n");
    	return;
}

	XMSB = deviceMMA8451QState.i2cBuffer[0];
  	XLSB = deviceMMA8451QState.i2cBuffer[1];
  	XCombined = ((XMSB & 0xFF) << 6) | (XLSB >> 2);
	//Sign extend 14-bit data
	XCombined = (XCombined ^ (1 << 13)) - (1 << 13);
	XAccel = realAcceleration(XCombined);

	YMSB = deviceMMA8451QState.i2cBuffer[2];
  	YLSB = deviceMMA8451QState.i2cBuffer[3];
  	YCombined = ((YMSB & 0xFF) << 6) | (YLSB >> 2);
  	//Sign extend 14-bit data
	YCombined = (YCombined ^ (1 << 13)) - (1 << 13);
	YAccel = realAcceleration(YCombined);
		
	ZMSB = deviceMMA8451QState.i2cBuffer[4];
  	ZLSB = deviceMMA8451QState.i2cBuffer[5];
  	ZCombined = ((ZMSB & 0xFF) << 6) | (ZLSB >> 2);
	//Sign extebd to 14-bit data
  	ZCombined = (ZCombined ^ (1 << 13)) - (1 << 13);
	ZAccel = realAcceleration(ZCombined);

	accelerationSquare = sqrt((uint32_t)(XAccel*XAccel) + (uint32_t)(YAccel*YAccel) + (uint32_t)(ZAccel*ZAccel));

	BufferShift();

	AccelerationSquareBuffer[BUFFER_SIZE -1] = accelerationSquare;
	
	//Implement LPF
	LPFdata();
	
	//Then need to run preceding steps of the algorithm repetitively until the BUFFER is filled so that inflection points can be determined with greater confidence for determining step count.
	
	count++;

	if(count % BUFFER_SIZE != 0){
		warpPrint("\n %d, %d, %d, %d, %d", XAccel, YAccel, ZAccel, accelerationSquare, LPFA2Buffer[BUFFER_SIZE-1]);
	}
	else{
		calculateSecondDerivative(LPFA2Buffer, BUFFER_SIZE, ddAccelerationSquareBuffer);
		InflectionPoints = findInflectionPoints(ddAccelerationSquareBuffer, BUFFER_SIZE-2);
	warpPrint("\n %d, %d, %d, %d, %d", XAccel, YAccel, ZAccel, accelerationSquare, LPFA2Buffer[BUFFER_SIZE-1], ddAccelerationSquareBuffer[BUFFER_SIZE-3], ddAccelerationSquareBuffer[BUFFER_SIZE - 4], InflectionPoints);	
	}
}

// Function for converting acceleration from multiples of 1/1024 g in 8g mode set, to mms-2
int32_t realAcceleration(int16_t accelg){
int32_t accel = ((int32_t)(accelg) * 9810) / 1024;
return accel;
}

//Function for shifting the acceleration and lpf buffer data left for new data entry.
void BufferShift(int32_t* Buffer){
	for (int i=1; i < BUFFER_SIZE; i++){
 	AccelerationSquareBuffer[i-1] = AccelerationSquareBuffer[i];
        LPFA2Buffer[i-1] = LPFA2Buffer[i];
        }
        AccelerationSquareBuffer[BUFFER_SIZE -1] = 0;
        LPFA2Buffer[BUFFER_SIZE - 1] = 0;
        }	

void LPFdata(){
	for (int i = 0; i < BUFFER_SIZE; i++){
	LPFA2Buffer[BUFFER_SIZE-1] += AccelerationSquareBuffer[i];
	}
	//Simple moving average filter
	LPFA2Buffer[BUFFER_SIZE-1] = LPFA2Buffer[BUFFER_SIZE-1]/BUFFER_SIZE;
}

uint32_t sqrt(uint32_t num) {
    uint32_t low = 0, high = num, mid;
    while (low <= high) {
        mid = low + (high - low) / 2;
        uint64_t square = (uint64_t)mid * mid;

        if (square == num) {
            return mid;
        } else if (square < num) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }
    return high; // Return the floor value of the square root
}


// Function to calculate the second derivative
int32_t calculateSecondDerivative(uint32_t data[], int size, int32_t secondDerivative[]) {
    for (int i = 1; i < size - 1; i++) {
        secondDerivative[i - 1] = data[i - 1] - 2 * data[i] + data[i + 1];
	uint32_t secondDMag = sqrt((uint32_t)(secondDerivative[i - 1]*secondDerivative[i - 1]));
	if(secondDMag < 10){
	secondDerivative[i - 1] = 0; //attempt thresholding
	}
	//should be divide by sample period squared, but only interested in zerocrossings.
	//Note second derivative is two data points smaller than data as expected.
    }
    return 0;
}


// Function to find the number of inflection points
int findInflectionPoints(int32_t secondDerivative[], int size) {
    int count = 0;
    for (int i = 1; i < size - 1; i++) {
        if (secondDerivative[i - 1] * secondDerivative[i] < -100) {
            count++; //thresholding based on value of second derivative
        }
    }
    return count;
}

/*void StepRateIdentify(){
	
	TotalSteps += InflectionPoints/2;
	
	StepRate = TotalSteps * 100 ; //10 second interval of data collection (in mSteps/s for comparisons as integer)
	
	warpPrint("Step-rate is %d\n", StepRate);
	
	//A 6 foot male step length is around 0.75m
	//Stationary below 0.2ms-1 (step rate 0.267 steps/s) 
	//Walking Speed is upto 2 ms-1 (step rate 2.667 steps/s)
	//Jogging Speed is upto 5 ms-1 (step rate 6.667 steps/s)
	//Sprinting is above 5 ms-1 (with absolute certainity at around 7ms-1/9.333 steps/s even for a highly trained sprinter).
	
	if(StepRate < StationaryRate){
        ActivityDetect = StationaryLabel;
	Uncertainty = 0;
        warpPrint("Activity = Stationary, \t\t Confidence Level = %d Percent.\n", 100 - Uncertainty);
	}
        else if(WalkingRate > StepRate > StationaryRate){
        ActivityDetect = WalkingLabel;
	Uncertainty = 100 * (WalkingRate - StepRate) / (WalkingRate - StationaryRate);
	warpPrint("Activity = Walking, \t\t Confidence Level = %d Percent.\n", 100 - Uncertainty);
        }
        else if(JoggingRate > StepRate > WalkingRate){
        ActivityDetect = JoggingLabel;
	Uncertainty = 100 * (JoggingRate - StepRate) / (JoggingRate - WalkingRate);
	warpPrint("Activity = Jogging, \t\t Confidence Level = %d Percent.\n", 100 - Uncertainty);
        }
	else{
	ActivityDetect = SprintingLabel;
	if(StepRate > ThresholdSprint){
		Uncertainity = 0;
		warpPrint("Activity = Sprinting, \t\t Confidence Level = %d Percent.\n", 100 - Uncertainty);
	}
	else{
		Uncertainty = 100 * (ThresholdSprint - StepRate) / (ThresholdSprint - SprintingRate);
                warpPrint("Activity = Sprinting, \t\t Confidence Level = %d Percent.\n", 100 - Uncertainty);
	}
	}
		
}*/
