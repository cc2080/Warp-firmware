
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
	uint16_t XLSB, YLSB, ZLSB;
	uint16_t XMSB, YMSB, ZMSB;
	int32_t XAccel, YAccel, ZAccel;
	WarpStatus i2cReadStatus;
	
    	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6 /* numberOfBytes */);
	
	warpPrint("\nReading acceleration measurements from MMA8451Q registers %d to %d.\n", kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, kWarpSensorOutputRegisterMMA8451QOUT_X_MSB + 5);

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
	
	AccelSum = XAccel + YAccel + ZAccel;

	BufferShift();
	
	AccelerationBuffer[BUFFER_SIZE -1] = AccelSum;

	// X-acc, Y-acc, Z-acc, AccSum
	warpPrint("%d, %d, %d, %d", XAccel, YAccel, ZAccel, AccelerationBuffer[BUFFER_SIZE - 1]);

	//Implement LPF
	
	//Implement Differentiation

	//Calculate Step-rate
	
	//Identify Activity
	
	
}

// Function for converting acceleration from multiples of 1/1024 g in 8g mode set, to mms-2
int32_t realAcceleration(int16_t accelg){
int32_t accel = ((int32_t)(accelg) * 9810) / 1024;
return accel;
}

//Function for shifting the acceleration and lpf buffer data left for new data entry.
void BufferShift(){
	for (int i=1; i < BUFFER_SIZE; i++){
	AccelerationBuffer[i-1]=AccelerationBuffer[i];
	LPFBuffer[i-1] = LPFBuffer[i];
	}
	
      	AccelerationBuffer[BUFFER_SIZE - 1] = 0;
  	LPFBuffer[BUFFER_SIZE - 1] = 0;
}

void StepRate(){

}

void MaximalAxis(){

}

//void LPFdata(){
//	int i;
//	for(i=0; i<BUFFER_SIZE; i++){
//	LPFBuffer[BUFFER_SIZE - 1] += (AccelerationBuffer[i] * (uint32_t)LPFCoeffs[i]);
//	}
//	warpPrint("AccelerationBuffer[%d] = %d, LPFBuffer[%d] = %d.\n", BUFFER_SIZE - 1, AccelerationBuffer[BUFFER_SIZE - 1], BUFFER_SIZE - 1, LPFBuffer[BUFFER_SIZE - 1]);
//}

//Need to search for points of inflection for pedometer algorithm, about these points the curvature changes.
//void Diffdata(){
//	maxminpoints = 0;
//
//	if((LastEntry > PenultimateEntry) && (lastElement > LPFBuffer[0]) && (minimumSamples > 10)){ // A concave inflection point (maximum) has been reached.
//    numberOfInflectionPoints = numberOfInflectionPoints + 1;
 //   minimumSamples = 0;
    // warpPrint("simpleDiff(): %d > %d and %d > %d - MAXIMUM detected in LPFBuffer[-1].\n", lastElement, secondToLastElement, lastElement, LPFBuffer[0]);
//    if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
 //     firstExcessTest = 0;
  //    firstExcessTime = (numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE);
      // warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
//    }
//    finalInflectionTime = (numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE);
    // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
//  }
 // else if((lastElement < secondToLastElement) && (lastElement < LPFBuffer[0]) && (minimumSamples > 10)){ // A convex inflection point (minimum) has been reached.
//    numberOfInflectionPoints = numberOfInflectionPoints + 1;
//   minimumSamples = 0;
//    warpPrint("simpleDiff(): %d < %d and %d < %d - MINIMUM detected in LPFBuffer[-1].\n", lastElement, secondToLastElement, lastElement, LPFBuffer[0]);
//    if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
//      firstExcessTest = 0;
//      firstExcessTime = (numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE);
      // warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
//    }
//    finalInflectionTime = (numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE);
    // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
//  }
//  minimumSamples++;

  // Secondly, check if the first element of the current LPFBuffer is an inflection point.
//  if((LPFBuffer[0] > lastElement) && (LPFBuffer[0] > LPFBuffer[1])  && (minimumSamples > 10)){ // A concave inflection point (maximum) has been reached.
//    numberOfInflectionPoints = numberOfInflectionPoints + 1;
//    minimumSamples = 0;
//    warpPrint("simpleDiff(): %d > %d and %d > %d - MAXIMUM detected in LPFBuffer[0].\n", LPFBuffer[0], lastElement, LPFBuffer[0], LPFBuffer[1]);
//    if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
//      firstExcessTest = 0;
//      firstExcessTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + SAMPLE_PERIOD;
      // warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
//    }
//    finalInflectionTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + SAMPLE_PERIOD;
    // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
// }
//  else if((LPFBuffer[0] < lastElement) && (LPFBuffer[0] < LPFBuffer[1])  && (minimumSamples > 10)){ // A convex inflection point (minimum) has been reached.
//    numberOfInflectionPoints = numberOfInflectionPoints + 1;
//    minimumSamples = 0;
//    warpPrint("simpleDiff(): %d < %d and %d < %d - MINIMUM detected in LPFBuffer[0].\n", LPFBuffer[0], lastElement, LPFBuffer[0], LPFBuffer[1]);
//    if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
//      firstExcessTest = 0;
//      firstExcessTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + SAMPLE_PERIOD;
//      warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
//    }
//    finalInflectionTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + SAMPLE_PERIOD;
    // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
//  }
//  minimumSamples++;

  // Thirdly, check the middle 37 points of the buffer in a for loop.
//  for(int i = 1; i < BUFFER_SIZE - 1; i++){
//    if((LPFBuffer[i] > LPFBuffer[i-1]) && (LPFBuffer[i] > LPFBuffer[i+1])  && (minimumSamples > 10)){ // A concave inflection point (maximum) has been reached.
//      numberOfInflectionPoints = numberOfInflectionPoints + 1;
//      minimumSamples = 0;
//      warpPrint("simpleDiff(): %d > %d and %d > %d - MAXIMUM detected in LPFBuffer[%d].\n", LPFBuffer[i], LPFBuffer[i-1], LPFBuffer[i], LPFBuffer[i+1], i);
//      if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
//        firstExcessTest = 0;
//	firstExcessTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + (SAMPLE_PERIOD * (i + 1));
	// warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
//      }
//      finalInflectionTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + (SAMPLE_PERIOD * (i + 1));
      // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
//    }
//    else if((LPFBuffer[i] < LPFBuffer[i-1]) && (LPFBuffer[i] < LPFBuffer[i+1]) && (minimumSamples > 10)){ // A convex inflection point (minimum) has been reached.
//      numberOfInflectionPoints = numberOfInflectionPoints + 1;
//      minimumSamples = 0;
//      warpPrint("simpleDiff(): %d < %d and %d < %d - MINIMUM detected in LPFBuffer[%d].\n", LPFBuffer[i], LPFBuffer[i-1], LPFBuffer[i], LPFBuffer[i+1], i);
//      if(firstExcessTest){ // Runs if this is the first inflection point of the experiment.
//        firstExcessTest = 0;
//	firstExcessTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + (SAMPLE_PERIOD * (i + 1));
	// warpPrint("simpleDiff(): firstExcessTime: %d.\n", firstExcessTime);
//      }
//      finalInflectionTime = ((numberOfCycles - 1) * (SAMPLE_PERIOD * BUFFER_SIZE)) + (SAMPLE_PERIOD * (i + 1));
      // warpPrint("simpleDiff(): finalInflectionTime: %d.\n", finalInflectionTime);
//    }
//    minimumSamples++;
//  }
  // Save the last and second-to-last elements in the range - this is required to avoid missing inflection points which occur in the 0th and 38th elements of the buffer.
//  lastElement = LPFBuffer[BUFFER_SIZE - 1];
//  secondToLastElement = LPFBuffer[BUFFER_SIZE - 2];
//}

//void Activity(){
//	if(StepRate > SprintingRate){
//	ActivityDetect = SprintingLabel;
//	}
	
//	else if(StepRate > JoggingRate{
//	ActivityDetect = JoggingLabel;
//	}
	
//	else if(StepRate > WalkingRate{
//	ActivityDetect = WalkingLabel;
//	}
	
//	else{
//	ActivityDetect = StationaryLabel;
//	}

//}
