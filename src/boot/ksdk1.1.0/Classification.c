
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
	
    	if (i2cReadStatus != kWarpStatusOK){
    	warpPrint("\nFailed to read acceleration measurements.\n");
    	return;
}
}

// Function for converting acceleration from multiples of 1/1024 g in 8g mode set, to mms-2
int32_t realAcceleration(int16_t accelg){
int32_t accel = ((int32_t)(accelg) * 9810) / 1024;
return accel;
}

void BufferShift(){

}

void StepRate(){

}

void MaximalAxis(){

}

void LPFdata(){
	int i;
	for(i=0; i<BUFFER_SIZE; i++){
	LPFBuffer[BUFFER_SIZE - 1] += (AccelerationBuffer[i] * (uint32_t)LPFCoeffs[i]);
	}
	warpPrint("AccelerationBuffer[%d] = %d, LPFBuffer[%d] = %d.\n", BUFFER_SIZE - 1, AccelerationBuffer[BUFFER_SIZE - 1], BUFFER_SIZE - 1, LPFBuffer[BUFFER_SIZE - 1]);
}

//Need to search for points of inflection for pedometer algorithm, about these points the curvature changes.
void Diffdata(){
	maxminpoints = 0;
	clas
}

void Activity(){

}
