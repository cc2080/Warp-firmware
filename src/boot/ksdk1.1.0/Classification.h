// Output data rate of the MMA8451Q is [look on datasheet]
#pragma once

// Declare labels with enum type, so that ActivityDetect can be called instead of the individual label.
typedef enum
{
	StationaryLabel		= 0x0,
	WalkingLabel		= 0x1,
	JoggingLabel		= 0x2,
	SprintingLabel		= 0x3,
} ActivityDetect;

WarpStatus UpdateAccelerations();

void ClassificationAlg();
void BufferShift();
void StepRate();
void MaximalAxis();
void LPFdata();
void Diffdata();
void Activity();

// Concatenated acceleratiun MSB and LSB data
int16_t XCombined - 0;
int16_t YCombined - 0;
int16_t ZCombined - 0;

// Acceleration and LPF buffer intialisation
uint32_t AccelerationBuffer[BUFFER_SIZE] ={0}
uint32_t LPFBuffer[BUFFER_SIZE] ={0}

uint16_t maxminpoints = 0;

int32_t realAcceleration(int16_t accelg);
