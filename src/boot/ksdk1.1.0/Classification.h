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

#define BUFFER_SIZE 15   //Arbitrarly chosen to provide good fitlering whilst being memory efficient.
#define SAMPLE_PERIOD 100  // 100ms time between sensor outputs.
#define StationaryRate 267
#define WalkingRate 2667
#define JoggingRate 6667
#define ThresholdSprint 9333

void classificationAlg();
void BufferShift();
void LPFdata();
void StepRateIdentify();
int32_t calculateSecondDerivative(uint32_t data[], int size, int32_t secondDerivative[]);
int findInflectionPoints(int32_t secondDerivative[], int size);
// Concatenated acceleratiun MSB and LSB data
int16_t XCombined = 0;
int16_t YCombined = 0;
int16_t ZCombined = 0;
uint8_t InflectionPoints = 0;
uint8_t TotalSteps = 0;
uint32_t StepRate = 0;
uint16_t Uncertainty;

// Acceleration and LPF buffer intialisation
uint32_t AccelerationSquareBuffer[BUFFER_SIZE] = {0};
uint32_t LPFA2Buffer[BUFFER_SIZE] = {0};
int32_t ddAccelerationSquareBuffer[BUFFER_SIZE-2] ={0};

//uint16_t maxminpoints = 0;

int32_t realAcceleration(int16_t accelg);
uint32_t sqrt(uint32_t num);

uint16_t timeBefore = 0;
uint16_t timeAfter = 0;
uint16_t timeDifference = 0;
uint16_t count = 0;
bool AcclerometerFill = 0; //Must ensure the Accelerometer buffer is filled before processing.
