// Accelerometer and temperature sensor header file
// 
// Benny Lo
// Imperial College London

#ifndef ACCEL_TEMP_H 
#define ACCEL_TEMP_H

#include "MSP430ADC12.h"

//Temperature sensor set to use ADC1
enum
{
  TOS_ADC_TEMP_PORT = unique("ADCPort"),
  TOSH_ACTUAL_ADC_TEMP_PORT=ASSOCIATE_ADC_CHANNEL(
		INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss,REFVOLT_LEVEL_2_5)			  
};

//Photo sensor set to use ADC0
enum
{
  TOS_ADC_PHOTO_PORT = unique("ADCPort"),
  TOSH_ACTUAL_ADC_PHOTO_PORT=ASSOCIATE_ADC_CHANNEL(
		INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss,REFVOLT_LEVEL_2_5)			  
};

//Accelerometer X axis set to use ADC2
enum {
	TOS_ADC_ACCEL_X_PORT = unique("ADCPort"),
	TOSH_ACTUAL_ADC_ACCEL_X_PORT = ASSOCIATE_ADC_CHANNEL( 
		   INPUT_CHANNEL_A2, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5)
};

//Accelerometer Y axis set to use ADC3
enum{
	TOS_ADC_ACCEL_Y_PORT = unique("ADCPort"),
    TOSH_ACTUAL_ADC_ACCEL_Y_PORT = ASSOCIATE_ADC_CHANNEL( 
		   INPUT_CHANNEL_A3, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5)
};

//----------------------
// Setting for ADCSingle and ADCMultiple

//Temperature sensor set to use ADC1
#define MSP430ADC12_TEMP ADC12_SETTINGS( \
           INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss, SAMPLE_HOLD_4_CYCLES, \
           SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPCON_SOURCE_SMCLK, \
           SAMPCON_CLOCK_DIV_1, REFVOLT_LEVEL_2_5) 

//Accelerometer X axis set to use ADC2
#define MSP430ADC12_ACCELX ADC12_SETTINGS( \
           INPUT_CHANNEL_A2, REFERENCE_VREFplus_AVss, SAMPLE_HOLD_4_CYCLES, \
           SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPCON_SOURCE_SMCLK, \
           SAMPCON_CLOCK_DIV_1, REFVOLT_LEVEL_2_5) 

//Accelerometer Y axis set to use ADC3
#define MSP430ADC12_ACCELY ADC12_SETTINGS( \
           INPUT_CHANNEL_A3, REFERENCE_VREFplus_AVss, SAMPLE_HOLD_4_CYCLES, \
           SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPCON_SOURCE_SMCLK, \
           SAMPCON_CLOCK_DIV_1, REFVOLT_LEVEL_2_5) 

//Photo sensor set to use ADC0
#define MSP430ADC12_PHOTO ADC12_SETTINGS( \
           INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss, SAMPLE_HOLD_4_CYCLES, \
           SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPCON_SOURCE_SMCLK, \
           SAMPCON_CLOCK_DIV_1, REFVOLT_LEVEL_2_5) 

#endif

