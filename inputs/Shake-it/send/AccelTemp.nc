includes AccelTemp;
configuration AccelTemp 
{  
  provides 
  {
    interface StdControl;

    interface ADC as Temp;
    interface ADC as AccelX;
    interface ADC as AccelY;
	interface ADC as Photo;
  }
}

implementation
{
  components AccelTempM, ADCC;

  StdControl            =  AccelTempM;
  StdControl            =  ADCC;

  AccelX				= ADCC.ADC[TOS_ADC_ACCEL_X_PORT];
  AccelY				= ADCC.ADC[TOS_ADC_ACCEL_Y_PORT];
  Temp					= ADCC.ADC[TOS_ADC_TEMP_PORT];
  Photo					= ADCC.ADC[TOS_ADC_PHOTO_PORT];

  AccelTempM.ADCControl ->  ADCC;
}
