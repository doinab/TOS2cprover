includes AccelTemp;
module AccelTempM {
  provides interface StdControl;
  uses interface ADCControl;
}

implementation {

  command result_t StdControl.init() 
  {
	  call ADCControl.setSamplingRate(TOS_ADCSample3750ns);//set the ADC sampling rate
	  call ADCControl.bindPort(TOS_ADC_ACCEL_X_PORT,TOSH_ACTUAL_ADC_ACCEL_X_PORT);
	  call ADCControl.bindPort(TOS_ADC_ACCEL_Y_PORT,TOSH_ACTUAL_ADC_ACCEL_Y_PORT);
	  call ADCControl.bindPort(TOS_ADC_TEMP_PORT,TOSH_ACTUAL_ADC_TEMP_PORT);	
	  call ADCControl.bindPort(TOS_ADC_PHOTO_PORT,TOSH_ACTUAL_ADC_PHOTO_PORT);	
    
	  return call ADCControl.init();
  }

  command result_t StdControl.start() 
  {
    return SUCCESS;
  }

  command result_t StdControl.stop() 
  {
      return SUCCESS;
  }
} // end of implementation
