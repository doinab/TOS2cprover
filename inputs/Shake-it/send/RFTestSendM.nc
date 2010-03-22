/* 
 * File Name: RFTestSendM.nc
 *
 * Description: Sensor network module.
 */

module RFTestSendM
{ 


  provides
  {
    interface StdControl;
  }

  uses 
  {
    interface StdControl  as CommControl;
    interface SendMsg     as CommSend;

    interface Timer       as Timer;
    interface Leds;
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	interface StdControl    as ADCControl;
    interface ADC           as AccelX;
    interface ADC           as AccelY;
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  }
}

/* 
 *  Module Implementation
 */
implementation 
{
  enum
  {
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    //PACKET_INTERVAL  = 100,  // ms
    PACKET_INTERVAL  = 50,  // ms
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  };
  
  // module scoped variables
  uint16_t   wSequenceNum;
  bool       fPending;

  TOS_MsgPtr pBuffer;
  TOS_Msg    Buffer;

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  uint16_t	accelxbuffer[16],accelybuffer[16];//buffer to store the accelerometer readings
  uint16_t  count;
  uint8_t   enoughsamples;

  /************************************************************** 
   * Processing the accelerometer data to determine the variance
   * and which is used to estimate the amount of movement 
   **************************************************************/
   task void ProcessData()
   {
		uint16_t meanx,meany,varx,vary;
		uint16_t i;
		uint16_t *wPtr;
		if (enoughsamples==0) return;//make sure the buffer is filled
		atomic{
				meanx=0;meany=0;//initialise the mean and variance
				varx=0;vary=0;
				//calculate the mean
				for (i=0;i<16;i++)
				{
					meanx+=(accelxbuffer[i]>>4);
					meany+=(accelybuffer[i]>>4);
				}
				meanx>>=4;
				meany>>=4;
				//calculate the variance
				for (i=0;i<16;i++)
				{
					varx+=(((accelxbuffer[i]>>4)-meanx)*((accelxbuffer[i]>>4)-meanx));
					vary+=(((accelybuffer[i]>>4)-meany)*((accelybuffer[i]>>4)-meany));
				}
				varx>>=4;
				vary>>=4;
				//Thresholding to estimate the amount of movement
				if (varx>50||vary>50)			//small movement
					call Leds.redOn();
				else call Leds.redOff();
				if (varx>800 || vary>800)		//moderate movement
					call Leds.greenOn();
				else call Leds.greenOff();
				if (varx>1800||vary>1800)		//intense movement
					call Leds.yellowOn();
				else call Leds.yellowOff();
			}
			if(!fPending)
			{
				atomic{
					wPtr=(uint16_t *)(pBuffer->data);
					wPtr[0]=wSequenceNum++;
					wPtr[1]=varx;
					wPtr[2]=vary;
					fPending = TRUE;
				}
					if(call CommSend.send(TOS_BCAST_ADDR, 6, pBuffer))
							return;
				atomic	fPending = FALSE;
			}

	}
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  /******************************************** 
   * Initialization for the application:
   *  1. Initialize communication layer and Leds
   *  2. Initialize module static variables
   *  Always returns SUCCESS
   **/
  command result_t StdControl.init()
  {
		uint16_t i;
		atomic {
			//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			//initialise the buffers
  			for (i=0;i<16;i++)
			{
				accelxbuffer[i]=0;
				accelybuffer[i]=0;
			}
			count=0;
			enoughsamples=0;
			//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
			// Initialize module static variables
			wSequenceNum=0;
			fPending    = FALSE;
			pBuffer     =&Buffer;
		}
    // Initialize communication layer and Leds
    call CommControl.init();
	call Leds.init();
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	call ADCControl.init();				//initialise the ADC
    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    


	
    return SUCCESS;
  }

  //** start communication layer and Timer
  command result_t StdControl.start()
  {
    call CommControl.start();
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    call ADCControl.start();			//start the ADC
    TOSH_MAKE_HUM_PWR_OUTPUT();			//enable the sensors
    TOSH_SET_HUM_PWR_PIN();
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    call Timer.start(TIMER_REPEAT, PACKET_INTERVAL);

    return SUCCESS;
  }

  //** stop communication layer and Timer
  command result_t StdControl.stop()
  {
    call CommControl.stop();
    call Timer.stop();
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	call ADCControl.stop();				//stop the ADC
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    return SUCCESS;
  } 

  /******************************************** 
   * Signalled when the clock ticks.
   * Toggle the red LED and try to send out a packet and .
   * Returns SUCCESS or FAIL
   */
  event result_t Timer.fired()
  {
    //uint16_t *wPtr;
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	call AccelX.getData();				//signal to get the accelerometer X value
	
    /*****
	 if(!fPending)
    {
      //call Leds.redToggle();

      wPtr=(uint16_t *)(pBuffer->data);
      *wPtr=wSequenceNum++;

      fPending = TRUE;
      if(call CommSend.send(TOS_BCAST_ADDR, 2, pBuffer))
        return SUCCESS;

      fPending = FALSE;
    }*****/
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    return FAIL;
  }

  /******************************************** 
   * Signalled when the previous packet has been sent.
   * Always returns SUCCESS.
   */
  event result_t CommSend.sendDone(TOS_MsgPtr pMsg, result_t success)
  {
    if(fPending && pMsg==pBuffer)
      fPending=FALSE;

    return SUCCESS;
  }

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  async event result_t AccelX.dataReady(uint16_t this_data)
  {
    atomic  accelxbuffer[count]=this_data; //store the data into the buffer

    call AccelY.getData();				   //signal to get the accelerometer Y value

    return SUCCESS;
  }

  async event result_t AccelY.dataReady(uint16_t this_data)
  {
    atomic
    {
		accelybuffer[count++]=this_data;  //store the data into the buffer
		if (count >=16)	//reset the counter -> round buffer
		{
				count=0;
				enoughsamples=1;
		}
    }
	post ProcessData();	//initiate the processing of the sensor data 

    return SUCCESS;
  }
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

} // end of implementation
