/* 
 * File Name: RFTestRecvM.nc
 *
 * Description: Sensor network module.
 */

module RFTestRecvM
{ 
  provides
  {
    interface StdControl;
  }

  uses 
  {
    interface StdControl  as CommControl;
    interface ReceiveMsg  as CommRecv;

    interface Leds;
  }
}

/* 
 *  Module Implementation
 */

implementation 
{
  enum
  {
    CHECK_MASK  = 0x0001,
  };

  /******************************************** 
   * Initialization for the application:
   *  Initialize communication layer and Leds
   *  Always returns SUCCESS
   **/
  command result_t StdControl.init()
  {
    // Initialize communication layer and Leds
    call CommControl.init();
    call Leds.init();

    return SUCCESS;
  }

  //** start communication layer
  command result_t StdControl.start()
  {
    call CommControl.start();

    return SUCCESS;
  }

  //** stop communication layer
  command result_t StdControl.stop()
  {
    call CommControl.stop();

    return SUCCESS;
  } 

  /******************************************** 
   * Signalled when a packet is received.
   * Return the free TOS_MsgPtr. 
   */
  event TOS_MsgPtr CommRecv.receive(TOS_MsgPtr pMsg)
  {
    uint16_t *wPtr;
    uint16_t wSequenceNum;
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	uint16_t varx,vary;	//variance
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    wPtr=(uint16_t *)(pMsg->data);
    wSequenceNum=*wPtr;
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	varx=wPtr[1];
	vary=wPtr[2];
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
	
    //if( (wSequenceNum & CHECK_MASK) == CHECK_MASK )
     // call Leds.redToggle();
	//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    return pMsg;
  }
} // end of implementation
