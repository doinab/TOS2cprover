// $Id: BSN_RadioRangeM.nc,v 1.0 2005/06/09 
//
// Testing the radio range
//
// Benny Lo
// Imperial College London


includes UbiMonMsg;
#define MAX_NO_BSN		20

module BSN_RadioRangeM
{
  provides interface StdControl;
  uses {
    interface Timer;
    interface Leds;
    interface StdControl as CommControl;
    interface SendMsg as DataMsg;
    interface ReceiveMsg as ReceiveMsg;

	interface StdControl as UARTControl;
    interface BareSendMsg as UARTSend;
    interface ReceiveMsg as UARTReceive;
  }
}
implementation
{
  TOS_Msg msg[2];
  uint8_t currentMsg;
  uint16_t count;
  uint8_t UARTConnected;
  uint16_t state;
  uint8_t sentRadio;
  uint8_t strength;
  uint8_t heartbeatfromUART;
  uint16_t parent;
  TOS_Msg replymsg,forwardmsg;

  enum {REGISTER=0,STARTED=1};

	char SendData(uint16_t data,int channel);

  // Used to initialize this component.
  command result_t StdControl.init() {
    call Leds.init();
    call Leds.yellowOff(); call Leds.redOff(); call Leds.greenOff();

    //turn on the sensors so that they can be read.
    call CommControl.init();
	call UARTControl.init();
    
    atomic {
      currentMsg = 0;
	  count=0;
	  UARTConnected=0;
	  state=REGISTER;
	  sentRadio=0;
	  strength=0;
	  heartbeatfromUART=0;
	  parent =TOS_BCAST_ADDR;
    }
    
    dbg(DBG_BOOT, "OSCOPE initialized\n");
    return SUCCESS;
  }

  //Starts the CommControl components.
  // @return Always returns SUCCESS.
  command result_t StdControl.start() {
    call UARTControl.start();
    call CommControl.start();
	call Timer.start(TIMER_REPEAT, 2234);
    return SUCCESS;
  }

  // Stops the CommControl components.
  // @return Always returns SUCCESS.
  command result_t StdControl.stop() {
    call Timer.stop();
    call CommControl.stop();
	call UARTControl.stop();
    return SUCCESS;
  }
  
  uint8_t ToRadio(uint16_t destination,TOS_MsgPtr data)
  {
		if (call DataMsg.send(destination, sizeof(struct UbiMonMsg),data))
		{
			return 1;
		}
		return 0;
  }

  // Signalled when the clock ticks.
  // @return The result of calling ADC.getData().
  event result_t Timer.fired() {
	if (state == REGISTER)
	{
		struct UbiMonMsg *pack;
		pack = (struct UbiMonMsg *) msg[currentMsg].data;    
		pack->sourceID = TOS_LOCAL_ADDRESS;
		pack->BSNcommand = 0xffff;
		pack->destination=TOS_BCAST_ADDR;
		if (sentRadio)
		{
			ToRadio(TOS_BCAST_ADDR,&msg[currentMsg]);
			call Leds.redToggle();
		}
		else {
			pack->destination=0xffff;
			msg[currentMsg].addr = TOS_UART_ADDR;
			call Leds.greenToggle();
			call UARTSend.send(&msg[currentMsg]);
		}
		sentRadio=!sentRadio;
	}
	return SUCCESS;
  }

  task void UartReply()
  {
		struct UbiMonMsg *pack;
		pack = (struct UbiMonMsg *) replymsg.data; 
		pack->sourceID = TOS_LOCAL_ADDRESS;
		replymsg.addr = TOS_UART_ADDR;
		call UARTSend.send(&replymsg);
  }

  task void RadioReply()
  {
		struct UbiMonMsg *pack;
		pack = (struct UbiMonMsg *) replymsg.data; 
		pack->sourceID = TOS_LOCAL_ADDRESS;				
		ToRadio(TOS_BCAST_ADDR,&replymsg);
		//ToRadio(parent,&replymsg);
  }

  task void ForwardRadio()
  {
		struct UbiMonMsg *pack;
		pack = (struct UbiMonMsg *) forwardmsg.data; 
		//pack->sourceID = TOS_LOCAL_ADDRESS;				
		ToRadio(forwardmsg.addr,&forwardmsg);
  }

  task void ForwardUart()
  {
		struct UbiMonMsg *pack;
		pack = (struct UbiMonMsg *) forwardmsg.data; 
		forwardmsg.addr = TOS_UART_ADDR;
		call UARTSend.send(&forwardmsg);			
  }
  task void ReplyHeartBeat()
  {
	struct UbiMonMsg *pack;
	pack = (struct UbiMonMsg *) replymsg.data; 
	pack->sourceID = TOS_LOCAL_ADDRESS;				
	pack->destination = TOS_BCAST_ADDR;
	pack->data[0] = 0;
	pack->BSNcommand = 0x81;
	if (!heartbeatfromUART)
	{
		ToRadio(TOS_BCAST_ADDR,&replymsg);
	}
	else {
		replymsg.addr = TOS_UART_ADDR;
		call UARTSend.send(&replymsg);
	}	
  }

  TOS_MsgPtr receive(TOS_MsgPtr data, bool fromUART) 
  {
	struct UbiMonMsg *pack = (struct UbiMonMsg *)data->data;    
	if (pack->destination == TOS_LOCAL_ADDRESS|| pack->destination == TOS_BCAST_ADDR)
	{	
		if (pack->BSNcommand == 0)
		{	
			if (pack->redStatus) call Leds.redOn();
			else call Leds.redOff();
			if (pack->greenStatus) call Leds.greenOn();
			else call Leds.greenOff();
			if (pack->yellowStatus) call Leds.yellowOn();
			else call Leds.yellowOff();
		} 
		else if (pack->BSNcommand == 0xfffe)
		{
			call Timer.stop();
			call Leds.redOff(); 
			call Leds.yellowOff();
			call Leds.greenOff();
			state= STARTED;
		}
		else if (pack->BSNcommand == 0x80)
		{
			if (fromUART) heartbeatfromUART = 1;
			else heartbeatfromUART=0;
			post ReplyHeartBeat();
		}
	}	
	if (pack->destination == TOS_BCAST_ADDR)
	{
		if (!fromUART)
		{
			if (pack->BSNcommand == 0x40)
			{//reset
				state= REGISTER;
				call Leds.redOff();
				call Leds.greenOff();
				call Leds.yellowOff();
				call Timer.start(TIMER_REPEAT, 2234);
			}
		}
		if (!fromUART && UARTConnected)
		{				
			forwardmsg = *data;
			pack->signalstrength = data->strength;
			if (pack->BSNcommand == 0xffff)//register message
				pack->destination = TOS_LOCAL_ADDRESS;				
			memcpy(forwardmsg.data,data->data,TOSH_DATA_LENGTH);		
			post ForwardUart();			
		}		
	}
	return data;
  }

  // Signalled when the reset message counter AM is received.
  // @return The free TOS_MsgPtr. 
  event TOS_MsgPtr ReceiveMsg.receive(TOS_MsgPtr data) 
  {  
    return receive(data, FALSE);
  }
  
  // Signalled when the previous packet has been sent.
  // @return Always returns SUCCESS.
  event result_t DataMsg.sendDone(TOS_MsgPtr sent, result_t success) {
    return SUCCESS;
  }


  TOS_MsgPtr UartReceive(TOS_MsgPtr data)
  {
	struct UbiMonMsg *pack = (struct UbiMonMsg *)data->data;    
	if (pack->destination == TOS_LOCAL_ADDRESS ||
		pack->destination == TOS_BCAST_ADDR)
	{		
		if (pack->BSNcommand == 0)
		{
			if (pack->redStatus) call Leds.redOn();
			else call Leds.redOff();
			if (pack->greenStatus) call Leds.greenOn();
			else call Leds.greenOff();
			if (pack->yellowStatus) call Leds.yellowOn();
			else call Leds.yellowOff();
		}
		else if (pack->BSNcommand == 0xfffe)
		{//registration acknowledged
			call Timer.stop();
			call Leds.redOff(); 
			call Leds.yellowOff();
			call Leds.greenOff();
			state= STARTED;
		}
		else if (pack->BSNcommand == 0x40)
		{//reset
			state= REGISTER;
			call Leds.redOff();
			call Leds.greenOff();
			call Leds.yellowOff();
			call Timer.start(TIMER_REPEAT, 2234);			
			if (pack->destination == TOS_BCAST_ADDR)
			{
				atomic {		
					forwardmsg = *data;
					memcpy(forwardmsg.data,data->data,TOSH_DATA_LENGTH);		
				}
				post ForwardRadio();
			}
		}
		else if (pack->BSNcommand == 0x80)
		{
			heartbeatfromUART =1;
			post ReplyHeartBeat();
		}
	}
	if (pack->destination != TOS_LOCAL_ADDRESS )
	{
		atomic {
			forwardmsg = *data;
			memcpy(forwardmsg.data,data	->data,TOSH_DATA_LENGTH);		
		}
		post ForwardRadio();
	}
	return data;
  }

  event TOS_MsgPtr UARTReceive.receive(TOS_MsgPtr data) {
	UARTConnected=1;
    return UartReceive(data);
  }
  
  event result_t UARTSend.sendDone(TOS_MsgPtr data, result_t success) {	
    return SUCCESS;
  }
  
}
