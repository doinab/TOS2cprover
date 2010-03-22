// $Id: BSN_RadioRange.nc,v 1.0 2005/09/06 
//
//	Testing the radio range
//
// Benny Lo
// Imperial College London


includes UbiMonMsg;


configuration BSN_RadioRange { }
implementation
{
  components Main, BSN_RadioRangeM
           , TimerC
           , LedsC           
           , GenericComm as Comm
		   , UARTNoCRCPacket;

  Main.StdControl -> BSN_RadioRangeM;
  Main.StdControl -> TimerC;
  
  BSN_RadioRangeM.Timer -> TimerC.Timer[unique("Timer")];
  BSN_RadioRangeM.Leds -> LedsC;

  BSN_RadioRangeM.CommControl -> Comm;
  BSN_RadioRangeM.ReceiveMsg -> Comm.ReceiveMsg[AM_OSCOPEMSG];
  BSN_RadioRangeM.DataMsg -> Comm.SendMsg[AM_OSCOPEMSG];

  BSN_RadioRangeM.UARTControl -> UARTNoCRCPacket;
  BSN_RadioRangeM.UARTSend -> UARTNoCRCPacket;
  BSN_RadioRangeM.UARTReceive -> UARTNoCRCPacket;

}
