/* 
 * File Name: RFTestSend.h
 *
 * Description: Sensor network configuration.
 */

#define AM_MOTE_MSG       8

configuration RFTestSend
{
}

implementation 
{
  components Main, RFTestSendM, GenericComm as Comm, TimerC, LedsC, 
  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
		AccelTemp; //the module for accelerometer and temperature sensor
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  Main.StdControl             ->  RFTestSendM;

  RFTestSendM.CommControl     ->  Comm;
  RFTestSendM.CommSend        ->  Comm.SendMsg[AM_MOTE_MSG];

  RFTestSendM.Timer           -> TimerC.Timer[unique("Timer")];
  RFTestSendM.Leds            ->  LedsC;

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  RFTestSendM.ADCControl      ->  AccelTemp.StdControl;
  RFTestSendM.AccelX          ->  AccelTemp.AccelX;
  RFTestSendM.AccelY          ->  AccelTemp.AccelY;
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  
}
