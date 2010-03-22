/* 
 * File Name: RFTestRecv.h
 *
 * Description: Sensor network configuration.
 */

#define AM_MOTE_MSG       8

configuration RFTestRecv
{
}

implementation 
{
  components Main, RFTestRecvM, GenericComm as Comm, LedsC;

  Main.StdControl             ->  RFTestRecvM;

  RFTestRecvM.CommControl     ->  Comm;
  RFTestRecvM.CommRecv        ->  Comm.ReceiveMsg[AM_MOTE_MSG];

  RFTestRecvM.Leds            ->  LedsC;
}
