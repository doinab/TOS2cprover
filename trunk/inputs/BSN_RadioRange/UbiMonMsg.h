/*									UbiMon protocol
 * Authors:		Benny Lo
 * Created:			Dec 4, 2003
 * Last modified:	Dec 4, 2003
 *
 */

/* Packets used by Oscope. */
enum {
  BUFFER_SIZE = 30
};

struct UbiMonMsg
{
/*    uint16_t sourceMoteID;
	uint16_t lastSampleNumber;
    uint16_t channel;
    uint16_t data[BUFFER_SIZE];*/
	uint16_t sourceID;
	uint16_t destination;
	uint16_t BSNcommand;
	uint8_t signalstrength;
	uint8_t	 redStatus;
	uint8_t  greenStatus;
	uint8_t  yellowStatus;
	uint16_t data[28];
};

struct UbiMonResetMsg
{
    /* Empty payload! */
};

enum {
  AM_OSCOPEMSG = 10,/*packet size sent*/
  AM_OSCOPERESETMSG = 10/* packet size received*/
};
