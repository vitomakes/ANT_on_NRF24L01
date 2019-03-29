 /** 
 *
 * @author Vitomakes
 * $Date: 19.06.14$
 * $Revision: 1.01$
 * ----------------------------------------------------------------------------------------
 *                              ANT packet description
 *
 * (bytes from first transmitted to last transmitted):
 *
 * [ ANT ][ANT chann. ID][pt][       payload         ][ CRC ]
 * [5b 25][dd DD][dt][tt][pt][p0 p1 p2 p3 p4 p5 p6 p7][cs cs]
 *  
 * 	5b 25 = ANT message preamble 
 *		 dd = device number LSB(*)
 *		 DD = device number MSB(*)
 *		 dt = device type(*)
 *     tt = transmission type (*)
 *		 pt = packet type
 *		 p0..7 = payload bytes (LSB to MSB)
 *		 crc = "CRC-16/CCITT-FALSE" poly=0x1021 init=0xFFFF (calculated on whole packet)
 *
 *    (*) these parameters must be set as in the ANT device [5.2 D00000652_ANT_Message_Protocol_and_Usage_Rev_5.0.pdf]
 *
 *		 pt packet type:
 *
 *		 	[broadcast]
 *			0a = broadcast message (no ack)
 *			 
 *		 	[acknowledged]
 *      aa = acknowledged message
 *      f2 = ACK to acknowledged message
 *
 *		 	[burst mode]
 *      8a = start burst transfer
 *      d2 = ACK to start burst transfer
 *					
 *      92 = burst packet 0
 *      c2 = ACK to burst packet 0
 *					
 *      82 = burst packet 1
 *      d2 = ACK to burst packet 1
 *					
 *      b2 = last burst packet 0 (preceeding message was packet 1) 
 *      e2 = ACK to last burst packet 0 (preceeding message was packet 1)
 *					
 *      a2 = last burst packet 1 (preceeding message was packet 0) 
 *      f2 = ACK to last burst packet 1 (preceeding message was packet 0)
 *
 *      example of burst sequence (ACK not shown):
 *			8a						start
 *			92						packet 0
 * 			82						packet 1
 *			92						packet 0
 *			82 --> b2			packet 1	--> ending packet type 0
 *			92 --> a2			packet 0	--> ending packet type 1
 *
 *			[unrequested packets]  in direction nRF --> ANT, only in answer to a broadcast packet
 *      02 = unrequested broadcast packet
 *			
 *			a2 = unrequested acknowledged packet
 *			f2 = ACK to unrequested acknowledged packet
 *
 *			82 = unrequested burst transmission start
 *
 *			
 */
 
/** ***V1.01
*
* TIMING/POWER MANAGMENT
*	x = CE ON TX
*	o = CE ON RX
*	- = CE OFF
*	
*	Time-->
*	
*	Example:
*	
*	ANT_period is calculated starting from the message rate value. It is the interval between two broadcast messages on the channel
*	nRF_ANT switches on radio GUARD_TIME microseconds before the end o ANT_period. This is done in order to allow the wake-up time of nRF,
* the elaboration of the packet and allow some degree of period variation.
*	After CE is switched on, it is kept on when RX_WINDOW_TIME is over, or the packet is received, whichever comes first.
* The next ANT_period is waited from the interrupt caused by the incoming packet-
*																				 														 	
*		                                                                         	
*	    |<------------------------ANT_period--------------------------------------->|
*	    |<----------------------ANT_period_Time----------------->|<---------------RX_WINDOW_TIME------------>
*	                                                             |<---GUARD_TIME--->|
*                                                                  |<------------------ANT_pediod----
* nRF	|o-------------------------------------------------------|ooo|o-------------|
*    RX1 	    	                                                  RX2          expected          
*
*	Only one timer is used for all time calculations. In case any other event / transmission happens between two broadcast message receives, the delays are
* accumulated in ANT_DelayAccu, and removed from ANT_period.
* Here is an example of an unrequested BROADCAST packet sent from nRF to ANT:
*	
*	
*		  |<------------------------ANT_period--------------------------------------->|               |<---ANT_period---
*		  |<--ANT_DelayAccu-->|<---------ANT_period_Timer--------->|<---------------RX_WINDOW_TIME------------>
*                                                              |<---GUARD_TIME--->|
*	
*	nRF |o----------xxxxxxxx|------------------------------------|oooooooooooooooooo|ooooooooooooooo|o------
*    RX0 	     	  TX (unrequested broadcast)                  expected           RX1
*                                                                            	 packet
*		 
*/

//  Global defines & includes
#include <cygnal\c8051F320.h>			// uC specific register mapping
#include "include\nRF_ANT.h"
#include "include\LL_API.h" 			//(include CE PIN function)
#include "include\nRF_API.h" 			//(include SPI function)
#include "include\usb.h"					//(used to write debug info via USB)


// ANT Preamble
//***do not change***
#define	ANT_Preamble_HI			0x25
#define ANT_Preamble_LO			0x5b

// size of buffer for saving burst RX data
#define MAX_BUFFER					0x08

// masks for reading interrupt values from nRF
#define MASK_IRQ_FLAGS      0x70
#define RX_DR           		0x40  // RX data received

//***V1.01
// Timer managment defines. See comments near definition of void Start_Timer(BYTE mode)

//***V1.01
// GUARD_TIME: the next packet is expected after ANT_period counter value. Radio is switched on GUARD_TIME before that time in order to allow time drift.
// current value is 2.5ms. Tune this value for tradeoff between battery saving and RX robustness.
#define GUARD_TIME					0xa00

//***V1.01
// RX_WINDOW_TIME: window time during which an packet is expected. Overflow of this timer will cause a MISSED_MESSAGE event. This time must be greater than GUARD_TIME
#define RX_WINDOW_TIME			0x1760    	//6ms

//***V1.01
// RX_TIME is the amount of time that need to be removed from ANT_period in order to allow the receiver to turn on, and to start receiving the packets
#define RX_TIME 						280

//***V1.01
// SCAN_TIME is the number of extra periods, divided by 16, to be awaited before calling a scan timeout
// example: SCAN_TIME = 25 scan duration = ANT_period * (1 + 25/16) = ANT_period * 2,5625
#define SCAN_TIME						0x8	    		// scan = 1,5 * ANT_period

//***V1.01: TMR_ACK_H and TMR_ACK_L of version 1 joined and NOT-ed for consistency with other timers
// Timer value for ACK packets
#define TMR_ACK							0x06e0

//***V1.01: TMR_BURST_H and TMR_BURST_L of version 1 joined and NOT-ed for consistency with other timers
// Timer value for BURST packets (time between a packet and its ACK, or between an ACK and the following packet)
#define TMR_BURST						0x0466 

//***V1.01
// Timer used for retransmission whenever a burst packet is missed (approx 2 * TMR_BURST, keeping into account execution/TX time)
#define BURST_TIMEOUT				0x0a77 

#define BURST_MAX_RETRIES		0x05	// Maximum number of retries after a missed BURST packet


/** ANT packet (initialize to some defaults)
*
*		Important! Declare the arrays ANT_packet and Temp_ANT_RX as "extern" in SPI routines file (e.g. nRF_API.c),
*		in order to make them accessible.
*
*/
BYTE xdata ANT_packet[15] = {ANT_Preamble_LO, ANT_Preamble_HI, 0x21, 0x0, 0x1, 0x1};			// template for ANT ACK packet, stores also informations about the channel [dd DD][dt][tt]
BYTE xdata Temp_ANT_RX[10];								// temporary location	for RX packets. Size is 17 Bytes - 5 Bytes (address) - 2 Bytes (CRC)

BYTE xdata ANT_RX_buf[MAX_BUFFER][8]; 		// buffer for RX packets (8 bytes each). In case of single packet (BROADCAST or ACKNOWLEDGED) only ANT_RX_buf[0][...] is used
UINT BufferCounter = 0;										// Size - 1 of burst transmission received, or last element of ANT_RX_buf[] used (0 = no RX burst transmission in course)

BYTE nRF_channel = 0x42;									// ANT radio channel (0...125)
BYTE LastPacket = NOT_VALID;							// Content of ANT_RX_buf, last packet

//***V1.01
// ANT channel period
unsigned long int xdata ANT_period = 0x3d090;		// Default value is 8192. 8192/32768 = 0,25s that is 250000us, that is 0x3d090 with Timer2 (1 tick = 1us)
																								// Max value is 0xFFFF, that would be 0x1E8480, 24 bit long. A long int is used. 

//***V1.01
// Timer variables
BYTE xdata TimerMode = TIMER_NONE;							// Stores information about what kind of timer is running
BYTE xdata ANT_period_count;										// Used to count how many times the timer should run from 0x0000 to 0xffff for ANT_period (needed because ANT_period is >0xFFFF)
unsigned long int xdata ANT_period_timer;				// 24 bit timer value or ANT_period
UINT xdata ANT_DelayAccu = 0;										// Used to account time used for ACKs and retransmissions, to be removed rom ANT_Period


//***V1.01
//nRF --> ANT transmission managment
BYTE TxPacket[8];       												// Packet to be sent to ANT. It will be sent to ANT as soon as a packet matching TxType is received. It must be set by host application.
												
// Variable used to mark how the next TX packed should be sent. 
// TX_NONE:								No TX packet will be sent. If an ACK is necessary, ACK packet will contain last used data.
// TX_ACK: 								Packet sent only in response to ACKNOWLEDGED packet. Correct receiving is not signaled.
// TX_BurstACK:						Packet sent only in response to BURST packet. Correct receiving is signaled.
// TX_UnreqBROADCAST:			BROADCAST packet sent only in response to BROADCAST packet. Correct receiving is not signaled.
// TX_UnreqACKNOWLEDGED:	ACNKOWLEDGED packet sent only in response to a BROADCAST packet. Correct receiving is signaled.
// TX_UnreqBURST:					BURST tranmission initiated by nRF device. The number of packets to be transmitted must be set in TxBurstCount. Correct receiving is signaled.
BYTE TxType = TX_NONE;							
UINT TxBurstCount = 0;		// Number of tx packet to be loaded in the TX burst transmission. The counter will decrease during tranmission and it will be 0 at the end of it.
													// Take care not to set it with a value <2 when setting TxType to TX_UnreqBURST (A burst transmission of 1 packet is an ACKNOWLEDGED transmission)

// Variables esed to count wether a TX packet has been transmitted. Every time a new TX packet is loaded in TX fifo for transmission, TXLoadedPacket is increased.
// Whenever thr packet has been transmitted, TXSentPacket is set = TXLoadedPacket.
// Only if the last loaded packet has been transmitted, a new one is request to the host application via TX_PACKET_SENT event.
BYTE TXSentPacket = 0x00;	
BYTE TXLoadedPacket = 0x00;


/** Burst mode variables
*
*
*/


//***V1.01
// State machine for TX burst transmission
BYTE TXBurstState = TX_BURST_OFF;

BYTE BurstRetries = BURST_MAX_RETRIES;	// Retransmission counter for TX burst
BYTE TxPacketAcked = 0;									// Flag that signals if TX burst packet has been acknowledged by ANT

// This constant array stores the association between TxType: TX_NONE, TX_ACK, TX_BurstACK, TX_UnreqBROADCAST, TX_UnreqACKNOWLEDGED, TX_UnreqBURST and the content of LastPacket.
// Index of array is a TxType, and result is a LastPacket. Special cases are treated in function CheckMatchRxTx()
// It is used to check wether the received packet matches the type marked for transmission. 
code const BYTE TxRxPairs[6] = {0xff, ACKNOWLEDGED, BURSTINPROGRESS, BROADCAST, BROADCAST, BROADCAST};
BYTE TxMatchesRx = 0;										// Flag that signals if current TX packet matches RX


// RX Burst state machine 
BYTE BurstState = BURST_OFF;							// burst state machine state

// Other variables
BYTE xdata reversed[5];										// Used as temporary variable to reverse array for SPI commands
BYTE flagPacketSent = 0;									// Set by interrupt whenever a TX packet is sent
BYTE QueuedScan = 0;											// ***V1.01 Flag set whenever a Scan must be performed after ACK transmission

/** ANT parameters
*
*		Set these parameters according to ANT device settings by calling related nRF_ANT_SET_xxx(xxx)
*		These functions assemble the packet as shown above, and set related parameters in nRF chip when necessary
*		When changing device number, device type or radio channel, the radio is switched off setting CE=0
*		Settings can be changed before or after nRF_ANT_init(), there is no influence on that.
*
*/
void nRF_ANT_SET_DeviceNo(UINT ANT_DeviceNo)
{
	ANT_packet[2] = ANT_DeviceNo & 0xff;					//low byte of device number
	ANT_packet[3] = (ANT_DeviceNo >> 8) & 0xff;		//high byte of device number
	CE_Pin(CE_LOW);																//switch CE low in order to allow SPI command
	nRF_SET_P0_address();													//set new bytes in nRF address RX
	nRF_SET_TX_address();													//and TX
	nRF_Flush();																	//flush nRF RX/TX buffers
}

void nRF_ANT_SET_DeviceType(BYTE ANT_DeviceType)
{
	ANT_packet[4] = ANT_DeviceType;
	CE_Pin(CE_LOW);																//switch CE low in order to allow SPI command
	nRF_SET_P0_address();													//set new bytes in nRF address RX
	nRF_SET_TX_address();													//and TX
	nRF_Flush();																	//flush nRF RX/TX buffers
}

void nRF_ANT_SET_TransmissionType(BYTE ANT_TransmissionType)
{
	ANT_packet[5] = ANT_TransmissionType;
}

// channel must be 0...125
void nRF_ANT_SET_Channel(BYTE ANT_Channel)
{
	nRF_channel = ANT_Channel;
	CE_Pin(CE_LOW);						
	nRF_SET_channel();
}

//***V1.01
// Set ANT channel period. Pass as argument the same Channel Period set in ANT device
// Adjust this function in target system in order to calculate the right time interval
void nRF_ANT_SET_Period(UINT ANT_ChannelPeriod)
{
	// Channel period[s] is ANT_ChannelPeriod / 32768
	// The timer value is expressed in [us], that is, multiplied by 1000000
	// ANT_period = ANT_ChannelPeriod * 1000000 / 32768
	// 1000000/32768 = 5^6 * 2^6 / 2^15 = 5^6 / 2^9 = 15625 / 2^9
	// from this result, it is possible to remove the amount of time needed for transmission and software execution.
	// That is: RX_TIME = 130us (from standby-I to RX mode) + 150us (17 bytes + preamble transmission @ 1Mbps) + execution time
	// however, RX_WINDOW_TIME and GUARD_TIME allow some tolerance.
	Stop_Timer();				// Stop running timers
	CE_Pin(CE_LOW);			// Switch off CE
	ANT_period = ((unsigned long int)ANT_ChannelPeriod * 15625) >> 9;		// Calculate timer value I
	ANT_period = ANT_period - RX_TIME;																	// Calculate timer value II
}


//***V1.01
/** ANT commands
*
*		Host application should issue ANT commands with these functions
*
*		Initialization sequence: 1-nRF_ANT_init() 2-nRF_ANT_power() 3-nRF_ANT_scan()
*		Other commands will bring again the transmitter in power off and CE off state. 
*
*/


//initialization of nRF chip for ANT compatibility. Everything is set on pipe0, the other pipes are disabled.
void nRF_ANT_init(void)
{
	//reset burst state machines
	BurstState = BURST_OFF;
	TXBurstState = TX_BURST_OFF;
	
	//turn off nRF power and reset every timer
	nRF_ANT_power(0);
	
	//turn off CE (needed for accepting SPI commands)
	CE_Pin(CE_LOW);						

	//enable interrupt for RX_DR, TX_DS and disable MAX_RT (auto retransmission not compatible with ANT)
	//enable CRC 2 Bytes, PWR_UP=0, PRIM_RX=1 --> CONFIG = 0b00011101 = 0x1d
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x1d);
	
	//set data rate to 1Mbps, TX power= 0dB --> RF_SETUP = 0b00000110 = 0x6
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x6);         
	
	//set radio channel
	nRF_SET_channel();
	
	//disable auto ack (for all pipes)
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x0);
	
	//switch on data pipe 0
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);
	
	//set address and address lenght for data pipe 0 (RX and TX)
	nRF_SET_P0_address();
	nRF_SET_TX_address();

	//set payload length for pipe 0 (17 bytes - used address length - 2 bytes CRC)
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, 0xa);

	//switch off dynamic payload
	SPI_RW_Reg(WRITE_REG + DYNPD, 0);
	
	//flush RX and TX	
  nRF_Flush();
}

//***V1.01 
// if PowerOn != 0 --> power on nRF in rx mode
// if PowerOn == 0 --> power off nRF, force CE to low
// USE ANT scan after nRF_ANT_power for connection
void nRF_ANT_power(BYTE PowerOn)
{
	BYTE ConfigRead;
	ConfigRead = SPI_Read(CONFIG);											// Read back current CONFIG register status
	if (PowerOn) {
		ConfigRead |= 0x2;																// set to 1 power on bit	
	} else
	{
		Stop_Timer();																			// stop all running timers
		CE_Pin(CE_LOW);																		// turn CE low	
		ConfigRead &= 0xfd;																// reset to 0 power bit
	}
	SPI_RW_Reg(WRITE_REG + CONFIG, ConfigRead);					// write the modified register
}

//***V1.01
// Switch on CE and waits for incoming packets. It must be called after a nRF_init() and nRF_power(1) 
// and scans for incoming packets on the channel. Waiting time is defined by SCAN_TIME and ANT_period (see comments in SCAN_TIME)
// If timer ends and no packet is received, nRF_ANT_scan_fail() is called.
void nRF_ANT_scan(void)
{
		CE_Pin(CE_HIGH);												// Turn CE HIGH	
		Start_Timer(TIMER_ANTSCAN);							// Start timeout timer for scan
}

//***V1.01 
/** ANT returns
*
*		This function is called whenever a relevant event on the ANT channel occurs
*
*/

//	ANT events. Should be used to interface with host system
//	***Important! This function runs as an interrupt routine, a flag/poll mechanism should be used in host code in order not to stop interrupts execution
void nRF_ANT_event(BYTE EventCode)
{
	switch (EventCode)
	{
		case RECEIVED:
				// Data available in ANT_RX_buf[0]. Data type is available in variable LastPacket
		break;
		
		case BURST_RECEIVED:
			// A BURST (RX) transmission succesfully finished; The data is available in ANT_RX_buf[0...BufferCounter]
		break;
		
		case TX_PACKET_SENT:
			// A packet of TxType has just been sent (for TxType without acknowledgment) or succesfully received by ANT (for TxType with acknowledgement: TX_BurstACK, TX_UnreqACKNOWLEDGED, TX_UnreqBURST)
			// *** GOOD POINT TO LOAD THE NEXT PACKET TO BE SENT (IF ANY) AND UPDATE TxType
		break;

		case TX_BURST_SENT:
			// A TX BURST transmission ended succesfully
		break;
		
		case SCAN_TIMEOUT:
			// no packets received during the scan time
		break;
		
		case	MISSED_MESSAGE:
			// after syncing master and slave, an expected packet is not received in the expected timeslot
			// host application may choose to switch off radio after a number of MISSED_MESSAGE, or initiate a new scan
		break;
		
		case BURST_FAIL:
			// A RX BURST transmission ANT --> nRF just failed. 
		break;
		
		case UNREQ_ACK_FAIL:
			// An unrequested ACKNOWLEDGED packet was not acknowledged in time by ANT.
		break;		
		
		case UNREQ_BURST_FAIL:
			// An unrequestes BURST (TX) transmission failed
		break;
	}
}
		
		// Transmission nRF-->ANT
		// If host application needs to send a packet (or a burst) to ANT, the following action must be performed:
		// 1) Load the array BYTE TxPacket[] with the payload to be transmitted using LoadTXPacket().
		// 2) Set the variable TxType according to which type of transmission should be used;
		// 
		// Notes:
		// -If TxType is set to TX_NONE, but the received packet needs an ACK packet (i.e. ACKNOWLEDGED or BURST), the last TxPacket[] is used as payload
		// -If TxType is set to TX_UnreqBURST, host application must also set TxBurstCount with the number of 8-bytes packets to be sent. 
		// -TX packets can be loaded at anytime. LoadTXPacket() cancels pending transmissions (for instance, TxType was set to TX_ACK and no ACKNOWLEDGED packet has been received yet)
		//  TX_PACKET_SENT event informs when the last loaded packet has been sent, and it's called before a new packet is loaded in the TX FIFO of nRF.
		// -If TxType is set to not-null type, the TX packet will be sent only when the right received packet is matched, that is:
		// 		TX_ACK: 								an ACKNOWLEDGED packet is received
		// 		TX_BurstACK:						a BURST packet is received
		// 		TX_UnreqBROADCAST:			a BROADCAST packet is received
		// 		TX_UnreqACKNOWLEDGED:		a BROADCAST packet is received
		// 		TX_UnreqBURST:					a BROADCAST packet is received (or a TX BURST transmission is running)
		// -After transmitting a packet, TxType is set to TX_NONE. If TxType == TX_UnreqBURST, TxType is set to TX_NONE at the end of the burst transmission.


// ***V1.01 
// Use this function to copy your TX payload to TxPacket. Additionally, this function cancels any previous pending TX packets
// 
void LoadTXPacket(void)
{
// *** Insert here your code to copy your payload in TxPacket
	
if (TXSentPacket != TXLoadedPacket)		// If there was a pending packet to be sent
	TXLoadedPacket--;										// Decrease the TXLoadedPacket counter and allow a new packet to be loaded
}
	
/****************************************************************************************************************************************************** 
/******************************************************************************************************************************************************
*   Internal functions
*
*		The following functions don't need to be called by host code
*
*/

/** nRF parameters
*
*  These functions set the parameters on the nRF device
*
*/

void nRF_SET_channel(void)
{
	SPI_RW_Reg(WRITE_REG + RF_CH, nRF_channel);
}

// The address must be the first part of ANT packet. Address can be 2 to 5 bytes long. We use the first 5 bytes
// that is: [5b 25][dd DD][dt] in order to minimize false triggering of IRQ
void nRF_SET_P0_address(void)
{																			
	ReverseArray(ANT_packet, 5);															// reverse address, SPI accepts from MSB to LSB
	SPI_RW_Reg(WRITE_REG + SETUP_AW, 0x03);										// Set address length = 5 bytes
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, reversed, 5);		 		// Take the first 5 bytes from ANT_packet to be used as address for P0
}

// Set the address of TX packets. RX and TX address must be the same
void nRF_SET_TX_address(void)
{
	ReverseArray(ANT_packet, 5);
	SPI_Write_Buf(WRITE_REG + TX_ADDR, reversed, 5);		 			// Take the first 5 bytes from ANT_packet to be used as TX address
}

void nRF_Flush(void)
{
	SPI_RW_Reg(FLUSH_RX,0);
  SPI_RW_Reg(FLUSH_TX,0);
}

/** RX functions
*		
* 	these functions manage the incoming packet
*
*/

//***V1.01 Updated: CE is not turned on after receiving packet
// Call nRF_RX() with an interrupt on the IRQ pin of nRF chip (falling edge)
void nRF_RX(void)
{
	BYTE temp;
	CE_Pin(CE_LOW);																								// For compatibily with some nRF24L01+, it's preferrable to keep CE low while reading payloads
	temp = SPI_RW_Reg(WRITE_REG + STATUS, MASK_IRQ_FLAGS);				// Read status register to get interrupt source and clear IRQs in nRF
		switch(temp & MASK_IRQ_FLAGS)
		{
			case RX_DR: 																							// Is there a new RX packet? 
				do
				{
					SPI_Read_Buf(RD_RX_PLOAD, Temp_ANT_RX, 10);						// Read Payload and copy it to Temp_ANT_RX
					if (Temp_ANT_RX[0] ==  ANT_packet[5])									// Check if [tt] field o received message matches with settings (the other 5 bytes are filtered automatically by nRF)
						ANT_RX();																						// Valid ANT packet, go to decode		
					temp = SPI_Read(FIFO_STATUS);													// Read if any other packet is present in RX FIFO		
				}	while (!(temp & 0x1));																// Check bit0 of FIFO_STATUS: if FIFO is not empty, repeat reading (should not happen)
				break;
				
			case TX_DS:       
				flagPacketSent = 1;                                     // Flag for TX_DS (packet sent)
			break;
		}
}

// Manage a valid ANT packet: switch according to packet type value. 
// For every packet type there is a switch selection that behaves according to the current state machine state.
void ANT_RX(void)
{
	if (TimerMode == TIMER_ANTSCAN)													// If a scan was running
		Stop_Timer();																					// Stop it
	
	switch(Temp_ANT_RX[1])            											// Check [pt] packet type
  {
		case PCK_BROADCAST:																		// a BROADCAST packet was received
			if(TXBurstState!=TX_BURST_OFF)											// if TX burst mode was active, switch off burst mode
					QuitTXBurstMode();

			LastPacket = BROADCAST;															// Save type of message
			if (TxType>=TX_UnreqBROADCAST)											// If there is a packed marked for transmission with unrequested message
			{
				switch (TxType)																		// Behave accordingly with TxType
				{
					case TX_UnreqBROADCAST:													// Unrequested BROADCAST:
							ANT_DelayAccu = TMR_ACK;										// Load time needed for TX BROADCAST in delay accumulator 
							Start_Timer(TIMER_ACK);											// Start timer for TX BROADCAST
							PrepareTXPacket(UNREQ_BROADCAST);						// Prepare and load in TX FIFO proper BROADCAST packet
					break;
					case TX_UnreqACKNOWLEDGED:											// Unrequested ACKNOWLEDGED:
							ANT_DelayAccu = TMR_ACK;										// Load time needed for TX ACKNOWLEDGED in delay accumulator 
							Start_Timer(TIMER_ACKTXANDWAIT);						// Start timer for TX ACKNOWLEDGED
							PrepareTXPacket(UNREQ_ACKNOWLEDGED);				// Prepare and load in TX FIFO proper ACKNOWLEDGED packet
					break;
					case TX_UnreqBURST:															// Unrequested BURST transmission (TX)
							Start_Timer(TIMER_TXBURSTSTART);						// Start timer for TX BURST start
							PrepareTXPacket(UNREQ_BURST_START);					// Prepare and load in TX FIFO proper BURST START packet
							TXBurstState = TX_BURST_START;							// Set state machine to TX_BURST_START	
					break;
				}
			}
			else
			{
				ANT_DelayAccu = 0;																// Packet doesn't need ACK, reset delay accumulator
				Start_Timer(TIMER_ANTPERIOD);											// Start timer for radio ON again next period
			}
															
			if(BurstState!=BURST_OFF)														// If burst mode was active, switch off burst mode
			{
				if((BurstState==END_0)||(BurstState==END_1))			// If last packet was the end of a burst transmission, reset burst state machine
				{
					BurstState=BURST_OFF; 													// state machine is kept in BURST_OFF mode in order to detect double PCK_BURST_END_x (i.e. ack was lost), legit burst mode end
					TxPacketSent();																	// last ack of BURST transmission was received, increment TXSentPacket
				}
				else
					QuitBurstMode();																// burst mode interrupted by sudden BROADCAST packet
			}
			BufferCounter = 0;						
			SaveMsg();																					// Save valid BROADCAST message
			nRF_ANT_event(RECEIVED);														// Call ANT_event function
		break;
		
		case PCK_ACKNOWLEDGED:  															// An ACKNOWLEDGED packet is received
			ANT_DelayAccu = TMR_ACK;														// Load time needed for ACK in delay accumulator 
			
			Start_Timer(TIMER_ACK);															// Start timer for Ack
			LastPacket = ACKNOWLEDGED;													// Save type of message

			if(TXBurstState!=TX_BURST_OFF)											// if TX burst mode was active, switch off burst mode
					QuitTXBurstMode();
		
			if(BurstState!=BURST_OFF)														// if burst mode was active, switch off burst mode
			{
				if((BurstState==END_0)||(BurstState==END_1))			// if last packet was the end of a burst transmission, reset burst state machine
				{
					BurstState=BURST_OFF; 													// state machine is kept in BURST_OFF mode in order to detect double PCK_BURST_END_x (i.e. ack was lost), legit burst mode end
					TxPacketSent();																	// last ack of BURST transmission was received, increment TXSentPacket
				}
				else
					QuitBurstMode();																// burst mode interrupted by sudden BROADCAST packet
			}
				
			PrepareTXPacket(ACK_ACKNOWLEDGED);									// Prepare and loads in TX FIFO proper ack packet
			BufferCounter = 0;
			SaveMsg();																					// Save valid ACKNOWLEDGED message
			nRF_ANT_event(RECEIVED);														// Call ANT_event function
		break;

// BURST state machine:
// Read the current packet type and behave accordingly to current state machine.
// For every packet type there is a switch according to the state machine:
// First case:
// the incoming packet is of the same type of last packet. That means that the last ACK went lost. 
// if the packet is exactly the same of the last received, the ACK is sent again (BufferCounter is not incremented)
// else the burst mode is quit with an error
// Second case:
// This is the expected packet. In this case, prepare the ACK packet, save the message and set LastPacket value properly
// Default case:
// Any other unexpected packet is dropped and the BURST mode is quitted
		
		case PCK_BURST_START:
			switch (BurstState)
			{
						case BURST_START:														// Start message received twice: probably ack was lost.
 								if (PacketRepeated())										// Checks if packet is identical to last received
 								{
									Start_Timer(TIMER_BURSTSTART);				// Start timer for Ack transmission, burst mode
 									PrepareTXPacket(ACK_BURST_START);			// Load correct payload in TX FIFO
 								}	else
 									QuitBurstMode();											// Error, quit burst mode
						break;

						case BURST_OFF:
							Start_Timer(TIMER_BURSTSTART);						// Start timer for Ack transmission, burst mode
							LastPacket = BURSTINPROGRESS;							// Save type of message
							PrepareTXPacket(ACK_BURST_START);					// Prepare and loads in TX FIFO proper ack packet
							BufferCounter = 0;
							SaveMsg();																// Save valid broadcast message
							BurstState = BURST_START;									// Burst mode started
						break;
						
						default:
								QuitBurstMode();												// Error, quit burst mode
						break;
				}
		break;
		case PCK_BURST_0:
			switch (BurstState)
			{
						case BURST_0:
								if (PacketRepeated())										// Checks if packet is identical to last received
								{
									Start_Timer(TIMER_BURST);							// Start timer for Ack transmission, burst mode
									PrepareTXPacket(ACK_BURST_0);					// Load correct payload in TX FIFO
								}	else
									QuitBurstMode();											// Error, quit burst mode
						break;
						
						case BURST_START:														// It is possible to arrive to burst0 after start
						case BURST_1:
								Start_Timer(TIMER_BURST);								// Start timer for Ack transmission, burst mode
								LastPacket = BURSTINPROGRESS;						// Save type of message
								TxPacketSent();													// Last ack of burst packet was correctly received, increment counter of TX packets
								PrepareTXPacket(ACK_BURST_0);						// Prepare and loads in TX FIFO proper ack packet
								BufferCounter++;
								SaveMsg();															// Save valid broadcast message
								BurstState = BURST_0;										// new state
						break;
							
						default:
								QuitBurstMode();												// Error, quit burst mode
						break;						
			}
		break;
		case PCK_BURST_1:  
			switch (BurstState)			
			{
						case BURST_1:
								if (PacketRepeated())										// Checks if packet is identical to last received
								{
									Start_Timer(TIMER_BURST);							// Start timer for Ack transmission, burst mode
									PrepareTXPacket(ACK_BURST_1);				// Load correct payload in TX FIFO
								}	else
									QuitBurstMode();											// Error, quit burst mode
						break;
						
						case BURST_0:
								Start_Timer(TIMER_BURST);								// Start timer for Ack transmission, burst mode
								LastPacket = BURSTINPROGRESS;						// Save type of message
								TxPacketSent();													// Last ack of burst packet was correctly received, increment counter of TX packets
								PrepareTXPacket(ACK_BURST_1);						// Prepare and loads in TX FIFO proper ack packet
								BufferCounter++;
								SaveMsg();															// Save valid broadcast message
								BurstState = BURST_1;										// new state
						break;
						
						default:
								QuitBurstMode();												// Error, quit burst mode
						break;						
			}
		break;
		case PCK_BURST_END_0:  
			switch (BurstState)			
			{
						case END_0:
									if (PacketRepeated())									// Checks if packet is identical to last received
								{
									Start_Timer(TIMER_BURST);							// Start timer for Ack transmission, burst mode
									PrepareTXPacket(ACK_BURST_END_0);		// Load correct payload in TX FIFO
								}	else
									QuitBurstMode();											// Error, quit burst mode
						break;
						
						case BURST_START:														// A PCK_BURST_END_0 can come also after a PCK_BURST_START
						case BURST_1:																// Succesful burst transfer END!
								Start_Timer(TIMER_BURST);								// Start timer for Ack transmission, burst mode
								LastPacket = BURST;											// Save type of message
								TxPacketSent();												// Last ack of burst packet was correctly received, increment counter of TX packets
								PrepareTXPacket(ACK_BURST_END_0);				// Prepare and loads in TX FIFO proper ack packet
								BufferCounter++;
								SaveMsg();															// Save valid broadcast message
								BurstState = END_0;											// new state
								BurstSuccess();
						break;
						
						default:
								QuitBurstMode();												// Error, quit burst mode
						break;						
			}
		break;
		case PCK_BURST_END_1: 
			switch (BurstState)				
			{
						case END_1:
									if (PacketRepeated())										// Checks if packet is identical to last received
								{
									Start_Timer(TIMER_BURST);								// Start timer for Ack transmission, burst mode
									PrepareTXPacket(ACK_BURST_END_0);				// Load correct payload in TX FIFO
								}	else
									QuitBurstMode();												// Error, quit burst mode
						break;
				
						case BURST_0:																	// Succesful burst transfer END!
								Start_Timer(TIMER_BURST);									// Start timer for Ack transmission, burst mode
								LastPacket = BURST;												// Save type of message
								PrepareTXPacket(ACK_BURST_END_1);					// Prepare and loads in TX FIFO proper ack packet
								TxPacketSent();														// Last ack of burst packet was correctly received, increment counter of TX packets
								BufferCounter++;
								SaveMsg();																// Save valid broadcast message
								BurstState = END_1;												// new state						
								BurstSuccess();
						break;
						
						default:
								QuitBurstMode();													// Error, quit burst mode
						break;						
			}
		break;	
		
	//
	// TX BURST state machine:
	//
		
		case PCK_ACNKOWLEDGE:																					// This is packet type = 0xf2, valid both for ACK_ACNOWLEDGED and ACK_BURST_END_1
			
			if(BurstState!=BURST_OFF)																		// If burst mode was active, quit burst mode
				QuitBurstMode();																					
		
			if (TxType == TX_UnreqACKNOWLEDGED)													// If this is an ACK for a unrequested ACNKNOWLEDGED packet
			{
				ANT_DelayAccu += ~( TMR2H<<8 + TMR2L + RX_WINDOW_TIME);		// ACK received, increase accumulator of time elapsed from start of TIMER_ACKTXANDWAIT
				Start_Timer(TIMER_ANTPERIOD);															// Start timer for radio ON again
				CE_Pin(CE_LOW);																						// CE LOW, waiting or next period
				TxPacketSent();																						// Mark packet as sent
				LastPacket = ACKNOWLEDGE;																	// Save packet type
				SaveMsg();																								// Save valid ACKNOWLEDGE message
				nRF_ANT_event(RECEIVED);																	// Call event
			}
			
			if (TXBurstState != TX_BURST_OFF)					// If this is an ACK for an unrequested BURST transmission
			{
				switch (TXBurstState)										// Switch according to state machine
				{
					case TX_BURST_END_1:									// Last packet type 1		
						TXBurstState = TX_BURST_OFF;				// Update state machine
						TxPacketSent();											// Mark packet as sent
						LastPacket = BURST_ACK;							// Save received BURST_ACK packet
						SaveMsg();													// Save valid ACKNOWLEDGE message
						nRF_ANT_event(RECEIVED);						// Call event
						TXBurstSuccess();										// TX burst success
					break;
					
					default:
						QuitTXBurstMode();									// Wrong packet, quit TX BURST
					break;
				}	
			}				
		break;
			
		case ACK_BURST_1:													
			if(BurstState!=BURST_OFF)								// If burst mode was active, quit burst mode
					QuitBurstMode();										
			
			switch (TXBurstState)										// Switch according to state machine
			{
				case TX_BURST_1:											// (This ACK packet can arrive also after a TX_BURST_START)
				case TX_BURST_START:
					Start_Timer(TIMER_TXBURST);					// Start timer or TX burst packet
					TxPacketAcked = 1;									// ACK has been received
					TxPacketSent();											// Mark packet as sent
					if (TxBurstCount>1)									// If the packet to be sent is not the last
					{
						PrepareTXPacket(PCK_BURST_0);			// Send a PCK_BURST_0
						TXBurstState = TX_BURST_0;				// update state
					}
					else																// If it's last packet
					{
						PrepareTXPacket(PCK_BURST_END_0); // Send a PCK_BURST_END_0
						TXBurstState = TX_BURST_END_0;		// update state		
					}
					LastPacket = BURST_ACK;							// Save received BURST_ACK packet
					SaveMsg();													// Save valid ACKNOWLEDGE message
					nRF_ANT_event(RECEIVED);						// Call event
				break;
					
				case TX_BURST_0:											// Old ack received, retransmit old packet without quitting (wait for timeout)
					Start_Timer(TIMER_TXBURST);					// Start timer for retransmitting TX burst packet
				break;
				
				default:
					QuitTXBurstMode();									// Wrong packet
				break;
			}
		break;
				
		case ACK_BURST_0:													 
			if(BurstState!=BURST_OFF)								// If burst mode was active, quit burst mode
					QuitBurstMode();										// With error
			
			switch (TXBurstState)										// Switch according to state machine
			{
				case TX_BURST_0:											
 					Start_Timer(TIMER_TXBURST);					// Start timer or TX burst packet
 					TxPacketAcked = 1;									// ACK has been received
					TxPacketSent();											// Mark packet as sent
 					if (TxBurstCount>1)									// If the packet to be sent is not the last
 					{
 						PrepareTXPacket(PCK_BURST_1);			// Send a PCK_BURST_1
 						TXBurstState = TX_BURST_1;				// update state
 					}
 					else																// If it's last packet
 					{				
 						PrepareTXPacket(PCK_BURST_END_1); // Send a PCK_BURST_END_0
 						TXBurstState = TX_BURST_END_1;		// update state		
 					}
 					LastPacket = BURST_ACK;							// Save received BURST_ACK packet
 					SaveMsg();													// Save valid ACKNOWLEDGE message
 					nRF_ANT_event(RECEIVED);						// Call event
 				break;
 				
				case TX_BURST_1:											// Old ack received, retransmit old packet without quitting (wait for timeout)
					Start_Timer(TIMER_TXBURST);					// Start timer for retransmitting TX burst packet
				break;

 				default:
 						QuitTXBurstMode();						// Wrong packet
 				break;
 			}

		break;
			
		case ACK_BURST_END_0:											 
			if(BurstState!=BURST_OFF)								// If burst mode was active, quit burst mode
				QuitBurstMode();											// With error
			
			switch (TXBurstState)										// Switch according to state machine
			{
				case TX_BURST_END_0:											
					TXBurstState = TX_BURST_OFF;				// update state
					TxPacketSent();											// Mark packet as sent
					LastPacket = BURST_ACK;							// Save received BURST_ACK packet
					SaveMsg();													// Save valid ACKNOWLEDGE message
					nRF_ANT_event(RECEIVED);						// Call event
					TXBurstSuccess();
				break;
				
				default:
					QuitTXBurstMode();									// Wrong packet
				break;
			}
		break;	
			
		// case ACK_BURST_END_1 is the same as case PCK_ACKNOWLEDGE, because both of them have packet type = 0xf2
	
		default:
				// unrecognized packet type
		break;
	}
}

// Copy payload of message in ANT_RX_buf, removing ANT address bytes. Returns 1 if RX buffer had overflow.
BYTE SaveMsg(void)	
{
	BYTE count;
	BYTE BufferOverflow = 0;
	if (BufferCounter > MAX_BUFFER)
		BufferOverflow = 1;
	for (count = 0; count < 8; count++)
	{
		ANT_RX_buf[BufferCounter % MAX_BUFFER][count] = Temp_ANT_RX[count + 2];
	}
	return BufferOverflow;
}

//***V1.01
// Load TX packet in TX fifo; If a TX packet is supplied by host application, and the TX type matches received packet, use it
void PrepareTXPacket(BYTE PacketType)
{
	BYTE count;																									// counter for loading TX payload	
	
	TxMatchesRx = CheckMatchRxTx();															// Check if TX packet matches currently received packet, and save it in a flag
																															
	if (TxMatchesRx && (TXLoadedPacket == TXSentPacket))				// Check if a new TX packet from host app must be loaded - last loaded packet was succesfully sent?
	{
		TXLoadedPacket++;																					// If yes, update TXLoadedPacket
		for (count = 0; count < 8; count++)												// And load the new packet in TX FIFO
			{
				ANT_packet[7+count]=TxPacket[count];					
			}														
	}
	ANT_packet[6]	= PacketType;																	// If last TX packet was not succesfully sent or TxType doesn't match received packet, send again last TX packet.
	SPI_Write_Buf(WR_TX_PLOAD, ANT_packet + 5, 10);							// In this case, payload must not be reversed. Packet will be sent as soon as Timer2 is elapsed.
}

// ***V1.01
// This function increments TX packet counter
void TxPacketSent(void)
{
	if (TxMatchesRx)									// If last TX packet matched marked TX type
	{
		nRF_ANT_event(TX_PACKET_SENT);	// Call event TX_PACKET_SENT
		TXSentPacket = TXLoadedPacket;	// The packet waiting was succesfully transmitted, increment Tx Packet counter
		if (TxType != TX_UnreqBURST)		// Only if a TX BURST is not running
			TxType = TX_NONE;							// Set TX_NONE (in case of TX BURST TxType is set to TX_NONE at the end of the transmission)
	}
}


/** Burst mode functions
*		
*  	function used to manage Burst mode state machine
*/

// Check if received packed is identical to last packet
BYTE PacketRepeated(void)
{
	BYTE isRepeated = 1;
	BYTE count;
	for (count = 0; count < 8; count++)
	{
		if (ANT_RX_buf[BufferCounter][count] != Temp_ANT_RX[count + 2])
			isRepeated = 0;
	}	
	return isRepeated;
}

//***V1.01
// Quit Burst state machine for an error in transmission
void QuitBurstMode(void)
{
	BurstState = BURST_OFF;
	if (LastPacket==BURSTINPROGRESS)		// If burst failure has not been triggered yet
		nRF_ANT_event(BURST_FAIL);				// Trigger event
	LastPacket = NOT_VALID;
	BufferCounter = 0;
	nRF_ANT_scan();										// After BURST transfer, sync is lost ANT goes in scan mode. We need to wait for ACK transmission before entering scan mode
}

//***V1.01
// Function called at the end of a succesful BURST transfer
void BurstSuccess(void)
{
	nRF_ANT_event(BURST_RECEIVED);		// Trigger event
	QueuedScan = 1;										// Set flag to perform scan after last ACK
}	

//***V1.01
// Quit Burst state machine for an error in transmission
void QuitTXBurstMode(void)
{
	TXBurstState = TX_BURST_OFF;					// Switch off TX burst mode
	if (TxType == TX_UnreqBURST)					// If TX burst failure has not been triggered yet
		nRF_ANT_event(UNREQ_BURST_FAIL);		// Trigger event
	TxType = TX_NONE;
	nRF_ANT_scan();												// After BURST transfer, sync is lost ANT goes in scan mode. We need to wait for ACK transmission before entering scan mode
}


//***V1.01
// Function called at the end of a succesful TX BURST transfer
void TXBurstSuccess(void)
{
	nRF_ANT_event(TX_BURST_SENT);				// Trigger event
	TxType = TX_NONE;										// Set TxType to TX_NONE
	nRF_ANT_scan();											// After BURST transfer, sync is lost ANT goes in scan mode.
}	

// ***V1.01
// Function that returns true if TxType matches current LastPacket
BYTE CheckMatchRxTx(void)
{
	if (TxRxPairs[TxType]==LastPacket)													// Look up in array of relations, if types match
		return 1;
	else if ((TxType == TX_BurstACK) && (LastPacket == BURST))	// Manage peculiar case of TX_BurstACK that matches both with BURSTINPROGRESS and BURST
		return 1;
	else if ((TxType == TX_UnreqBURST) && (LastPacket == BURST_ACK)) // Manage peculiar case of TX_UnreqBURST that matches both with BROADCAST and BURST_ACK
		return 1;
	else
		return 0;
}

/** Timers. ***V1.01
*		
*
*  	Delays managed by interrupt. These function must be replaced by proper ones in target system
*		I use timer2, wich is 16 bit, every bit lasts 1 us.
*		The TimerMode variable is set according to which type of timer is running, so that the consequent interrupt routine can behave accordingly.
*		This delays are very critical for ANT. 
*
*		TMR_ACK 	--> acknowledged packet --> measured delay between packets 2184,81us --> my code works with delay of 0xFFFF - 0xF91F = 1760 us (425us more for code execution and packet TX)
*		TMR_BURST --> burst packet --> measured delay between packets 1552,70us --> my code works with delay of 0xFFFF - 0xFB98 = 1172us
*
*
*/  

//***V1.01 Updated for ANTPERIOD TIMER
void Start_Timer(BYTE mode)				// Start Timer
{
	TimerMode = mode;								// Save the reason for starting the timer so that timer interrupt routine behaves accordingly
	switch(mode)
	{
		case TIMER_ACK:								// Timer value for acknowledged packet 
			Load_Timer(TMR_ACK); 
		break;
		
		case TIMER_BURSTSTART:
			Load_Timer(TMR_ACK); 				// Timer value for burst start; Time is the same as TIMER_ACK
		break;
		
		case TIMER_BURST:							// Timer value for burst packet
			Load_Timer(TMR_BURST);
		break;
		
		case TIMER_ACKTXANDWAIT:			// Timer value for sending a TX packet and waiting for the ACK
			Load_Timer(TMR_ACK);
		break;
		
		case TIMER_ACKTIMEOUT:
			Load_Timer(RX_WINDOW_TIME);	// Timeout for ACK packet
		break;
		
		case TIMER_TXBURSTSTART:
			Load_Timer(TMR_ACK);				// Timeout for TXBURST START
		break;
		
		case TIMER_TXBURST:
			Load_Timer(TMR_BURST);			// Timer value for burst packet
		break;
		
		case TIMER_BURSTTIMEOUT:
			Load_Timer(BURST_TIMEOUT);	// Timer value for burst timeout
		break;

		//***V1.01 
		// Calculate the delay value for switching ON CE
		// The delay can be greater than 16 bits. For this reason the timer ANT_period is calculated with a long unsigned int.
		// The first 16 bits of ANT_period will be used to load the timer registers TMR2H and TMR2L (after calculating 0xffff - value)
		// The bits 16..24 will be loaded in the ANT_period_count variable:
		// Timer will count ANT_period_count times from 0 to 0xffff and then it will be finally loaded with ANT_period_timer
		// The delay between a RX broadcast and following CE on is:
		// ANT_period_timer = ANT_period - ANT_DelayAccu - GUARD_TIME
			
		case TIMER_ANTPERIOD:
			ANT_period_timer = ANT_period;																// Load ANT_period value in variable for delay calculation
			ANT_period_timer -= ANT_DelayAccu;														// Subtract all accumulated delays
			ANT_period_timer -= GUARD_TIME; 						 									// Subtract GUARD_TIME
			Load_ANT_period_timer();
		break;
			
		case TIMER_ANTSCAN:
			ANT_period_timer = ANT_period;																// Load ANT_period value in variable for delay calculation
			ANT_period_timer += (ANT_period * SCAN_TIME) >> 4;						// Increase ANT_period of SCAN_TIME / 16
			Load_ANT_period_timer();
		break;
	}
	TF2H = 0;												// reset T2 interrupt flags   
	TF2L = 0;					
  ET2 = 1;												// enable interrupt
	TR2 = 1;												// Start timer
	}


//***V1.01 Updated for ANTPERIOD TIMER
// nRF IRQ interrupt must have higher priority than timer interrupt, otherwhise flagPacketSent will never be set.
// This function must be called by the timeout of delay timer. It will manage the right action to perform, according to the timer type
void TimerInterrupt(void)
{	
	switch(TimerMode)															
	{
		case TIMER_ACK:															// Timer for ACKNOWLEDGED packet or unrequested BROADCAST message
			Start_Timer(TIMER_ANTPERIOD);							// Start timer for radio ON again on next period (ANT_DelayAccu is taken into account in Start_Timer)
			SendPacket();															// Send ACK
		break;
					
		case TIMER_BURSTSTART:
			Stop_Timer();															// Burst initiated, stop ANTPERIOD timer (keep CE on during all transmission)		
			SendPacket();															// Send ACK
			CE_Pin(CE_HIGH);													// Leave CE on for ACK reception
		break;
		
		case TIMER_BURST:														// or for an incoming BURST
				Stop_Timer();														// Burst initiated, stop ANTPERIOD timer		
				SendPacket();														// Send ACK
				CE_Pin(CE_HIGH);												// listen for next BURST packet ******ADD TIMEOUT!!!*****
		break;
		
		case TIMER_ACKTXANDWAIT:
				Start_Timer(TIMER_ACKTIMEOUT);					// Start timer for ACK timeout
				SendPacket();														// Send packet
				CE_Pin(CE_HIGH);												// Keep CE high for ACK
		break;
		
		case TIMER_ACKTIMEOUT:											// Ack not received
				ANT_DelayAccu += RX_WINDOW_TIME;				// Increase delay accumulator of RX_WINDOW_TIME
				Start_Timer(TIMER_ANTPERIOD);						// Start timer for next period
				CE_Pin(CE_LOW);													// Turn off CE
				if (TXBurstState == TX_BURST_START)			// If ack was waited for TX_BURST_START
					QuitTXBurstMode();										// Quit TX burst mode
				else																		
					nRF_ANT_event(UNREQ_ACK_FAIL);				// Call event, packet not acknowledged
		break;
		
		case TIMER_TXBURSTSTART:
				Start_Timer(TIMER_ACKTIMEOUT);					// Start timer for next period
				SendPacket();														// Send packet
				TxBurstCount--;													// Decrease TX BURST packet counter
				CE_Pin(CE_HIGH);												// listen for ACK packet
		break;
		
		case TIMER_TXBURST:													// Timer for sending a TX burst pacekt
				Start_Timer(TIMER_BURSTTIMEOUT);				// Start timer for next period
				if (TxPacketAcked)
				{
					SendPacket();														// Send packet
					TxBurstCount--;													// Decrease TX BURST packet counter
					CE_Pin(CE_HIGH);												// listen for ACK packet
					BurstRetries = BURST_MAX_RETRIES;				// reset the number of retries
					TxPacketAcked = 0;											// reset ACKED flag
				} 
				else
				{
					ReTransmitTX();												// Old ACK was received, retransmit packet
				}		
		break;
		
		case TIMER_BURSTTIMEOUT:
				ReTransmitTX();													// No ACK was received, retransmit packet
				TF2H = 0;																// reset T2 interrupt flags   
				TF2L = 0;
		break;
		
		case TIMER_ANTPERIOD:												// ANT_period timer is elapsed
				if (ANT_period_count == 1)							// If it's last decrement, load remainder value
				{
					Load_Timer((UINT)ANT_period_timer);		// Load directly values of ANT_period_timer
				}
				else if (ANT_period_count == 0)					// If timer is elapsed
				{
					Load_Timer(RX_WINDOW_TIME);						// Start window timer: load values in timer
					TimerMode = TIMER_RXWINDOW;						// Set timer mode for RX window timer
					SPI_RW_Reg(WRITE_REG + CONFIG, 0x1f);	// SET RX mode PRIM_RX=1
					CE_Pin(CE_HIGH);											// Switch ON CE
				}
				TF2H = 0;																// reset T2 interrupt flags   
				TF2L = 0;
				ANT_period_count--;											// Decrease period counter. If ANT_period_count > 1, the timer reloads to 0x0000
		break;
					
		case TIMER_RXWINDOW:															// RX has timed out; If a packet is received before timeout, this timer is automatically reset
				CE_Pin(CE_LOW); 															// Turn off CE
				ANT_DelayAccu += RX_WINDOW_TIME - GUARD_TIME;	// Increment delay accumulator with time lost waiting for missed packet
				Start_Timer(TIMER_ANTPERIOD);									// Start timer for radio ON again on next timeslot
				nRF_ANT_event(MISSED_MESSAGE);								// Call ANT event managing function
		break;

		case TIMER_ANTSCAN:													// Scan timer elapsed
				if (ANT_period_count == 1)							// If it's last decrement, load remainder value
				{
					Load_Timer((UINT)ANT_period_timer);		// Load directly values of ANT_period_timer
				}
				else if (ANT_period_count == 0)					// Scan has failed
				{
					Stop_Timer();													// Stop timer
					CE_Pin(CE_LOW);												// Turn CE LOW
					nRF_ANT_event(SCAN_TIMEOUT);					// Call ANT event managing function
				}
				TF2H = 0;																// reset T2 interrupt flags   
				TF2L = 0;
				ANT_period_count--;											// Decrease period counter. If ANT_period_count > 1, the timer reloads to 0x0000
		break;
	}
}

//***V1.01 Updated for ANTPERIOD TIMER
// Can be customized to adapt to host system configuration
void Stop_Timer(void)				  // Stop Timer2 and clear T2 counter
{
  ET2 = 0; 										// disable T2 interrupt
	TR2 = 0; 										// stop T2
	TimerMode = TIMER_NONE;			// update TimerMode variable
}

//***V1.01 
// Generic timer loading function. Loads the right value in the timer, starting with the number of "ticks" to be counted.
// Can be customized to adapt target system configuration
void Load_Timer(UINT value)
{
	value = ~value;										// Timer must be loaded with 0xffff - value
	TMR2H = 0xff & (value >> 8);			// Set timer current value, high byte
	TMR2L = 0xff & value;							// Set timer current value, low byte
	TMR2RLH = 0xff & (value >> 8);		// Set timer reload value, high byte
	TMR2RLL = 0xff & value;						// Set timer reload value, low byte			
}

//***V1.01 
// Used to load long int ANT_period_timer
void Load_ANT_period_timer(void)
{
			ANT_period_count = (ANT_period_timer >> 16) & 0xff;						// Save bytes 4 and 5 in ANT_period_count
			if (ANT_period_count)																					// If timer needs to be > than 16 bit
			{
				Load_Timer(0xffff); 																				// Timer will count ANT_period_count times 0xffff
			} else
			{																															// Timer is <= 16 bit
				Load_Timer((UINT)ANT_period_timer);													// Load directly values of ANT_period_timer
			}
}

//***V1.01
//Switch to TX, send ACK loaded with PrepareTXPacket switch again to RX (CE off) 
void SendPacket(void)
{

	SPI_RW_Reg(WRITE_REG + CONFIG, 0x1e);		// set TX mode enable CRC 2 Bytes, PWR_UP=1, PRIM_RX=0 --> CONFIG = 0b00011110 = 0x1e
	CE_Pin(CE_HIGH);												// Set CE HIGH. ***can be changed with a 10us pulse
	while(!flagPacketSent){}								// wait for packet sent (updated by interrupt routine on IRQ pin) ***a timeout can be introduced for stability.
	CE_Pin(CE_LOW);												
	flagPacketSent = 0;											// reset packet sent flag
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x1f);		// return to RX mode PRIM_RX=1

	if ((LastPacket == ACKNOWLEDGED)				// If RX packet was for an ACKNOWLEDGED packet
		 ||(TxType == TX_UnreqBROADCAST))			// or if an un unrequested TX_BROADCAST has just been sent
		TxPacketSent();												// Increment counter of TX packets

	if (QueuedScan)												 	// After BURST transfer, sync is lost ANT goes in scan mode. We need to wait for ACK transmission before entering scan mode
		{																			// QueuedScan is marked to perform a scan after last TX
			nRF_ANT_scan();											// Start scan
			QueuedScan = 0;											// Reset flag for scan
		}
}

//***V1.01
//Send last packet after an ACK was lost during TX BURST transmission
void ReTransmitTX(void)
{
	BurstRetries--;													// Decrement retries
	if (BurstRetries)												// If are there still retries available
		{
			CE_Pin(CE_LOW);												// CE low for writing payload
			SPI_Write_Buf(WR_TX_PLOAD, ANT_packet + 5, 10);		// Reload last packet in ANT_packet
			SendPacket();													// Resend last packet
			CE_Pin(CE_HIGH);											// listen for ACK packet
		}
		else
			QuitTXBurstMode();										// QuitTXBurstMode
}

/** 
*
* 	Helper functions
*
*/

// Reverse the first "length" elements of an array and store it in reversed[]
void ReverseArray(BYTE *Array, BYTE length)
{
BYTE count;
for (count = 0; count < length; count++)
 		reversed[count] = Array[length - count -1];
}
