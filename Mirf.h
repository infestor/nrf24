
#ifndef _MIRF_H_
#define _MIRF_H_

#include <stdint.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "arduino_simple.h"
#include "spilib.h"
#include "packet_defs.h"

// Nrf24l settings

#define mirf_ADDR_LEN	3 //5
const char mirf_ADDR[] = "honza";
#define ADDR_TYPE uint8_t
#define MULTICAST_ADDR 0xFF //for uint16 0xffff;

#define MAX_RX_PACKET_QUEUE 10
#define MAX_TX_PACKET_QUEUE 1
#define MAX_ACK_PACKET_QUEUE 6
#define MAX_TX_ATTEMPTS 4
#define MAX_ACK_WAIT_TIME 5 //25
#define NOP_ASM __asm__("nop\n\t");

typedef struct {
  ADDR_TYPE txAddr;
  ADDR_TYPE rxAddr;
  //ADDR_TYPE origAddr;
  uint8_t  type;
  uint8_t  counter;
  uint8_t  payload[7];
} mirfPacket;

#define NRF_PAYLOAD_SIZE sizeof(mirfPacket)

//==========================================================================
class Nrf24l {
	public:
    Nrf24l();

	void init();
	void config();
	//void send(uint8_t *value);
	void setADDR(void);
    void setDevAddr(ADDR_TYPE);
	bool dataReady();
	volatile bool isSending();
	bool rxFifoEmpty();
	bool txFifoEmpty();
	void getData(uint8_t * data);
    uint8_t getCarrier();
	uint8_t getStatus();

	void removePacketfromTxQueue(void);
	void removePacketfromAckQueue(void);
  
    void handleRxLoop(void);
    void handleTxLoop(void);
    void readPacket(mirfPacket* paket);
    uint8_t sendPacket(mirfPacket* paket);
    void createAck(mirfPacket* paket);

    //,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
	void configRegister(uint8_t reg, uint8_t value);
	void readRegister(uint8_t reg, uint8_t * value, uint8_t len);
	void writeRegister(uint8_t reg, uint8_t * value, uint8_t len);
	void flushTx();
	void flushRx();
	void powerUpRx();
	void powerUpTx();
	void powerDown();
		
	void nrfSpiWrite(uint8_t reg, uint8_t *data = 0, bool readData = false, uint8_t len = 0);
	void nrfSpiWrite2(uint8_t reg, uint8_t *data = 0, bool readData = false, uint8_t len = 0);
  
	void csnHi();
	void csnLow();
	void ceHi();
	void ceLow();
    
    //-----------------------------------------------------------------------
    //------------------ Variables ------------------------------------------
    //--- setup values ------------------------------------------------------
		/**
		 * In sending mode.
		 */
		volatile uint8_t PTX;

		/**
		 * CE Pin controls RX / TX, default D9.
		 */
		//uint8_t cePin;

		/**
		 * CSN Pin Chip Select Not, default D10.
		 */
		//uint8_t csnPin;

		/**
		 * Channel 0 - 127 or 0 - 84 in the US.
		 */
		uint8_t channel;

		/**
		 * The base config register.
		 * When required PWR_UP and/or PRIM_RX will be OR'ed with this.
		 * 
		 * NOTE: Use "_BV(EN_CRC) | _BV(CRCO)" here if you want to
		 *       connect to a device using the RF24 library.
		 */
		uint8_t baseConfig;

    	//address of device
    	//unique for every node
    	//1 for master node
		ADDR_TYPE devAddr;

		/**
		 * Spi interface (must extend spi).
		 */
		SPIlib *spi;
		//MirfSpiDriver *spi; //MirfSpiDriver

    //-------------------------------------------------------------------------
    mirfPacket volatile pendingPacket;

    mirfPacket volatile rxQueue[MAX_RX_PACKET_QUEUE];
    uint8_t volatile rxPosBeg;
    uint8_t volatile rxPosEnd;

    mirfPacket volatile txQueue[MAX_TX_PACKET_QUEUE];
    uint8_t volatile txPosBeg;
    uint8_t volatile txPosEnd;
    uint8_t volatile txQueueSize;
    uint8_t volatile txAttempt;

    mirfPacket volatile ackQueue[MAX_ACK_PACKET_QUEUE];
    uint8_t volatile ackPosBeg;
    uint8_t volatile ackPosEnd;
    uint8_t volatile ackQueueSize;

    //uint8_t volatile confirmedPackets[MAX_TX_PACKET_QUEUE];
    //uint8_t volatile confirmedStackSize;

    // sign that there is received packet in buffer adressed to this device
	// 0 means no packets
	// 1..x means number of packets ready
    uint8_t volatile inPacketReady;
    
    // sign that there is sending in progress
    //0 means - no packet to be sent (ready)
    //1 means - packet in fifo, waiting for Tx
    //2 means unsent - waiting for free air
    //3 means packet sent, but waiting for ACK
    //4 means timeout
    uint8_t volatile sendingStatus;
    uint8_t volatile sendResult;  //sending status is copied here, to be able to send next packets
    
    //incremented with every new sent packet (used for identification of ACKs)
    uint8_t packetCounter;
    
    //not fully implemented now
    //address of last transmitter from which packet was received and processed
    //used for detecting of same packet, which ACK was probably not received by transmitter
    //so it is sending the same packet again
    //so we send ack again, but not process the packet because it was already processed 
    //uint8_t last_addr_in;
    //uint8_t last_packetCounter_in;
        
    //increments with every call of periodic Rx and Tx handle loop functions
    //dedicated to recognize timeouts in waiting for ACK
    //and to measure random time between send attempts
    uint8_t volatile Timer;
    uint8_t volatile TimerNewAttempt;
    uint8_t volatile ackTimeoutTimer; //timer state when timeout will occur
    
    //whether there is to be sent some ack
    //volatile bool sendAck;
    
};

extern Nrf24l Mirf;

#endif /* _MIRF_H_ */
