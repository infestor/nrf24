
#ifndef _MIRF_H_
#define _MIRF_H_

#include "arduino_simple.h"
#include <string.h>
#include "spilib.h"
#include "Mirf_nRF24L01.h"

// Nrf24l settings

#define mirf_ADDR_LEN	5
const char mirf_ADDR[] = "honza";
const uint16_t MULTICAST_ADDR = 0xffff;

#define MAX_RX_PACKET_QUEUE 10
#define MAX_TX_PACKET_QUEUE 10

#define NOP __asm__("nop\n\t");

typedef struct {
  uint16_t txAddr;
  uint16_t rxAddr;
  uint8_t  type;
  uint8_t  counter;
  uint8_t  payload[10];
} mirfPacket;

typedef struct {
  uint16_t addr;
  uint8_t  counter;
} ackRecord;

typedef enum {
  READY = 0,
  IN_FIFO = 1,
  WAIT_FREE_AIR = 2,
  WAIT_ACK = 3,
  TIMEOUT = 4
} SENDING_STATUS;

typedef enum {
  RESERVED = 0,
  ACK = 1,
  ACK_RESP = 2,
  REQUEST = 3,
  RESPONSE = 4,
  PRESENTATION = 5,
  AUTOSEND = 6,
  RETRANSLATION = 7,
  CONFIGURATION = 8
} PACKET_TYPE;

typedef enum {
  ON_OFF_OUTPUT = 0,
  TEMP = 1,
  DOOR_SWITCH = 2
} SENSOR_TYPE;
//payload related structures

typedef enum {
  READ = 0,
  WRITE = 1,
  WRITE_FIRST = 2,
  WRITE_MIDDLE = 3,
  WRITE_LAST = 4
} CMD_TYPE;

//for type REQUEST
typedef struct {
  uint8_t cmd; //command to be executed by receiving node
  uint8_t len;
  uint8_t for_sensor;
  uint8_t payload[7];
} payloadRequestStruct;


//for type RESPONSE (kupr odpoved na command READ)
typedef struct {
  uint8_t cmd; //executed command by receiving node (same as in request packet)
  uint8_t len;
  uint8_t from_sensor;
  uint8_t payload[7];
} payloadResponseStruct;

//for presentation packet
typedef struct {
  uint8_t num_sensors;
  uint8_t sensor_type[9];
} payloadPresentationStruct;

//==========================================================================
class Nrf24l {
	public:
    Nrf24l();

	void init();
	void config();
	//void send(uint8_t *value);
	void setADDR(void);
    void setDevAddr(uint16_t);
	bool dataReady();
	bool isSending();
	bool rxFifoEmpty();
	bool txFifoEmpty();
	void getData(uint8_t * data);
    uint8_t getCarrier();
	uint8_t getStatus();

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

	void csnHi();
	void csnLow();
	void ceHi();
	void ceLow();
    
    //-----------------------------------------------------------------------
    //------------------ Variables ------------------------------------------
    
		/**
		 * In sending mode.
		 */
		volatile uint8_t PTX;

		/**
		 * CE Pin controls RX / TX, default 8.
		 */
		uint8_t cePin;

		/**
		 * CSN Pin Chip Select Not, default 7.
		 */
		uint8_t csnPin;

		/**
		 * Channel 0 - 127 or 0 - 84 in the US.
		 */
		uint8_t channel;

		/**
		 * Payload width in bytes default 16 max 32.
		 */
		uint8_t payload;

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
    	uint16_t devAddr;

		/**
		 * Spi interface (must extend spi).
		 */
		SPIlib *spi;
		//MirfSpiDriver *spi; //MirfSpiDriver

    //-------------------------------------------------------------------------
    mirfPacket pendingPacket;
    mirfPacket *outPacket;
    mirfPacket ackPacket;

    volatile mirfPacket *rxQueue[MAX_RX_PACKET_QUEUE];
    volatile mirfPacket *txQueue[MAX_TX_PACKET_QUEUE];
    volatile uint8_t txNum, rxNum;
    
    // sign that there is received packet in buffer adressed to this device
	// 0 means no packets
	// 1..x means number of packets ready
    volatile uint8_t inPacketReady;
    
    // sign that there is sending in progress
    //0 means - no packet to be sent (ready)
    //1 means - packet in fifo, waiting for Tx
    //2 means unsent - waiting for free air
    //3 means packet sent, but waiting for ACK
    //4 means timeout
    volatile uint8_t sendingStatus;
    
    //incremented with every new sent packet (used for identification of ACKs)
    volatile uint8_t packetCounter;
    
    //increments with every call of periodic Rx and Tx handle loop functions
    //dedicated to recognize timeouts in waiting for ACK
    //and to measure random time between send attempts
    volatile uint8_t Timer;
    volatile uint8_t TimerDelayed;    
    volatile uint8_t ackTimeout; //timer state when timeout will occur
    
    //whether there is to be sent some ack
    volatile bool sendAck;
    
};

extern Nrf24l Mirf;

#endif /* _MIRF_H_ */
