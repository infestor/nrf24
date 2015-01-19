#include <string.h>
#include <stdlib.h>
#include "Mirf_nRF24L01.h"
#include "Mirf.h"

// Defines for setting the MiRF registers for transmitting or receiving mode


Nrf24l Mirf = Nrf24l();

void Nrf24l::removePacketfromTxQueue(void)
{
    free((void *)txQueue[txPosBeg]);  //free the packet from queue
    txQueueSize--;
    txPosBeg++;
    if (txPosBeg == MAX_TX_PACKET_QUEUE) txPosBeg = 0; //rxPos could wrap.. and we are going anti clockwise
}

//should be run within some timered loop (really often - 50ms)
void Nrf24l::handleRxLoop(void)
{   
  Timer++; //every time we must increment timer
  uint8_t innerCounter = 0;
  
  //if (inPacketReady == MAX_RX_PACKET_QUEUE) return;
  //if there is full queue, return a wait for next turn
   while ( (dataReady()) && (inPacketReady != MAX_RX_PACKET_QUEUE) ) //while something in rx buffer and queue not full
   {
	  getData( (uint8_t*) &pendingPacket);
      
      if ( (pendingPacket.rxAddr == devAddr) || (pendingPacket.rxAddr == MULTICAST_ADDR) )
      { //is the packet for this device? or multicast
        if ( ((PACKET_TYPE)pendingPacket.type == ACK) || ((PACKET_TYPE)pendingPacket.type == ACK_RESP) )
        {
          //TODO: handle ACK_RESP packet payload.. propably just give it to app as it is
          if ( ((SENDING_STATUS)sendingStatus == WAIT_ACK) && (packetCounter == pendingPacket.counter) )
          {
            sendingStatus = 0;
            removePacketfromTxQueue();
            //remove sent packet from queue
          }
        }
        else 
        { //other packets are saved to queue and app has to hadle them
          mirfPacket *newPacket;
          newPacket = (mirfPacket *)malloc(payload);
          memcpy(newPacket, &pendingPacket, payload);
          inPacketReady++;
          rxPosEnd++;
          if (rxPosEnd == MAX_RX_PACKET_QUEUE) rxPosEnd = 0; //queue counted from 0, so on the max  number we are already out of array bounds
          rxQueue[rxPosEnd] = newPacket;
          newPacket = NULL;
          createAck(newPacket);
        }
      }
      
      //we have to have some theoretical limit staying in this function
      //because if there were too much incoming packets all the time,
      //then this function would never end
      innerCounter++;
      if (innerCounter == 5) break;
   }
}

void Nrf24l::handleTxLoop(void) //probably should be run from main program loop, because can be time consuming in wait for finished tx
{

    //bool onlyAck = (sendAck && (sendingStatus==0)
	//TODO: separate normal and ack packets. Now the queue in tx handle function does not work properly

	//this is for sending user packet
	if ( (sendingStatus == IN_FIFO) || ((sendingStatus == WAIT_FREE_AIR) && (Timer == TimerNewAttempt)) ) //new packet in fifo waiting to be sent
    {

      if ( !getCarrier() ) //no carrier detected (free air/free to send)
      {
	    ceLow();
        flushTx();  
        //write packet to fifo
        nrfSpiWrite(W_TX_PAYLOAD, (uint8_t*)&txQueue[txPosBeg], false, payload);
        //we will left the packet in the queue array until timeout occures or ack is received

        powerUpTx();       // Set to transmitter mode , Power up
      	ceHi();                     // Start transmission
      	//ceLow(); //when 2 or more packets, we have to wait until fifo is empty (while isSending() )
        while ( isSending() ) NOP_ASM //wait for end of transfer and immediately start RX mode
        sendingStatus = WAIT_ACK;
        //sendAck = false;
        ackTimeoutTimer = Timer + 10;
      }
      else  //there is someone already transmitting, wait random time to next try
      {
        txAttempt++;
        sendingStatus = WAIT_FREE_AIR;        
        TimerNewAttempt = Timer + 1 + (Timer & 252); //little bit randomize (increment also with lowest 2 bits of timer)
      }
    }

	//only finite number of attempts to send (when air is full)
    if (txAttempt == 4) {
        sendingStatus = TIMEOUT;
        removePacketfromTxQueue();
    }
    
    //check if there was TIMEOUT in waiting for ACK
    if ( (SENDING_STATUS)sendingStatus == WAIT_ACK) //check whether timeout waiting for ack occured
    {
       if (Timer == ackTimeoutTimer)
       {
         sendingStatus = TIMEOUT;
         removePacketfromTxQueue();
       }
    }
}


void Nrf24l::readPacket(mirfPacket* paket)
{
  if (inPacketReady) {
	//we have to temporarily disable timer0 interrupt because no one else could be able to touch
	//the packet queue until we are finished
	cli();
    memcpy(paket, (const void*)rxQueue[rxPosBeg], payload);
    free((void*)rxQueue[rxPosBeg]);  //free the packet from queue
    inPacketReady--;
    rxPosBeg++;
    if (rxPosBeg == MAX_RX_PACKET_QUEUE) rxPosBeg = 0; //rxPos could wrap.. and we are going anti clockwise
    sei();
  }
}

uint8_t Nrf24l::sendPacket(mirfPacket* paket)
{
  //another sending in progress, busy -> fail
  if ( (sendingStatus > 0) || (isSending() )  )
  {
     return 99;
  }

  if (txQueueSize == MAX_TX_PACKET_QUEUE) return 98;

  //set all parameters in packet
  packetCounter++;
  paket->counter = packetCounter;
  paket->txAddr = devAddr;

  mirfPacket *newPacket;
  newPacket = (mirfPacket*)malloc(payload);
  if (newPacket == NULL) {return 98;}
  memcpy(newPacket, paket, payload);

  cli();
  txQueueSize++;
  txPosEnd++;
  if (txPosEnd == MAX_TX_PACKET_QUEUE) txPosEnd = 0; //queue counted from 0, so on the max  number we are already out of array bounds
  txQueue[txPosEnd] = newPacket; //save pointer to packet
  sendingStatus = 1; //set sign that there is sending packet pending
  txAttempt = 1;
  sei();
  
  while ( !( ((SENDING_STATUS)sendingStatus == READY) || ((SENDING_STATUS)sendingStatus == TIMEOUT)) ) //success or timeout
  {
    NOP_ASM
  } 
  
  uint8_t stat = sendingStatus;
  sendingStatus = 0; //we must reset status to ready(in case that it was TIMEOUT), to be able work out next packets
  return stat;
}

void Nrf24l::createAck(mirfPacket* paket)
{
  if (txQueueSize == MAX_TX_PACKET_QUEUE) return;

  //cli();
  mirfPacket *ackPacket;
  ackPacket = (mirfPacket*)malloc(payload);
  if (ackPacket == NULL) {
	  //sei();
	  return;
  }

  ackPacket->txAddr = paket->rxAddr;
  ackPacket->rxAddr = paket->txAddr;
  ackPacket->counter = paket->counter;
  ackPacket->type = ACK;

  txQueueSize++;
  txPosEnd++;
  if (txPosEnd == MAX_TX_PACKET_QUEUE) txPosEnd = 0; //queue counted from 0, so on the max  number we are already out of array bounds
  txQueue[txPosEnd] = ackPacket;
  ackPacket = NULL;
  
  //sendAck = true;
  //sei();
}



//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
Nrf24l::Nrf24l() {
	cePin = 8;
	csnPin = 7;
	channel = 70;
    payload = sizeof(mirfPacket);
	spi = &SPI;
	baseConfig = _BV(EN_CRC) & ~_BV(CRCO);
}

void Nrf24l::init()
// Initializes pins to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
	pinMode(cePin, OUTPUT);
	pinMode(csnPin, OUTPUT);

	ceLow();
	csnHi();

	// Initialize spi module
	spi->begin();
}

void Nrf24l::config() 
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
// NB: channel and payload must be set now.
{
	
  configRegister(EN_AA, 0); //disable auto ack
  configRegister(EN_RXADDR, (1 << ERX_P0) ); //only pipe 0 receive enabled
  configRegister(SETUP_AW, 3); //hw address width - 5bytes
  configRegister(SETUP_RETR, 0); //auto retransmission off
	configRegister(RF_CH, channel);  // Set RF channel
  configRegister(RF_SETUP,  7 );  //1mbit, 0dbm, max gain
  configRegister(FEATURE, 0); //dynamic length disabled(1<<EN_DPL) )
	// Set length of incoming payload 
	configRegister(RX_PW_P0, payload);
	configRegister(RX_PW_P1, payload);
  setADDR();
  
	// Start receiver 
	powerUpRx();
	flushRx();
}

void Nrf24l::setDevAddr(uint16_t addr)
// sets the unique node address
// is used during decoding of incoming packets (only packets for this address are handled)
// when transmitting packet, this address is used as SENDER adress
{
  devAddr = addr;
}

void Nrf24l::setADDR(void)
//sets address for RX and TX in NRF module (both the same)
{
	ceLow();
	writeRegister(RX_ADDR_P0, (uint8_t*)mirf_ADDR, mirf_ADDR_LEN);
	ceHi();
	writeRegister(TX_ADDR, (uint8_t*)mirf_ADDR, mirf_ADDR_LEN);  
} 

// void Nrf24l::setRADDR(uint8_t * adr) 
// // Sets the receiving address
// {
// 	ceLow();
// 	writeRegister(RX_ADDR_P0, adr, mirf_ADDR_LEN);
// 	ceHi();
// }
// 
// void Nrf24l::setTADDR(uint8_t * adr)
// // Sets the transmitting address
// {
// 	/*
// 	 * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
// 	 */
// 	writeRegister(RX_ADDR_P0,adr, mirf_ADDR_LEN);
// 	writeRegister(TX_ADDR,adr, mirf_ADDR_LEN);
// }

bool Nrf24l::dataReady() 
// Checks if data is available for reading
{
	// See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = getStatus();

	// We can short circuit on RX_DR, but if it's not set, we still need
	// to check the FIFO for any pending packets
	if (status & _BV(RX_DR))
		return 1;

	return !rxFifoEmpty();
}

bool Nrf24l::rxFifoEmpty(){
	uint8_t fifoStatus;

	readRegister(FIFO_STATUS, &fifoStatus, sizeof(fifoStatus));

	return (fifoStatus & _BV(RX_EMPTY));
}

void Nrf24l::getData(uint8_t * data) 
// Reads payload bytes into data array
{
	nrfSpiWrite(R_RX_PAYLOAD, data, true, payload); // Read payload

	// NVI: per product spec, p 67, note c:
	//  "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
	//  for handling this interrupt should be: 1) read payload through SPI,
	//  2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more 
	//  payloads available in RX FIFO, 4) if there are more data in RX FIFO,
	//  repeat from step 1)."
	// So if we're going to clear RX_DR here, we need to check the RX FIFO
	// in the dataReady() function
	configRegister(STATUS, _BV(RX_DR));   // Reset status register
}

void Nrf24l::configRegister(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
	writeRegister(reg, &value, 1);
}

void Nrf24l::readRegister(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
    nrfSpiWrite((R_REGISTER | (REGISTER_MASK & reg)), value, true, len);
}

void Nrf24l::writeRegister(uint8_t reg, uint8_t * value, uint8_t len) 
// Writes an array of bytes into inte the MiRF registers.
{
	nrfSpiWrite((W_REGISTER | (REGISTER_MASK & reg)), value, false, len);
}

/**
 * isSending.
 *
 * Test if chip is still sending.
 * When sending has finished return chip to listening.
 *
 */
bool Nrf24l::isSending() {
	uint8_t status;
	if(PTX){
		status = getStatus();
	    	
		/*
		 *  if sending successful (TX_DS) or max retries exceded (MAX_RT).
		 */

		if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
			powerUpRx();
			return false; 
		}

		return true;
	}
	return false;
}

uint8_t Nrf24l::getCarrier(){
//returns true if carrier detected in RX mode
//used before sending packet to determine if air is clear
	/* Initialize with NOP so we get the first byte read back. */
	uint8_t rv = NOP_CMD;
	readRegister(CD, &rv, 1);
	return (rv );
}

uint8_t Nrf24l::getStatus(){
	/* Initialize with NOP so we get the first byte read back. */
	uint8_t rv = NOP_CMD;
	readRegister(STATUS,&rv,1);
	return rv;
}

void Nrf24l::flushTx() {
	nrfSpiWrite(FLUSH_TX);
}

void Nrf24l::powerUpRx() {
	PTX = 0;
	ceLow();

	configRegister(CONFIG, baseConfig | _BV(PWR_UP) | _BV(PRIM_RX));
	configRegister(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)); 

	ceHi();
}

void Nrf24l::flushRx(){
	nrfSpiWrite(FLUSH_RX);
}

void Nrf24l::powerUpTx() {
	PTX = 1;
	configRegister(CONFIG, baseConfig | (_BV(PWR_UP) & ~_BV(PRIM_RX)) );
}

void Nrf24l::nrfSpiWrite(uint8_t reg, uint8_t *data, bool readData, uint8_t len) {  
  csnLow();

	spi->transfer(reg);

	if (data) {
		uint8_t i;
		for(i = 0; i < len; ++i) {
			uint8_t readValue = spi->transfer(data[i]);

			if (readData) {
				data[i] = readValue;
			}
		}
	}

	csnHi();
}

void Nrf24l::ceHi(){
	digitalWrite(cePin,HIGH);
}

void Nrf24l::ceLow(){
	digitalWrite(cePin,LOW);
}

void Nrf24l::csnHi(){
	digitalWrite(csnPin,HIGH);
}

void Nrf24l::csnLow(){
	digitalWrite(csnPin,LOW);
}

void Nrf24l::powerDown(){
	ceLow();

	configRegister(CONFIG, baseConfig);

	flushRx();
	flushTx();
}

// void Nrf24l::send(uint8_t * value) 
// // Sends a data package to the default address. Be sure to send the correct
// // amount of bytes as configured as payload on the receiver.
// {
// 	uint8_t status;
// 	status = getStatus();
// 
// 	while (PTX) {
// 		status = getStatus();
// 
// 		if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
// 			PTX = 0;
// 			break;
// 		}
// 	}                  // Wait until last paket is send
// 
// 	ceLow();
// 
// 	powerUpTx();       // Set to transmitter mode , Power up
// 	flushTx();
// 
// 	nrfSpiWrite(W_TX_PAYLOAD, value, false, payload);   // Write payload
// 
// 	ceHi();                     // Start transmission
// 	ceLow();
// }
