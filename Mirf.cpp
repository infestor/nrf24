#include <string.h>
#include <stdlib.h>
#include "Mirf_nRF24L01.h"
#include "Mirf.h"

Nrf24l Mirf = Nrf24l();

//should be run within some timered loop (really often - 10ms)
void Nrf24l::handleRxLoop(void)
{   
  Timer++; //every time we must increment timer
  uint8_t innerCounter = 0;
  
  //if (inPacketReady == MAX_RX_PACKET_QUEUE) return;
  //if there is full queue, return a wait for next turn
   while ( dataReady() ) //while something in rx buffer
   {
	  getData( (uint8_t*) &pendingPacket);
      
	  if ( (pendingPacket.rxAddr == devAddr) || (pendingPacket.rxAddr == MULTICAST_ADDR) )
	  { //is the packet for this device? or multicast
		if ( (sendingStatus == WAIT_ACK) )
		//ERROR: normal response packet does not copy COUNTER value from its request packet
		//so here counters cannot be compared
		//this is just workaround for a while. I will figure it out
		//but it will probably need adding some ,,needAck" flag in packets
		//&& (txQueue[txPosBeg].counter == pendingPacket.counter) )
		{ //ack(some response) for sent packet received
		sendingStatus = READY;
		sendResult = SUCCESS;
		removePacketfromTxQueue();
		//remove sent packet from queue
		//and add confirmation to confirmed queue
		//addConfirmedPacket(pendingPacket.counter);

		//store received response if queue is not full (and the packet is not just ACK)
		}
		else if (pendingPacket.type == PING)
		{
		  createAck((mirfPacket*)&pendingPacket);
		  continue;
		}

		if (pendingPacket.type != ACK)
		{
			if (inPacketReady != MAX_RX_PACKET_QUEUE)
			{
			  //last_addr_in = pendingPacket.txAddr;
			  //last_packetCounter_in = pendingPacket.counter;

			  memcpy((void*)&rxQueue[rxPosEnd], (mirfPacket*)&pendingPacket, NRF_PAYLOAD_SIZE);
			  inPacketReady++;
			  rxPosEnd++;
			  if (rxPosEnd == MAX_RX_PACKET_QUEUE) rxPosEnd = 0; //queue counted from 0, so on the max  number we are already out of array bounds
			  //createAck((mirfPacket*)&pendingPacket);
			}
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
	uint8_t *pPaket;
	uint8_t whatToSend = 0; //0 = nothing, 1 = ack, 2 = user packet

    //if there is some ack waiting
	  if (1) //(Timer == TimerNewAttempt)
	  {
		if ( (ackQueueSize > 0) )
		{
			pPaket = (uint8_t *)&ackQueue[ackPosBeg];
			whatToSend = 1;
		}
		else
		{
		   if ( ((SENDING_STATUS)sendingStatus == IN_FIFO) || (sendingStatus == WAIT_FREE_AIR) ) //new packet in fifo waiting to be sent
		   {
			   pPaket = (uint8_t *)&txQueue[txPosBeg];
			   whatToSend = 2;
		   }
		}
	  }

	//this is for sending user packet
	if ( whatToSend > 0 ) //new packet in fifo waiting to be sent
    {      

      flushTx();
      //write user packet to fifo       
      nrfSpiWrite(W_TX_PAYLOAD, pPaket, false, NRF_PAYLOAD_SIZE);  
      
      if ( 1 ) //getCarrier()==0 ) //no carrier detected (free air/free to send)
      {
        powerUpTx();       // Set to transmitter mode , Power up
      	ceHi();                     // Start transmission
                
      	if ( whatToSend == 1) //remove packet if it was ack
      	{
      		removePacketfromAckQueue();
      	}
        else  //if it was not ack packet, then it was for sure user packet
      	 //if ( (whatToSend >> 1) == 1) //if there was also user packet
        {
        	//when sending some REQUEST we will wait for answer packet
        	//else if this packet itself is response, we will not wait for anything and set
        	//seccesfull sending right away now
        	if ( (((mirfPacket*)pPaket)->type == REQUEST) || \
        		 (((mirfPacket*)pPaket)->type == PRESENTATION_REQUEST) )
        	{
        		sendingStatus = WAIT_ACK;
        		ackTimeoutTimer = Timer + MAX_ACK_WAIT_TIME;
        	}
        	else
        	{
        		sendingStatus = READY;
        		sendResult = SUCCESS;
        		//remove packet from queue - we can because it was already sent to NRF FIFO
        		removePacketfromTxQueue();
        	}
      	}

      	//powerUpRx(); //when 2 or more packets, we have to wait until fifo is empty (while isSending() )       
      }
      /*
      //NOT USED NOW (no carrier detection during sending)
      else  //there is someone already transmitting, wait random time to next try
      {
        TimerNewAttempt = Timer + 1 + (Timer & 7); //little bit randomize (increment also with lowest 2 bits of timer)
		#ifdef _DEBUG_
        UDR0 = 46; //DEBUG  .
		#endif
        if (whatToSend == 2)
      	{
      		txAttempt++;
      		sendingStatus = WAIT_FREE_AIR;
      	}
      }
      */
    }

    /*
    //NOT USED NOW (no carrier detection during sending)
	//only finite number of attempts to send (when air is full)
    if ((txAttempt == MAX_TX_ATTEMPTS) && (sendingStatus == WAIT_FREE_AIR) ) {
        sendingStatus = 0; //must be set to 0 to be able to send another packets
        sendResult = MAX_ATTEMPTS;
        removePacketfromTxQueue();
		#ifdef _DEBUG_
        UDR0 = 88; //DEBUG X
		#endif
    }
    */
    
    //check if there was TIMEOUT in waiting for ACK
    if ( (SENDING_STATUS)sendingStatus == WAIT_ACK) //check whether timeout waiting for ack occured
    {
       if (Timer == ackTimeoutTimer)
       {
    	 sendingStatus = READY; //must be set to 0 to be able to send another packets
         sendResult = TIMEOUT;
         removePacketfromTxQueue();
		 #ifdef _DEBUG_
         UDR0 = 94; //DEBUG sipka nahoru
		 #endif
       }
    }

    while ( isSending() ) NOP_ASM //wait for end of transfer and immediately start RX mode
}

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
void Nrf24l::readPacket(mirfPacket* paket)
{
  if (inPacketReady) {
	//we have to temporarily disable timer0 interrupt because no one else could be able to touch
	//the packet queue until we are finished
  	cli();
    memcpy(paket, (const void*)&rxQueue[rxPosBeg], NRF_PAYLOAD_SIZE);
    inPacketReady--;
    rxPosBeg++;
    if (rxPosBeg == MAX_RX_PACKET_QUEUE) rxPosBeg = 0; //rxPos could wrap.. and we are going anti clockwise
    sei();
  }
}

uint8_t Nrf24l::sendPacket(mirfPacket* paket)
{
  //another sending in progress, busy -> fail
  if ( (sendingStatus != READY) )
  {
     return 0;
  }

  if (txQueueSize == MAX_TX_PACKET_QUEUE) return 0;

  //set all parameters in packet
  cli();
  packetCounter++;
  if (packetCounter == 0) packetCounter++; //counter cannot be 0
  paket->counter = packetCounter;
  paket->txAddr = devAddr;

  memcpy((void*)&txQueue[txPosEnd], paket, NRF_PAYLOAD_SIZE);

  txQueueSize++;
  txPosEnd++;
  if (txPosEnd == MAX_TX_PACKET_QUEUE) txPosEnd = 0; //queue counted from 0, so on the max  number we are already out of array bounds
  sendingStatus = IN_FIFO; //set sign that there is sending packet pending
  sendResult = PROCESSING;
  txAttempt = 1;
  sei();
  
  return packetCounter;
}

void Nrf24l::createAck(mirfPacket* paket)
{
  if (ackQueueSize == MAX_ACK_PACKET_QUEUE) return;

  #ifdef _DEBUG_
  //UDR0 = 35; //DEBUG #
  #endif
  
  ackQueue[ackPosEnd].txAddr = paket->rxAddr;
  ackQueue[ackPosEnd].rxAddr = paket->txAddr;
  ackQueue[ackPosEnd].counter = paket->counter;
  ackQueue[ackPosEnd].type = ACK;

  ackQueueSize++;
  ackPosEnd++;
  if (ackPosEnd == MAX_ACK_PACKET_QUEUE) ackPosEnd = 0; //queue counted from 0, so on the max  number we are already out of array bounds
}

void Nrf24l::removePacketfromTxQueue(void)
{
    txQueueSize--;
    txPosBeg++;
    if (txPosBeg == MAX_TX_PACKET_QUEUE) txPosBeg = 0; //rxPos could wrap.. and we are going anti clockwise
}

inline void Nrf24l::removePacketfromAckQueue(void)
{
    ackQueueSize--;
    ackPosBeg++;
    if (ackPosBeg == MAX_ACK_PACKET_QUEUE) ackPosBeg = 0; //rxPos could wrap.. and we are going anti clockwise
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
Nrf24l::Nrf24l() {
	//cePin = D9;
	//csnPin = D10;
	spi = &SPI;
	baseConfig = _BV(EN_CRC) & ~_BV(CRCO) & _BV(MASK_TX_DS) & _BV(MASK_MAX_RT);
	packetCounter = 0;
	devAddr = 0;
	channel = DEFAULT_RF_CHANNEL;
}

void Nrf24l::init()
// Initializes pins to communicate with the MiRF module
// Should be called in the early initializing phase at startup.
{
	//pinMode(cePin, OUTPUT);
	//pinMode(csnPin, OUTPUT);
	DDRB |= 0b00000110;
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
  configRegister(SETUP_AW, mirf_ADDR_LEN - 2); //hw address width - mirf_ADDR_LEN bytes
  //here -2 is for normalising the value (1=3 bytes, 2=4bytes, 3=5bytes; so
  configRegister(SETUP_RETR, 0); //auto retransmission off
  setRfChannel(DEFAULT_RF_CHANNEL);  // Set RF channel
  configRegister(RF_SETUP,  0b00100111 ); //250kbit, 0dbm, max gain
  configRegister(FEATURE, 0); //dynamic length disabled(1<<EN_DPL) )
  configRegister(DYNPD, 0);
	// Set length of incoming payload 
	configRegister(RX_PW_P0, NRF_PAYLOAD_SIZE);
	configRegister(RX_PW_P1, NRF_PAYLOAD_SIZE);
  setADDR();
  
	flushRx();
}

inline void Nrf24l::setRfChannel(uint8_t new_channel)
{
	channel = new_channel;
	configRegister(RF_CH, new_channel);  // Set RF channel
}

inline uint8_t Nrf24l::getRfChannel(void)
{
	return channel;
}

void Nrf24l::setDevAddr(ADDR_TYPE addr)
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
	writeRegister(TX_ADDR, (uint8_t*)mirf_ADDR, mirf_ADDR_LEN);  
	ceHi();
} 

bool Nrf24l::dataReady() 
// Checks if data is available for reading
{
	// See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = getStatus();

	// We can short circuit on RX_DR, but if it's not set, we still need
	// to check the FIFO for any pending packets
	if (status & _BV(RX_DR))
		configRegister(STATUS, _BV(RX_DR)); //clear RX interrupt flag
		return 1;

	return !rxFifoEmpty();
}

bool Nrf24l::rxFifoEmpty(){
	uint8_t fifoStatus;

	nrfSpiWrite2((R_REGISTER | (REGISTER_MASK & FIFO_STATUS)), &fifoStatus, true, 1);
	//readRegister(FIFO_STATUS, &fifoStatus, sizeof(fifoStatus));

	return (fifoStatus & _BV(RX_EMPTY));
}

void Nrf24l::getData(uint8_t * data) 
// Reads payload bytes into data array
{
	nrfSpiWrite2(R_RX_PAYLOAD, data, true, NRF_PAYLOAD_SIZE); // Read payload

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
    nrfSpiWrite2((R_REGISTER | (REGISTER_MASK & reg)), value, true, len);
    //nrfSpiWrite(255, value, true, len);
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
volatile bool Nrf24l::isSending() {
	uint8_t status;
	if(PTX){
		nrfSpiWrite2((R_REGISTER | (REGISTER_MASK & FIFO_STATUS)), &status, true, 1);
	    	
		/*
		 *  if sending successful (TX_DS) or max retries exceded (MAX_RT).
		 */

		if( (status & (1 << TX_EMPTY)) ){
			powerUpRx();
			return false; 
		}

		return true;
	}
	return false;
}
// volatile bool Nrf24l::isSending() {
// 	uint8_t status;
// 	if(PTX){
// 		status = getStatus();
// 	    	
// 		/*
// 		 *  if sending successful (TX_DS) or max retries exceded (MAX_RT).
// 		 */
// 
// 		if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
// 			powerUpRx();
// 			return false;                                                                                                    
// 		}
// 
// 		return true;
// 	}
// 	return false;
// }

uint8_t Nrf24l::getCarrier(){
//returns true if carrier detected in RX mode
//used before sending packet to determine if air is clear
	/* Initialize with NOP so we get the first byte read back. */
	uint8_t rv = NOP_CMD;
	nrfSpiWrite2((R_REGISTER | (REGISTER_MASK & CD)), &rv, true, 1);
  //readRegister(CD, &rv, 1);
	return (rv & 1);
}

uint8_t Nrf24l::getStatus(){
	/* Initialize with NOP so we get the first byte read back. */
	uint8_t rv = NOP_CMD;
	nrfSpiWrite((R_REGISTER | (REGISTER_MASK & STATUS)), &rv, true, 1);
  //readRegister(STATUS, &rv, 1);
	return rv;
}

inline void Nrf24l::flushTx() {
	nrfSpiWrite(FLUSH_TX);
	//configRegister(STATUS, _BV(TX_DS));
}

void Nrf24l::powerUpRx() {
	PTX = 0;
	ceLow();

	configRegister(CONFIG, baseConfig | _BV(PWR_UP) | _BV(PRIM_RX));
	configRegister(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)); 
	#ifdef _DEBUG_
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = 'R'; //DEBUG _
	#endif
	ceHi();
}

void Nrf24l::flushRx(){
	nrfSpiWrite(FLUSH_RX);
}

void Nrf24l::powerUpTx() {
	PTX = 1;
	ceLow();
	#ifdef _DEBUG_
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = 'T'; //DEBUG !
	#endif
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

void Nrf24l::nrfSpiWrite2(uint8_t reg, uint8_t *data, bool readData, uint8_t len) {  
  csnLow();

	spi->transfer(reg);
  spi->transfer(255);

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

void Nrf24l::ceHi(){    //PB1
	//digitalWrite(cePin,HIGH);
  PORTB |= (1<<1);
}

void Nrf24l::ceLow(){
	//digitalWrite(cePin,LOW);
  PORTB &= (~(1<<1));
}

void Nrf24l::csnHi(){  //PB2
	//digitalWrite(csnPin,HIGH);
  PORTB |= (1<<2);
}

void Nrf24l::csnLow(){
	//digitalWrite(csnPin,LOW);
  PORTB &= (~(1<<2));
}


void Nrf24l::powerDown(){
	ceLow();

	configRegister(CONFIG, baseConfig);

	flushRx();
	flushTx();
}
