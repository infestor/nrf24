#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>
#include <stdio.h>
#include <string.h>
#include "Mirf.h"
#include "Mirf_nRF24L01.h"

mirfPacket volatile inPacket;
mirfPacket volatile outPacket;
uint8_t sendResult;
uint16_t volatile citac;
char buff[35];

typedef union {
  uint16_t uint;
  struct {
    uint8_t lsb;
    uint8_t msb;
  };
} IntUnion;

IntUnion volatile adcVal;

#define DEV_ADDR 1

//======================================================
ISR(TIMER0_COMPA_vect) {
  	Mirf.handleRxLoop();
  	Mirf.handleTxLoop();
    citac++;
}

ISR(BADISR_vect) { //just for case
  __asm__("nop\n\t");
}
//==========================================================

void USART_Transmit( char *data, uint8_t len )
{
  for (uint8_t i=0; i < len; i++)
  {
    /* Wait for empty transmit buffer */
    while ( !( UCSR0A & (1<<UDRE0)) );
    /* Put data into buffer, sends the data */
    UDR0 = data[i];
  }
}

#ifdef _DEBUG_
void debug(void)
{
  uint8_t i;
  uint8_t volatile rr;
  uint8_t adr[5];
  
  cli();
  for (i=0; i< 10; i++)
  {
    Mirf.readRegister(i, (uint8_t*)&rr, 1);
    sprintf(buff, "%d:%d,", i, rr);
    USART_Transmit(buff, strlen(buff) );
  }
  sprintf(buff, "\n");
  USART_Transmit(buff, strlen(buff) );
  
  Mirf.readRegister(TX_ADDR, (uint8_t*)&adr, 5);
  sprintf(buff, "TX: 0x%2x%2x%2x%2x%2x\n", adr[0], adr[1], adr[2], adr[3], adr[4]);
  USART_Transmit(buff, strlen(buff) );

  Mirf.readRegister(RX_ADDR_P0, (uint8_t*)&adr, 5);
  sprintf(buff, "RX: 0x%2x%2x%2x%2x%2x\n", adr[0], adr[1], adr[2], adr[3], adr[4]);
  USART_Transmit(buff, strlen(buff) );
  
  sei();
}
#endif

void setup()
{
  //configure uart0  (57600, 8bits, no parity, 1 stop bit)
  UBRR0H = 0;
  UBRR0L = 16;
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);

  //start radio    
  Mirf.init();
  Mirf.setDevAddr(DEV_ADDR);
  Mirf.config(); 

#ifdef _DEBUG_
  Mirf.csnLow();
  uint8_t a = Mirf.spi->transfer(R_REGISTER | ( REGISTER_MASK & 5 ));
  uint8_t b = Mirf.spi->transfer(255);
  uint8_t c = Mirf.spi->transfer(255);
  Mirf.csnHi();
  sprintf(buff, "po: %d, %d, %d\n", a, b, c); //DEBUG
  USART_Transmit(buff, strlen(buff) );
#endif

  //timer0 10ms period, interrupt enable
  //prescaler 1024, count to 156
  OCR0A = 156;
  OCR0B = 170;
  TCCR0A = 2;
  TCCR0B = 5;
  TIMSK0 = 2;

  //disable unused peripherials
  PRR = ( _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2) ) ;
  
  sei();
}

int main(void)
{
 wdt_disable();
 setup();

 memset((void*)&outPacket, 0, sizeof(mirfPacket) );
 outPacket.txAddr = DEV_ADDR; //but this should be filled during sending packet
 outPacket.rxAddr = 2;
 outPacket.type = (PACKET_TYPE)REQUEST;
 ((payloadRequestStruct *)&outPacket.payload)->cmd = READ;
 ((payloadRequestStruct *)&outPacket.payload)->len = 0;
 ((payloadRequestStruct *)&outPacket.payload)->for_sensor = 0;

 sprintf(buff, "COMM MASTER\n");
 USART_Transmit(buff, strlen(buff) );
      
 //------------------------------------
 while(1) {
   //zpracovat prichozi packet
   if (Mirf.inPacketReady)
   {
     Mirf.readPacket((mirfPacket*)&inPacket);
     //sprintf(buff, "RX:%d,TX:%d,T:%d,C:%d,P:%d\n", inPacket.rxAddr, inPacket.txAddr, inPacket.type, inPacket.counter, inPacket.payload[0]);
     //USART_Transmit(buff, strlen(buff) );
     
     if ( ((PACKET_TYPE)inPacket.type == RESPONSE) && (inPacket.txAddr == 2) )
	   {
		   payloadResponseStruct *res = (payloadResponseStruct*)&inPacket.payload;

	     if ( (res->from_sensor == 0) && (res->len==2) ) //temp sensor
		   {
			   adcVal.lsb = res->payload[0];
			   adcVal.msb = res->payload[1];
         //send the value through rs232
         double ff = ((adcVal.uint - 324.31 ) / 1.22);
         sprintf(buff, "\nT=%d, %f\n", adcVal.uint, ff);        
         USART_Transmit(buff, strlen(buff) );
	     }
	   }
   }
   else //if there is no packet to be processed,
   {
    if (citac > 500)  //time to send request
    {
      uint8_t last = Mirf.sendResult;
      if (Mirf.sendingStatus == 0) sendResult = Mirf.sendPacket((mirfPacket*)&outPacket); else sendResult = 99;
      sprintf(buff, "\nS:%d, R:%d ", sendResult, last);
      USART_Transmit(buff, strlen(buff) );
      citac = 0;
      //debug();
    }
    
    // we can enter idle mode to save some power
	  sleep_enable();
	  sleep_cpu();
	  sleep_disable();
   }
 }

 return 0;
}
