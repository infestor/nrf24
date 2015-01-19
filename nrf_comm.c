#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include "Mirf.h"

mirfPacket volatile inPacket;
mirfPacket volatile outPacket;
uint8_t volatile pinState;

#define DEV_ADDR 2
#define NUM_SENSOR 1

//======================================================
ISR(TIMER0_COMPA_vect) {
  	Mirf.handleRxLoop();
  	Mirf.handleTxLoop();
}

ISR(BADISR_vect) { //just for case
  __asm__("nop\n\t");
}

void setup()
{
  Mirf.init();
  Mirf.config();
  Mirf.setDevAddr(DEV_ADDR);
  
  //timer0 10ms period, interrupt enable
  //prescaler 1024, count to 156
  OCR0A = 156;
  OCR0B = 170;
  TCCR0A = 2;
  TCCR0B = 5;
  TIMSK0 = 2;

  //led13 as output
  pinMode(13, OUTPUT);
  pinState = HIGH;
  digitalWrite(13, pinState);
  
  sei();
}

int main(void)
{
 wdt_disable();

 setup();

 //nejak poslat PRESENTATION paket
 outPacket.txAddr = DEV_ADDR; //but this should be filled during sending packet
 outPacket.rxAddr = 1;
 outPacket.type = (PACKET_TYPE)PRESENTATION;
 ((payloadPresentationStruct *)&outPacket.payload)->num_sensors = NUM_SENSOR;
 ((payloadPresentationStruct *)&outPacket.payload)->sensor_type[0] = ON_OFF_OUTPUT;


 //send the presentation packet. Try it until ACK is received
 while( Mirf.sendPacket((mirfPacket*)&outPacket) != 0) NOP_ASM


 while(1) {
   //zpracovat prichozi packet
   if (Mirf.inPacketReady)
   {
     Mirf.readPacket((mirfPacket*)&inPacket);
     if ( (PACKET_TYPE)inPacket.type == REQUEST )
	 {
	    payloadRequestStruct *req = (payloadRequestStruct*)&inPacket.payload;
		if (req->for_sensor == NUM_SENSOR)
		{  
		  if (req->cmd == WRITE)
		  {
		     if (req->payload[0] > 0) pinState = HIGH; else pinState = LOW;
			 digitalWrite(13, pinState);
		  }
		  else if (req->cmd == READ)
		  {
		     outPacket.type = RESPONSE;
		     outPacket.rxAddr = inPacket.txAddr;
			 payloadResponseStruct *res = (payloadResponseStruct*)&outPacket.payload;
			 res->cmd = req->cmd;
			 res->len = 1;
			 res->from_sensor = req->for_sensor;
			 res->payload[0] = pinState;
			 Mirf.sendPacket((mirfPacket*)&outPacket);
		  }
        }
	 }
   }
 }

 return 0;
}
