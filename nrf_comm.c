#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include "Mirf.h"

uint8_t fire_mirf;
mirfPacket inPacket;
mirfPacket outPacket;
uint8_t pinState;

#define DEV_ADDR 2
#define NUM_SENSOR 1

//======================================================
ISR(TIMER0_COMPA_vect) {
  fire_mirf = 1;
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
 ((payloadPresentationStruct *)(&outPacket.payload))->num_sensors = NUM_SENSOR; //this and the simmilar down could not be together due aliasing
 outPacket.txAddr = DEV_ADDR; //but this should be filled during sending packet
 outPacket.rxAddr = 1;
 outPacket.type = (PACKET_TYPE)PRESENTATION;
 ((payloadPresentationStruct *)(&outPacket.payload))->sensor_type[0] = ON_OFF_OUTPUT;
 Mirf.sendPacket(&outPacket);


 while(1) {
   if(fire_mirf) {
  	Mirf.handleRxLoop();
  	Mirf.handleTxLoop();
	fire_mirf = 0;
   }

   //zpracovat prichozi packet
   if (Mirf.inPacketReady)
   {
     Mirf.readPacket(&inPacket);
     if ( (PACKET_TYPE)inPacket.type == (PACKET_TYPE)REQUEST )
	 {
	    payloadRequestStruct *pay = (payloadRequestStruct*)&inPacket.payload;
		if (pay->for_sensor == NUM_SENSOR)
		{  
		  if (pay->cmd == WRITE)
		  {
		     if (pay->payload[0] > 0) pinState = HIGH; else pinState = LOW;
			 digitalWrite(13, pinState);
		  }
		  else if (pay->cmd == READ)
		  {
		     outPacket.type = (PACKET_TYPE)RESPONSE; 
			 payloadResponseStruct *res = (payloadResponseStruct*)&outPacket.payload;
			 res->cmd = pay->cmd;
			 res->len = 1;
			 res->from_sensor = pay->for_sensor;
			 res->payload[0] = pinState;
			 Mirf.sendPacket(&outPacket);
		  }
        }
	 }
   }
 }

 return 0;
}
