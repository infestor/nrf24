#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>
#include "Mirf.h"

mirfPacket volatile inPacket;
mirfPacket volatile outPacket;
uint8_t volatile pinState;
uint16_t volatile adcVal;
uint8_t sendResult;

#define DEV_ADDR 2
#define NUM_SENSORS 2
#define SWITCHED_PIN 9

//======================================================
ISR(TIMER0_COMPA_vect) {
  	Mirf.handleRxLoop();
  	Mirf.handleTxLoop();
}

ISR(BADISR_vect) { //just for case
  __asm__("nop\n\t");
}

ISR(ADC_vect) {
	SMCR = 0; //disable sleep and enable normal Idle mode
}

void getAdcVal(void)
{
	  SMCR = 2; //enable ADC noise reduct sleep mode

	  ADCSRA |= _BV(ADSC);  // Start the ADC
	  sleep_enable();
	  sleep_cpu();

	  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
	  adcVal = ADCW;
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

  //set ADC to read temp from internal sensor, 1.1V reference, prescaler 128
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= (_BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) );  // enable the ADC

  //led13 as output
  pinMode(SWITCHED_PIN, OUTPUT);
  pinState = HIGH;
  digitalWrite(SWITCHED_PIN, pinState);

  //disable unused peripherials
  PRR = ( _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2) ) ;
  
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
 ((payloadPresentationStruct *)&outPacket.payload)->num_sensors = 2;
 ((payloadPresentationStruct *)&outPacket.payload)->sensor_type[0] = TEMP;
 ((payloadPresentationStruct *)&outPacket.payload)->sensor_type[1] = ON_OFF_OUTPUT;

 //send the presentation packet. Try it until ACK is received
 //while( Mirf.sendPacket((mirfPacket*)&outPacket) != 0) NOP_ASM


 while(1) {
   //zpracovat prichozi packet
   if (Mirf.inPacketReady)
   {
     Mirf.readPacket((mirfPacket*)&inPacket);
     if ( (PACKET_TYPE)inPacket.type == REQUEST )
	 {
	    payloadRequestStruct *req = (payloadRequestStruct*)&inPacket.payload;
		outPacket.type = RESPONSE;
		outPacket.rxAddr = inPacket.txAddr;
		payloadResponseStruct *res = (payloadResponseStruct*)&outPacket.payload;

	    if (req->for_sensor == 0) //temp sensor
		{
			 res->cmd = req->cmd;
			 res->len = 2;
			 res->from_sensor = req->for_sensor;
			 res->payload[0] = pinState;
			 sendResult = Mirf.sendPacket((mirfPacket*)&outPacket);
		}
		else if (req->for_sensor == 1) //door switch
		{  
		  if (req->cmd == WRITE)
		  {
		     if (req->payload[0] > 0) pinState = HIGH; else pinState = LOW;
			 digitalWrite(SWITCHED_PIN, pinState);
		  }
		  else if (req->cmd == READ)
		  {
			 res->cmd = req->cmd;
			 res->len = 1;
			 res->from_sensor = req->for_sensor;
			 res->payload[0] = pinState;
			 sendResult = Mirf.sendPacket((mirfPacket*)&outPacket);
		  }
        }
	 }
   }
   else //if there is no packet to be processed, we can enter idle mode to save some power
   {
	  sleep_enable();
	  sleep_cpu();
	  sleep_disable();
   }
 }

 return 0;
}
