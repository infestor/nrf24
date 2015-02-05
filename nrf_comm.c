#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdio.h>
#include "Mirf.h"
#include "Mirf_nRF24L01.h"

#define DEV_ADDR 2
#define NUM_SENSORS 2
#define SWITCHED_PIN 9
#define SENSOR_0_CALIB_ADDR (uint8_t *)1

mirfPacket volatile inPacket;
mirfPacket volatile outPacket;
uint8_t volatile pinState;
char buff[40];
uint8_t internalTempCalib;
uint8_t internalTempCalibValid;
uint8_t sendResult;

typedef union {
  uint16_t uint;
  struct {
    uint8_t lsb;
    uint8_t msb;
  };
} IntUnion;

uint8_t volatile adcVal;

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
	  //adcVal = ADCW;
}

void setup()
{
  //configure uart0  (57600, 8bits, no parity, 1 stop bit)
  UBRR0H = 0;
  UBRR0L = 16;
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);
  
  //read internal temp sensor calibration byte from eeprom
  internalTempCalib = eeprom_read_byte(SENSOR_0_CALIB_ADDR);
  if (internalTempCalib == 0xFF) internalTempCalib = 128; else internalTempCalibValid = 1;

  //start Radio
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
  //pinMode(SWITCHED_PIN, OUTPUT);
  pinState = HIGH;
  //digitalWrite(SWITCHED_PIN, pinState);

  //disable unused peripherials
  PRR = ( _BV(PRTWI) | _BV(PRTIM1) | _BV(PRTIM2) ) ;
  
  sei();
}

int main(void)
{
 wdt_disable();

 setup();

 //nejak poslat PRESENTATION paket
 memset((void*)&outPacket, 0, sizeof(mirfPacket) );
 outPacket.txAddr = DEV_ADDR; //but this should be filled during sending packet
 outPacket.rxAddr = 1;
 outPacket.type = (PACKET_TYPE)PRESENTATION;
 ((payloadPresentationStruct *)&outPacket.payload)->num_sensors = 2;
 ((payloadPresentationStruct *)&outPacket.payload)->sensor_type[0] = TEMP;
 ((payloadPresentationStruct *)&outPacket.payload)->sensor_type[1] = ON_OFF_OUTPUT;

 //send the presentation packet. Try it until ACK is received
 //while( Mirf.sendPacket((mirfPacket*)&outPacket) != 0) NOP_ASM
 
 memset(buff, 0, sizeof(buff));
 
 //debug();
 while(1) {
   //zpracovat prichozi packet
   if (Mirf.inPacketReady)
   {
     Mirf.readPacket((mirfPacket*)&inPacket);
     
     sprintf((char*)buff, "\nin:  RX:%d,TX:%d,T:%d,C:%d\n", inPacket.rxAddr, inPacket.txAddr, inPacket.type, inPacket.counter);
     USART_Transmit((char*)buff, strlen((char*)buff) );
          
     if ( (PACKET_TYPE)inPacket.type == REQUEST )
	 {
	    payloadRequestStruct *req = (payloadRequestStruct*)&inPacket.payload;
		outPacket.type = RESPONSE;
		outPacket.rxAddr = inPacket.txAddr;
		payloadResponseStruct *res = (payloadResponseStruct*)&outPacket.payload;
		res->cmd = req->cmd;
  		res->from_sensor = req->for_sensor;

  	    if (req->for_sensor == 0) //==== internal temp sensor =====
  	    {
  	    	if (req->cmd == READ)
  	    	{
  	    		res->len = 1;
  	    		getAdcVal();
  	    		adcVal = ADCW - 19 - internalTempCalib;
  	    		res->payload[0] = adcVal;
  	    		sendResult = Mirf.sendResult;
  	    		Mirf.sendPacket((mirfPacket*)&outPacket);
//  	    		sprintf((char*)buff, "out: RX:%d,TX:%d,T:%d,C:%d\n", outPacket.rxAddr, outPacket.txAddr, outPacket.type, outPacket.counter);
//  	    		USART_Transmit((char*)buff, strlen((char*)buff) );
//  	    		sprintf((char*)buff, "lastStat: %d\n", sendResult);
//  	    		USART_Transmit((char*)buff, strlen((char*)buff) );
  	    	}
  	    	else if (req->cmd == CALIBRATION_WRITE)
  	    	{
  	    		if (internalTempCalib != req->payload[0])
  	    		{
  	    			internalTempCalib = req->payload[0];
  	    			internalTempCalibValid = 1;
  	    			eeprom_write_byte(SENSOR_0_CALIB_ADDR, req->payload[0]);
  	    		}
  	    	}
  	    	else if (req->cmd == CALIBRATION_READ)
  	    	{
  	    		res->len = 1;
  	    		if (internalTempCalibValid) res->payload[0]=internalTempCalib; else res->payload[0]=0xFF;
  	    		Mirf.sendPacket((mirfPacket*)&outPacket);
  	    	}
         
  		}
  		else if (req->for_sensor == 1) //==== door switch =====
  		{  
  		  if (req->cmd == WRITE)
  		  {
  		     if (req->payload[0] > 0) pinState = HIGH; else pinState = LOW;
  			 digitalWrite(SWITCHED_PIN, pinState);
  		  }
  		  else if (req->cmd == READ)
  		  {
  			 res->len = 1;
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

// void debug(void)
// {
//   uint8_t i;
//   uint8_t volatile rr;
//   uint8_t adr[5];
//   
//   cli();
//   for (i=0; i< 10; i++)
//   {
//     Mirf.readRegister(i, (uint8_t*)&rr, 1);
//     sprintf(buff, "%d:%d,", i, rr);
//     USART_Transmit(buff, strlen(buff) );
//   }
//   sprintf(buff, "\n");
//   USART_Transmit(buff, strlen(buff) );
//   
//   Mirf.readRegister(TX_ADDR, (uint8_t*)&adr, 5);
//   sprintf(buff, "TX: 0x%2x%2x%2x%2x%2x\n", adr[0], adr[1], adr[2], adr[3], adr[4]);
//   USART_Transmit(buff, strlen(buff) );
// 
//   Mirf.readRegister(RX_ADDR_P0, (uint8_t*)&adr, 5);
//   sprintf(buff, "RX: 0x%2x%2x%2x%2x%2x\n", adr[0], adr[1], adr[2], adr[3], adr[4]);
//   USART_Transmit(buff, strlen(buff) );
//   
//   sei();
// }
