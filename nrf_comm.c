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

#include "onewire.h"
#include "ds18x20.h"

#define NUM_SENSORS 3
#define SWITCHED_PIN 9
#define SENSOR_0_CALIB_ADDR (uint8_t *)1

#define SENSOR_0_TYPE 3 //internal temp
#define SENSOR_1_TYPE 0 //on-off output
#define SENSOR_2_TYPE 4 //dallas 18b20 temp sensor
#define SENSOR_3_TYPE 6 //2 lion in series supply

#define MAXSENSORS 1
//uint8_t gDallasID[OW_ROMCODE_SIZE];


mirfPacket volatile inPacket;
mirfPacket volatile outPacket;
uint8_t volatile pinState;
char buff[30];
uint8_t internalTempCalib;
uint8_t sendResult;
uint16_t longTimer;

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
  	
  	longTimer++;
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
  if (internalTempCalib == 0xFF) internalTempCalib = 128;

  //start one wire comm and initialize dallas sensor
  //uint8_t diff = OW_SEARCH_FIRST;
//gDallasReady = 1;
//  if (DS18X20_find_sensor( &diff, &gDallasID[0] ) == DS18X20_OK)
//  {
//	 gDallasReady = 1;
//  }
//  else
//  {
//	  gDallasID[0] = diff;
//  }
  //DS18X20_get_power_status()

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

 memset((void*)&outPacket, 0, sizeof(mirfPacket) );
 memset(buff, 0, sizeof(buff));
 
 //debug();
 while(1) {
   //zpracovat prichozi packet
   if (Mirf.inPacketReady)
   {
     Mirf.readPacket((mirfPacket*)&inPacket);
     
     //sprintf((char*)buff, "in: TX:%d,T:%d,C:%d\n", inPacket.txAddr, inPacket.type, inPacket.counter);
     //USART_Transmit((char*)buff, strlen((char*)buff) );
          
     if ( (PACKET_TYPE)inPacket.type == REQUEST )
	 {
	    payloadRequestStruct *req = (payloadRequestStruct*)&inPacket.payload;
		outPacket.type = RESPONSE;
		outPacket.rxAddr = inPacket.txAddr;
		payloadResponseStruct *res = (payloadResponseStruct*)&outPacket.payload;
		res->cmd = req->cmd;
  		res->from_sensor = req->for_sensor;
  		res->len = 1;

  	    if (req->for_sensor == 0) //==== internal temp sensor =====
  	    {
  	    	if (req->cmd == READ)
  	    	{
  	    		getAdcVal();
  	    		adcVal = ADCW - 19 - internalTempCalib;
  	    		res->payload[0] = adcVal;
  	    		sendResult = Mirf.sendResult;
  	    		Mirf.sendPacket((mirfPacket*)&outPacket);
  	    	}
  	    	else if (req->cmd == CALIBRATION_WRITE)
  	    	{
  	    		if (internalTempCalib != req->payload[0])
  	    		{
  	    			internalTempCalib = req->payload[0];
  	    			eeprom_write_byte(SENSOR_0_CALIB_ADDR, req->payload[0]);
  	    		}
  	    	}
  	    	else if (req->cmd == CALIBRATION_READ)
  	    	{
  	    		res->payload[0] = internalTempCalib;
  	    		Mirf.sendPacket((mirfPacket*)&outPacket);
  	    	}
         
  		}
  		else if (req->for_sensor == 1) //==== door switch =====
  		{  
  		  if (req->cmd == WRITE)
  		  {
  		     if (req->payload[0] > 0) pinState = HIGH; else pinState = LOW;
  			 //digitalWrite(SWITCHED_PIN, pinState);
  		  }
  		  else if (req->cmd == READ)
  		  {
  			 res->payload[0] = pinState;
  			 sendResult = Mirf.sendPacket((mirfPacket*)&outPacket);
  		  }
        }
  		else if (req->for_sensor == 2) //==== dallas 1820 temperature ====
  		{
  				uint8_t sp[DS18X20_SP_SIZE];
  				DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL );
  				while (DS18X20_conversion_in_progress() == DS18X20_CONVERTING) __asm__("nop\n\t");
  				ow_command( DS18X20_READ, NULL );
  					for ( uint8_t i = 0; i < DS18X20_SP_SIZE; i++ ) {
  						sp[i] = ow_byte_rd();
  					}
  		  		res->len = 2;
  				res->payload[0] = sp[0];
  				res->payload[1] = sp[1];
  				//DS18X20_read_maxres_single( gDallasID[0], &temp_eminus4 );
  			Mirf.sendPacket((mirfPacket*)&outPacket);
  		}
  		else if (req->for_sensor == 3) //==== voltage of supply battery ====
  		{ //it is 2 cells in series, so there will be divider /2 on the input (real voltage would be 2x)
  			res->len = 2;
  			Mirf.sendPacket((mirfPacket*)&outPacket);
  		}
	 }
     else if ( (PACKET_TYPE)inPacket.type == PRESENTATION_REQUEST )
     {
 		outPacket.type = PRESENTATION_RESPONSE;
 		outPacket.rxAddr = inPacket.txAddr;
 		payloadPresentationStruct *res = (payloadPresentationStruct*)&outPacket.payload;
 		res->num_sensors = NUM_SENSORS;
   		res->sensor_type[0] = SENSOR_0_TYPE;
   		res->sensor_type[1] = SENSOR_1_TYPE;
   		res->sensor_type[2] = SENSOR_2_TYPE;
   		res->sensor_type[3] = SENSOR_3_TYPE;
   		Mirf.sendPacket((mirfPacket*)&outPacket);
     }
   }
   else if (longTimer > 3000) //30sec period
   {
   		//test environment temp and if it is over 25 degrees, switch channel 1MHz lower
   		longTimer = 0;
   		getAdcVal();
  	    adcVal = ADCW - 19 - internalTempCalib;
  	    uint8_t newChann = Mirf.channel;
        static uint8_t channel_altered = 0;
  	    if ( (adcVal > 133) )
        {
          if (channel_altered == 0)
          {
            newChann -= 1;
            channel_altered = 1;
  	        Mirf.configRegister(RF_CH, newChann);
          }
        }
        else
        {
          if (channel_altered == 1)
          {
            newChann += 1;
            channel_altered = 0;
  	        Mirf.configRegister(RF_CH, newChann);
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