#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdio.h>
#include "Mirf.h"
#include "Mirf_nRF24L01.h"

//DEVICE definition
#define DEV_ADDR 2 //1 is master, so it is not possible
#define LOW_POWER_ENABLE 1

#ifndef DEV_ADDR
    #error "Device(node) address is not defined! Use DEV_ADDR macro."
#else
    #if DEV_ADDR < 2
        #error "This is not master node - address below 2 is not permitted!"
    #endif
#endif

#define TIMER_3_SEC_PERIOD 300
#define TIMER_60_SEC_PERIOD 6000

#include "onewire.h"
#include "ds18x20.h"

#define SWITCHED_PIN 9
#define SENSOR_0_CALIB_ADDR (uint8_t *)1

#if DEV_ADDR==2
    #define NUM_SENSORS 3
#else
    #define NUM_SENSORS 4
#endif

#define LOW_POWER_SENSOR_TYPE_FLAG 128 //is added to value of DS1820 sensor to sign, that this is a low power device
#define SENSOR_0_TYPE 3 //internal temp
#define SENSOR_1_TYPE 0 //on-off output
#define SENSOR_2_TYPE 4 //dallas 18b20 temp sensor
#define SENSOR_3_TYPE 6 //2 lion in series supply

mirfPacket volatile inPacket;
mirfPacket volatile outPacket;
uint8_t volatile pinState;
//char buff[30];
uint8_t internalTempCalib;
volatile uint8_t sendResult;
uint16_t volatile longTimer;

#ifdef LOW_POWER_ENABLE
 #define LOW_POWER_CYCLES 8 //interval = this_number * 8sec
 uint8_t volatile wdt_timer;
 uint8_t volatile low_power_mode = 0;
 #undef SENSOR_2_TYPE
 #define SENSOR_2_TYPE 4 + LOW_POWER_SENSOR_TYPE_FLAG //dallas 18b20 temp sensor + low power sign 
#endif

typedef union {
  uint16_t uint;
  struct {
    uint8_t lsb;
    uint8_t msb;
  };
} IntUnion;

uint8_t volatile adcVal;
volatile IntUnion ds1820Temp;

//======================================================
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
	  SMCR = 0; //disable adc sleep and enable normal Idle mode
}

// WATCHDOG interrupt (for cyclic waking of low power device)
#ifdef LOW_POWER_ENABLE
ISR(WDT_vect)
{
    wdt_timer++;
}
#endif

void ReadDS1820(void)
{
//read temperature from DS1820 and store it to memory

	//DS18X20_read_maxres_single( gDallasID[0], &temp_eminus4 );

	DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL );
	while (DS18X20_conversion_in_progress() == DS18X20_CONVERTING) __asm__("nop\n\t");
	ow_command( DS18X20_READ, NULL );
	
    //read 16bit value into uint16
    ds1820Temp.lsb = ow_byte_rd();
    ds1820Temp.msb = ow_byte_rd();
    
    //read rest of bytes from sensor, but we do not use the data
    for ( uint8_t i = 2; i < DS18X20_SP_SIZE; i++ )
    {
		ow_byte_rd();
	}
}

//======================================================
//WARNING: adc MUX and reference must be set before calling this
void getAdcVal(void)
{
	  SMCR = 2; //enable ADC noise reduct sleep mode

	  ADCSRA |= _BV(ADSC);  // Start the ADC
	  sleep_enable();
	  sleep_cpu();

	  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
	  //adcVal = ADCW;
      //info: adcw is read outside - just right after finishing this function
}

//======================================================
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
  
}

//======================================================
void main(void) __attribute__ ((noreturn));
{
 wdt_disable();   

 setup();
 ReadDS1820();  //for the first time

 sei();
 
 memset((void*)&outPacket, 0, sizeof(mirfPacket) );
 //memset(buff, 0, sizeof(buff));
 
 //debug();
 while(1) {

    #ifdef LOW_POWER_ENABLE 
    //low power mode driven by watchdog resets - loop trap
    if (low_power_mode == 1)
    {
        if (wdt_timer == LOW_POWER_CYCLES) { //sleep mode elapsed, turn off and go to normal mode
            SMCR = 0; //power down mode = off
            WDTCSR = (1<<WDCE) | (1<<WDE);    
            WDTCSR = 0; //wdt = off
            wdt_timer = 0;
            low_power_mode = 0;
            ReadDS1820(); //refresh temperature measurement
            //enable Mirf receiver - run after DS1820 reading, because there is short timeout for sending ack and data packets back
	    //so it wouldt make sense to catch packets during reading of sensor because those packets would be useless after getting to them
            Mirf.powerUpRx();
            TIFR0 = 2; //delete possible interrupt flag of timer0      
            TIMSK0 = 2; //activate interrupts for timer0
        }
        else {  //continue with sleep mode
            WDTCSR |= (1<<WDIE); //interrupt enable flag is atomatically cleared by interrupt for watchdog, must be refreshed
            SMCR = 0b00000100; //power down sleep mode                       		
            sleep_enable();
            sleep_cpu();      
        }

        continue;
    }
    #endif
    
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
            //use value stored in memory
	  		res->len = 2;
			res->payload[0] = ds1820Temp.lsb;
			res->payload[1] = ds1820Temp.msb;
  			Mirf.sendPacket((mirfPacket*)&outPacket);
            
            #ifdef LOW_POWER_ENABLE
                //if we are in low power mode, after first response to request for this sensor
                //wait until the packet is really sent and then
                //increase long timer, so in next while loop it will jump right into power down mode,
                //even if whole interval (3sec) didnt elapse yet
                //this should save some power
                //but limit this feature only on SUCCESSFUL sending of packet 
                while (Mirf.sendResult == 1) NOP_ASM
                if (Mirf.sendResult == 0) { //was it succesfull send?
                    longTimer += TIMER_3_SEC_PERIOD;
                }
            #endif
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
#ifdef LOW_POWER_ENABLE 
   else if (longTimer > TIMER_3_SEC_PERIOD) //3sec period awake (only)
   {
   		longTimer = 0;
        //temperature measurement is refreshed during end of low power mode
         
        // ENTER POWER DOWN MODE - watchdog timed refresh
        TIMSK0 = 0; // turn off 10ms timer (by disabling its interrupt)
        SMCR = 0b00000100; //power down sleep mode
        WDTCSR = (1<<WDCE) | (1<<WDE) | (1<<WDP3) | (1<<WDP0);    
        WDTCSR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0); //8sec timeout of watchdog
        wdt_timer = 0;
        low_power_mode = 1;
        Mirf.powerDown();
                     		
        sleep_enable();
        sleep_cpu();
    }
#else    
   else if (longTimer > TIMER_60_SEC_PERIOD) //60sec period
   {
   		longTimer = 0;
        
        //refresh temperature measurement
        ReadDS1820();  	
   }
#endif   
   else //if there is no packet to be processed, we can enter idle mode to save some power
   {
	  sleep_enable();
	  sleep_cpu();
	  sleep_disable();
   }
 }

}
