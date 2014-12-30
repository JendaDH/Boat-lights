/*
 
 ********************************************************
 *                                                      *
 *                                                      *
 *              Lights Module for RC boat               *
 *              (c) 2014 JendaDH v. 1.0                 *
 *                                                      *
 *                                                      *
 ********************************************************

 MCU Atmega48 - Internal oscilator 8MHz
 LED control IC 2x 74HC595 8bit paralel out shift register
 
 RC Input PINS  - PD0 - PD3 (PD0 is used for syncync incoming pulse from RC receiver. Must be always connected in order to use the module
 Output PINS    - PB0 - PB4

 Self configuration is out of scope for this project
 */


#define F_CPU 8000000UL
#include <stdio.h>
#include <avr/io.h>
#include <interrupt.h>
#include <util/delay.h>

#define DS      0     // shift data
#define SHCP    1     // shift clock
#define STCP    2     // shift latch
#define OE      3     // shift enable
#define MR      4     // shift reset - not used - always set to HIGH by HW wire

#define SHIFT_PORT  PORTB

#define SHIFT_CLOCK_LOW()       SHIFT_PORT &= ~(1<<SHCP)
#define SHIFT_CLOCK_HIGH()      SHIFT_PORT |= (1<<SHCP)
#define SHIFT_LATCH_LOW()       SHIFT_PORT &= ~(1<<STCP)
#define SHIFT_LATCH_HIGH()      SHIFT_PORT |= (1<<STCP)
#define SHIFT_DATA_LOW()        SHIFT_PORT &= ~(1<<DS)
#define SHIFT_DATA_HIGH()       SHIFT_PORT |= (1<<DS)
#define SHIFT_MR_LOW()          SHIFT_PORT &= ~(1<<MR)
#define SHIFT_MR_HIGH()         SHIFT_PORT |= (1<<MR)
#define SHIFT_OE_LOW()          SHIFT_PORT &= ~(1<<OE)
#define SHIFT_OE_HIGH()         SHIFT_PORT |= (1<<OE)

#define MAST_ON()               byte_one |= (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5)
#define MAST_OFF()              byte_one &= ~((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5))
#define POS_ON()                byte_one |= (1<<6) | (1<<7)
#define POS_OFF()               byte_one &= ~((1<<6) | (1<<7))
#define BRIDGE_ON()             byte_two |= (1<<2)
#define BRIDGE_OFF()            byte_two &= ~(1<<2)
#define CREW_ON()               byte_two |= (1<<3) | (1<<4) | (1<<5)
#define CREW_OFF()              byte_two &= ~((1<<3) | (1<<4) | (1<<5))
#define SEARCH_ON()             byte_two |= (1<<0)
#define SEARCH_OFF()            byte_two &= ~(1<<0)
#define DECK_ON()               byte_two |= (1<<1)
#define DECK_OFF()              byte_two &= ~(1<<1)

uint8_t byte_one = 0;           // byte for first shift register
uint8_t byte_two = 0;           // byte for second shift register

volatile uint16_t tot_overflow; // overflow counter to measure 15ms pulse from starting pulse from RC receiver
volatile uint16_t rec_cycle;    // cycle counter - used when first signal is received from RC receiver

// Indivirual RC in counters 1..4
volatile uint16_t vstup1;
volatile uint16_t vstup2;
volatile uint16_t vstup3;
volatile uint16_t vstup4;


// Initialization routine for 8bit Timer0 and 8bit Timer2
void timer_init(void)
{
	// setup Timer0 with 64 prescale
	TCCR0B |= (1 << CS01) | (1 << CS00);
	
	// setup Timer2 with 8 prescale
	TCCR2B |= (1 << CS21);
	
	// enable overflow interrupt for Timer0
	TIMSK0 |= (1 << TOIE0);
	
	// enable overflow interrupt for Timer2
	TIMSK2 |= (1 << TOIE2);
    
	// initialize overflow interrupt counters
	TCNT0 = 0;      // full overflow
	TCNT2 = 208;    // overflow after 50us sequence
	
	//initialize other variables for counting pulses and cycles
	tot_overflow = 0;
	rec_cycle = 0;
	vstup1 = 0;
	vstup2 = 0;
	vstup3 = 0;
	vstup4 = 0;
	
	//enable interrupt
	sei();
}

// Timer0 overflow vector
ISR(TIMER0_OVF_vect)
{
	tot_overflow++;
}

ISR(TIMER2_OVF_vect)
{
	//toggle pin for oscilloscope measurements
	//PORTC ^= (1 << 5);
	
	// check if RC impulse has finished
	if(bit_is_clear(PIND,PIND0))
	{
		//if measurement cycle has not started yet, keep Timer0 at 0
        if(rec_cycle == 0)
		{
			//toggle pin for oscilloscope measurements
            //PORTC &= ~(1 << 4);
			TCNT0 = 0;
		}
	}
	
	// check if RC impulse has started
	if(bit_is_set(PIND,PIND0))
	{
		vstup1++;
        // if measurement cycle has not started yet, start it.
		if(rec_cycle == 0)
		{
			rec_cycle++;
            //toggle pin for oscilloscope measurements
			//PORTC |= (1 << 4);
		}
	}
	
	if(bit_is_set(PIND,PIND1))
	{
		vstup2++;
	}
	
	if(bit_is_set(PIND,PIND2))
	{
		vstup3++;
	}
	
	if(bit_is_set(PIND,PIND3))
	{
		vstup4++;
	}
    
    // reset counter to count another 50us again
	TCNT2 = 208;
}


// byte shifting routine
void shift_bit(uint8_t moje_data)
{
    
	SHIFT_CLOCK_LOW();
	SHIFT_DATA_LOW();
	
	for (int i=7; i>=0; i--)  {
		SHIFT_CLOCK_LOW();
		
		//if the value passed to myDataOut and a bitmask result
		// true then... so if we are at i=6 and our value is
		// %11010100 it would the code compares it to %01000000
		// and proceeds to set pinState to 1.
		
		if ( moje_data & (1<<i) )
		{
			SHIFT_DATA_HIGH();
		}
		else
		{
			SHIFT_DATA_LOW();
		}
		
		//register shifts bits on upstroke of clock pin
		SHIFT_CLOCK_HIGH();
		//zero the data pin after shift to prevent bleed through
		SHIFT_DATA_LOW();
	}
	
	SHIFT_CLOCK_LOW();
	// SHIFT_PORT &= ~(1<<DS);
}


// send shifted byte1 and byte2 to the shift registers
void shift_update(void)
{
	//prvni IC
	SHIFT_OE_HIGH();
	SHIFT_LATCH_LOW();
	shift_bit(byte_one);
	SHIFT_LATCH_HIGH();
	
	//druhe IC
	SHIFT_LATCH_LOW();
	shift_bit(byte_two);
	SHIFT_LATCH_HIGH();
	SHIFT_OE_LOW();
}


// cleat shift registers
void shift_clear(void)
{
	byte_one = 0x00;
	byte_two = 0x00;
	shift_update();
}


// main routine
int main (void)
{
	//set PORTC as output - only used for diagnostic purposes - oscilloscope
	DDRC = 0xFF;
	PORTC = 0x00;
	
	// PORTB as output
	DDRB = 0xFF;
	PORTB = 0x00;
	
	//PORTD as intput
	DDRD = 0x00;
	PORTD = 0x00;
    
    // delay to sync the internal clock
	_delay_ms(500);
	
    // clear shift registers in case there is previous state stored
    shift_clear();
	
    //start timers
	timer_init();
	
	while(1)
	{
		if(tot_overflow>=7)     //
		{                       // check if 15ms has passes or not
			if(TCNT0>=82)       //
			{
				//PORTC ^= (1 << 5);  // toggle pin for diagnostic purposes
				
				//************************
                //*       CHANNEL 1      *
                //************************
                if(vstup1 <= 26)
                {
                    SEARCH_OFF();
                    shift_update();
                }
                
                if((vstup1 > 26) && (vstup1 <= 33))
                {
                    //nic
                }
                
                if(vstup1 > 33)
                {
                    SEARCH_ON();
                    shift_update();
                }
                
                //************************
                //*       CHANNEL 2      *
                //************************
                if(vstup2 <= 26)
                {
                    DECK_OFF();
                    shift_update();
                }
                
                if((vstup2 > 26) && (vstup2 <= 33))
                {
                    //nic
                }
                
                if(vstup2 > 33)
                {
                    DECK_ON();
                    shift_update();
                }
                
                
                //************************
                //*       CHANNEL 3      *
                //************************
                if(vstup3 <= 26)
                {
                    POS_OFF();
                    MAST_OFF();
                    shift_update();
                }
                
                if((vstup3 > 26) && (vstup3 <= 33))
                {
                    POS_ON();
                    MAST_OFF();
                    shift_update();
                }
                
                if(vstup3 > 33)
                {
                    POS_ON();
                    MAST_ON();
                    shift_update();
                }
                
                //************************
                //*       CHANNEL 4      *
                //************************
                if(vstup4 <= 26)
                {
                    BRIDGE_OFF();
                    CREW_OFF();
                    shift_update();
                }
                
                if((vstup4 > 26) && (vstup4 <= 33))
                {
                    BRIDGE_ON();
                    CREW_OFF();
                    shift_update();
                }
                
                if(vstup4 > 33)
                {
                    BRIDGE_ON();
                    CREW_ON();
                    shift_update();
                }
				
				//reset all timers and wait for another pulse sequence from the RC reciever
				vstup1 = 0;
				vstup2 = 0;
				vstup3 = 0;
				vstup4 = 0;
				tot_overflow = 0;
				TCNT0 = 0;
				rec_cycle=0;
				TCNT2 = 208;
			}
		}
		
	}
	return 0;
}
