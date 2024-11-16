#ifndef F_CPU
#define F_CPU 20000000UL // 20.000 MHz clock speed
#endif

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#define LCD_PORT                PORTC
#define LCD_DDR			DDRC
#define LCD_EN                  PC4
#define LCD_RS                  PC5
#define LCD_D4                  PC0
#define LCD_D5                  PC1
#define LCD_D6                  PC2
#define LCD_D7                  PC3

#define PRESCALER 16

volatile uint32_t overflow = 0;
volatile double Frequency = 0;
volatile uint8_t print = 0;


void lcd_init();
void lcd_write_instr( uint8_t data );
void lcd_write_data( uint8_t data );
void lcd_write( uint8_t data );
void lcd_goto( uint8_t data );
void lcd_enable_write();
void lcd_busy();
void lcd_print( char *str );

void print_insert();
void print_mh(double val);
void print_uh(double val);

void lcd_init()
{
	LCD_PORT |= ( 1 << LCD_EN );        // Start-up Signal Control
	
	lcd_write_instr( 0b00110011 );
	lcd_write_instr( 0b00110010 );
	lcd_write_instr( 0b00101000 );
	
	lcd_write_instr( 0b00001100 );
	lcd_write_instr( 0b00000110 );
	lcd_write_instr( 0b00000001 );
}

void lcd_write_instr( uint8_t data )
{
	LCD_PORT &= ~( 1 << LCD_RS );       // Write Instruction select
	
	lcd_write( data );
}

void lcd_write_data( uint8_t data )
{
	LCD_PORT |= ( 1 << LCD_RS );        // Write Data Select
	
	lcd_write( data );
}

void lcd_write( uint8_t data )
{
	LCD_PORT &= 0b11110000;             // Clear old Data
	
	LCD_PORT |= ( data >> 4 );          // Write high nibble
	
	lcd_enable_write();
	
	LCD_PORT &= 0b11110000;             // Clear old Data
	
	LCD_PORT |= ( data & 0b00001111 );  // Write low nibble

	lcd_enable_write();
	
	lcd_busy();                         // Wait Busy
}

void lcd_goto( uint8_t data )
{
	data |= ( 1 << 7 );
	
	lcd_write_instr( data );
}

void lcd_enable_write()
{
	LCD_PORT &= ~( 1 << LCD_EN );       // Enable CS of LCD
	
	lcd_busy();                         // Wait Busy
	
	LCD_PORT |= ( 1 << LCD_EN );        // Disable LCD
}

void lcd_busy()
{
	_delay_ms(2);
}

void lcd_print( char *str )
{
	int i = 0;
	while( str[i] ) { lcd_write_data( str[i++] ); }
	
}

void print_insert(){
	lcd_goto( 0x00 );
	lcd_print( "Insert inductor" );
}

void print_mh(double val){
	char fstr[70];
	sprintf( (char*)&fstr, "%.2f mH             ", val / 1000 );
	lcd_goto( 0x00 );
	lcd_print( fstr );
}


void print_uh(double val){
	char fstr[70];
	sprintf( (char*)&fstr, "%.2f uH             ", val );
	lcd_goto( 0x00 );
	lcd_print( fstr );
}

ISR(TIMER0_OVF_vect) // TIMER 0 Interrupt
{
	overflow++;
}

ISR(TIMER1_COMPA_vect) // TIMER 1 Interrupt
{
	Frequency = overflow * 256 + TCNT0;
	TCNT0 = 0;
	overflow = 0;
	print = 1;
}

int main(void)
{
	LCD_DDR = 0xFF; //Makes PORT as Output
	
	double cap = 4.6E-10; //Calibrated value @1nF capacitors ;
	double inductance = 0;
	unsigned long FinalFrequency = 0;

	DDRB =0x00;                                                //PB0 Direction is Input
	PORTB|=(1<<0);                                            //Enabling internal pullup resistor

	// TIMER 1 for interrupt frequency 1.0000128001638422 Hz:
	cli(); // stop interrupts
	TCCR1A = 0; // set entire TCCR1A register to 0
	TCCR1B = 0; // same for TCCR1B
	TCNT1  = 0; // initialize counter value to 0
	// set compare match register for 1.0000128001638422 Hz increments
	OCR1A = 19530; // = 20000000 / (1024 * 1.0000128001638422) - 1 (must be <65536)
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS12, CS11 and CS10 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	//TIMER 0 Setup
	TCCR0A=0;
	TCCR0B |= (1<<CS02) | (1<<CS01) | (1<<CS00);
	TIMSK0=0;
	TIMSK0 |= (1<<TOIE0);
	sei();
	GTCCR=0;

	lcd_init();
	
	for (;;) // Loop forever
	{
		if ( ! print ) continue;

		FinalFrequency = Frequency * PRESCALER;

		inductance = 1. / ( 4 * cap * pow( FinalFrequency, 2 ) * pow( M_PI, 2 ) );
		inductance *= 1.0E6;                    // Converting from H to uH

		if ( FinalFrequency < 1 ) print_insert();
		else {
			if ( inductance >= 1000 ) {
				print_mh(inductance);
			} else print_uh(inductance);
		}
		
		print = 0;
	}
}


