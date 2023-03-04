#define F_CPU 12000000L
#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdbool.h>

//defining constants
#define BAUD 57600L
#define BRC	((F_CPU/(16L*BAUD)) - 1)
#define MAX_ENCODER_VALUE 1000
#define MIN_ENCODER_VALUE 0
#define NO_OF_POTS   2
#define NO_OF_ENCODERS   2
#define NO_OF_KEYS 16
#define DATA_SIZE  256 
#define DATA_MASK  (uint8_t)(DATA_SIZE-1)
#define BUTTON_COUNTER 0
#define ENCODER1_BUTTON (PINB & 0x02)
#define ENCODER2_BUTTON (PINC & 0x04)
#define MAX_POT_VALUE 2097152
#define NO_OF_STATES 3000

//defining ports
#define ADC1_CS (1<<3)
#define ADC2_CS (1<<2)
#define ADC_CLK (1<<4)
#define PIN_COLUMN1 (PIND & 0x40)
#define PIN_COLUMN2 (PINC & 0x20)
#define PIN_COLUMN3 (PIND & 0x80)
#define PIN_COLUMN4 (PINC & 0x08)
#define PORT_ROW1 0x01
#define PORT_ROW2 0x10
#define PORT_ROW3 0x10
#define PORT_ROW4 0x20

//Defining masks for keyboard and data
#define  KB_PD_MASK  0x30
#define  KB_PB_MASK  0x0D 
#define  KB_PC_MASK  0x10
char dane [DATA_SIZE];
char buffer [DATA_SIZE];

// Function prototypes
void scan_ROW1(void);
void scan_ROW2(void);
void scan_ROW3(void);
void scan_ROW4(void);
void scan_encoder1(void);
void scan_encoder2(void);
void scan_pot1(void);
void scan_pot2(void);
void appendSerial(char c);
void serialWrite (char c[]);
void scanPots(void);

//Structures for components
struct button
{
	char myChar;
	char state;
	char prevState;
	bool changed;
	int16_t  cnt;
};
struct potentiometers
{
	int32_t value;
	int32_t prevValue;
	bool changed;	
};
struct encoder
{
	struct button  b;
	uint32_t  counter;
	bool changed;
};
struct potentiometers pots [NO_OF_POTS] =
{ 
	{0, 0, false},
	{0, 0, false}
};
struct encoder e1[NO_OF_ENCODERS] =
{	{  { 'N', 'N', ' ', false, 0 },
		MAX_ENCODER_VALUE/2,
		false
	},
	{	{ 'N', 'N', ' ', false, 0 },
		MAX_ENCODER_VALUE/2,
		false
	}
};
struct button buttonPressed[NO_OF_KEYS] =
{	{ '1', '1', ' ', false, 0 },
	{ '2', '2', ' ', false, 0 },
	{ '3', '3', ' ', false, 0 },
	{ 'A', 'A', ' ', false, 0 },
	{ '4', '4', ' ', false, 0 },
	{ '5', '5', ' ', false, 0 },
	{ '6', '6', ' ', false, 0 },
	{ 'B', 'B', ' ', false, 0 },
	{ '7', '7', ' ', false, 0 },
	{ '8', '8', ' ', false, 0 },
	{ '9', '9', ' ', false, 0 },
	{ 'C', 'C', ' ', false, 0 },
	{ '*', '*', ' ', false, 0 },
	{ '0', '0', ' ', false, 0 },
	{ '#', '#', ' ', false, 0 },
	{ 'D', 'D', ' ', false, 0 }
	};
uint16_t tics = 0;
uint16_t serialReadPos = 0;
uint16_t serialWritePos = 0;
uint32_t tt=0;	

//defining constants for conversion
#define PS_START_CONV  0x0001	
#define PS_SCAN_POT_1  0x0002
#define PS_SCAN_POT_2  0x0004
#define PS_SCAN_POTS   0x0008
uint16_t programState = 0;
void checkButton(struct button *b, char znak, bool pressedDown);

int main(void)
{
	//assigning masks to direction registers
	DDRD = KB_PD_MASK;
	DDRB = KB_PB_MASK | ADC1_CS | ADC2_CS | ADC_CLK;
	DDRC = KB_PC_MASK;
	sei(); //turning on global interrupts
	UBRR0H = (uint8_t)(BRC/256); //UPPER BAUD BIT REGISTER
	UBRR0L = (uint8_t)(BRC%256); //LOWER BAUD BIT REGISTER
	
   //setting up transmitter
	UCSR0A = (1<< UDRE0);
	UCSR0B = (1<< TXEN0); //TRANSMITTER ENABLE
	UCSR0C = (1<< UCSZ01) | (1<< UCSZ00); //CONTROL REGISTER  
   
	EICRA = 0x0a; //interrupt control register both INT0 and INT1 triggers at falling edge
 
	EIMSK = 0x03; //external interrupt mask register turning on both INT0 and INT1
	
	TCCR0A = (1 << WGM01); //set CTC bit
	OCR0A = 12; //setting timer constant
	TIMSK0 = (1 << OCIE0A); //timer enable
	TCCR0B = (1 << CS02) | (1 << CS00); //prescalar at 1024
  
	PORTB = ADC1_CS | ADC2_CS | ADC_CLK;

	serialWrite("Reset\n");
  
    while (1) 
    {
		if(programState & PS_START_CONV )  // 1 flag
		{
			PORTB |= ADC_CLK;//CLK->1
			PORTB &= ~(ADC1_CS | ADC2_CS);//CS? ->0
			_delay_us(10);
			PORTB |= (ADC1_CS | ADC2_CS);//CS? ->1
			programState &= ~PS_START_CONV;
		}
		else if(programState & PS_SCAN_POT_1) // flag 2
		{
			scan_pot1(); 
			programState &= ~PS_SCAN_POT_1;
		}
		else if(programState & PS_SCAN_POT_2) // flag 3
		{
			scan_pot2();
			programState &= ~PS_SCAN_POT_2;
		}
		if(programState & PS_SCAN_POTS)
		{
			scanPots();
			programState &= ~PS_SCAN_POTS;
		}

		scan_encoder1();
		scan_encoder2();
		scan_ROW3();
		scan_ROW4();
		scan_ROW1();
		scan_ROW2();

		for(uint8_t j=0; j< NO_OF_KEYS;  j++)
		{
			if(buttonPressed[j].changed)
			{
				snprintf(dane, DATA_SIZE, "<KB:%c>\r\n", buttonPressed[j].state );
				serialWrite(dane);
				buttonPressed[j].changed = false;
			}
		}
		for( uint8_t i=0; i< NO_OF_ENCODERS; i++)
		{ 
			if( e1[i].b.changed )		
			{
				snprintf(dane, DATA_SIZE, "<KE%d:%c>\r\n", i, e1[i].b.state);
				serialWrite(dane);
				e1[i].b.changed = false;
			}
			if(e1[i].changed)
			{
				snprintf(dane, DATA_SIZE, "<EN%d:%06ld>\r\n", i, e1[i].counter);
				serialWrite(dane);
				e1[i].changed = false;
			}
		}
	}
}

void scanPots()
{
	for(uint8_t k=0; k< NO_OF_POTS;  k++)
	{
		if(pots[k].changed)
		{
			snprintf(dane, DATA_SIZE, "<POT%d:%06ld tt:%06ld>\r\n", k, pots[k].value, tt );
			serialWrite(dane);
			pots[k].changed = false;
		}
	}
}

void scan_ROW1() 
{
	PORTB |= PORT_ROW1;
	
	_delay_ms(1);
	if (PIN_COLUMN1)
	{
		checkButton( &buttonPressed[0], '1', true );
	}
	else
	{
		checkButton( &buttonPressed[0], ' ', false );
	}
	if (PIN_COLUMN2)
	{	
		checkButton( &buttonPressed[1], '2', true );
	}
	else
	{
		checkButton( &buttonPressed[1], ' ', false );
	}
	if (PIN_COLUMN3) 
	{	
		checkButton( &buttonPressed[2], '3', true );
	}
	else
	{
		checkButton( &buttonPressed[2], ' ', false );
	}
	
	if (PIN_COLUMN4)
	{	
		checkButton( &buttonPressed[3], 'A', true );
	}
	else
	{
		checkButton( &buttonPressed[3], ' ', false );
	}
	
	PORTB &=~ PORT_ROW1;
}

void scan_ROW2() 
{
	PORTD |= PORT_ROW2;
	_delay_ms(10);
	if(PIN_COLUMN1)
	{	
		checkButton( &buttonPressed[4], '4', true );
	}
	else 
	{
		checkButton( &buttonPressed[4], ' ', false );
	}
	if (PIN_COLUMN2)
	{	
		checkButton( &buttonPressed[5], '5', true );
	}
	else
	{
		checkButton( &buttonPressed[5], ' ', false );
	}
	if (PIN_COLUMN3) 
	{	
		checkButton( &buttonPressed[6], '6', true );
	}
	else
	{
		checkButton( &buttonPressed[6], ' ', false );
	}
	if (PIN_COLUMN4)
	{	
		checkButton( &buttonPressed[7], 'B', true );
	}
	else
	{
		checkButton( &buttonPressed[7], ' ', false );
	}
	PORTD &=~ PORT_ROW2;
}

void scan_ROW3() 
{
	PORTC |= PORT_ROW3;
	_delay_ms(1);
	if(PIN_COLUMN1)
	{
		checkButton( &buttonPressed[8], '7', true );
	}
	else
	{
		checkButton( &buttonPressed[8], ' ', false );
	}
	if (PIN_COLUMN2)
	{
		checkButton( &buttonPressed[9], '8', true );
	}
	else
	{
		checkButton( &buttonPressed[9], ' ', false );
	}
	if (PIN_COLUMN3)
	{
		checkButton( &buttonPressed[10], '9', true );
	}
	else
	{
		checkButton( &buttonPressed[10], ' ', false );
	}
	if (PIN_COLUMN4)
	{
		checkButton( &buttonPressed[11], 'C', true );
	}
	else
	{
		checkButton( &buttonPressed[11], ' ', false );
	}
	PORTC &=~ PORT_ROW3;
}

void scan_ROW4() 
{
	PORTD |= PORT_ROW4;
	_delay_ms(1);
	if(PIN_COLUMN1)  // if( bP.myPORT & bPO.myPin )
	{
		checkButton( &buttonPressed[12], '*', true );
	}
	else
	{
		checkButton( &buttonPressed[12], ' ', false );
	}
	if (PIN_COLUMN2)
	{
		checkButton( &buttonPressed[13], '0', true );
	}
	else
	{
		checkButton( &buttonPressed[13], ' ', false );
	}
	if (PIN_COLUMN3)
	{
		checkButton( &buttonPressed[14], '#', true );
	}
	else
	{
		checkButton( &buttonPressed[14], ' ', false );
	}
	if (PIN_COLUMN4)
	{
		checkButton( &buttonPressed[15], 'D', true );
	}
	else
	{
		checkButton( &buttonPressed[15], ' ', false );
	}
	PORTD &=~ PORT_ROW4;
}


void checkButton(struct button *b, char znak, bool pressedDown)
{
	if((pressedDown && b->prevState=='0') || (!pressedDown && b->prevState=='1'))
	{
		b->cnt = 0;
	}
	if(pressedDown)
	{
		b->prevState = '1';
	}
	else
	{
		b->prevState = '0';
	}
	
	if(b->cnt==BUTTON_COUNTER)
	{
		b->changed = true;
		b->state = znak;
		b->cnt++;
	}
	else if(b->cnt<BUTTON_COUNTER)
	{
		b->cnt++;
	}
}

void scan_encoder1()
{
	if (ENCODER1_BUTTON) 
	{	
		checkButton( &(e1[ 0 ].b), 'Y', true );
	}
	else
	{
		checkButton( &(e1[ 0 ].b), ' ', false );
	}
	return;
}

void scan_encoder2() 
{
	if (ENCODER2_BUTTON)
	{	
		checkButton( &(e1[ 1 ].b), 'Y', true );
	}
	else
	{
		checkButton( &(e1[ 1 ].b), ' ', false );
	}
	return;
}

void scan_pot1() 
{
	uint32_t  temp = 0;
	uint32_t mask = (1L << 24);
	PORTB = PORTB|(ADC_CLK);//CLK->1 
	PORTB = PORTB&~(ADC1_CS);//CS? ->0
		
	for(uint8_t i=0; i<24; i++)	
	{
		PORTB = PORTB&~(ADC_CLK); //CLk->0
		if( (PINB & (1 << PINB5)) ) 
		{
			temp |=  mask;
		}
		mask >>= 1;
		PORTB = PORTB|(ADC_CLK); //CLK ->1
	}
	PORTB = PORTB|(ADC1_CS);// CS -> 1
	if( temp>=MAX_POT_VALUE ) // 2^21
	{
		return;
	}
	
	tt = temp;
	temp &= 0x1fffff;
	temp /= (MAX_POT_VALUE/NO_OF_STATES);
	
	pots[0].value = temp;	
	PORTB = PORTB|(ADC1_CS);// CS -> 1
	if(pots[0].value != pots[0].prevValue )
	{
		pots[0].changed = true;
		pots[0].prevValue = pots[0].value;
	}
}

void scan_pot2()
{
	uint32_t  temp = 0;
	uint32_t mask = (1L << 24);
	PORTB = PORTB|(ADC_CLK);//CLK->1
	PORTB = PORTB&~(ADC2_CS);//CS? ->0
	for(uint8_t i=0; i<24; i++)
	{
		PORTB = PORTB&~(ADC_CLK); //CLk->0
		if ((PINB & (1 << PINB5))) 
		{
			temp |=  mask;
		}
		mask >>= 1;
		PORTB = PORTB|(ADC_CLK); //CLK ->1
	}
	PORTB = PORTB|(ADC2_CS);// CS -> 1
	if( temp>=MAX_POT_VALUE ) // 2^21
	{
		return;
	}
	tt = temp;
	temp &= 0x1fffff;
	temp /= (MAX_POT_VALUE/NO_OF_STATES);
	
	pots[1].value = temp;
	if(pots[1].value != pots[1].prevValue )
	{
		pots[1].changed = true;
		pots[1].prevValue = pots[1].value;
	}
}

ISR(INT0_vect) 
{
	if(!(PINC & (1 << PINC0))) 
	{
		if(e1[0].counter>MIN_ENCODER_VALUE)
		{
			e1[0].counter--;
			e1[0].changed = true;
		}
	}
	else 
	{
		if(e1[0].counter<MAX_ENCODER_VALUE)
		{
			e1[0].counter++;
			e1[0].changed = true;
		}
	}
}

ISR(INT1_vect) 
{
	if(!(PINC & (1 << PINC1)))
	{
		if(e1[1].counter>MIN_ENCODER_VALUE)
		{
			e1[1].counter--;
			e1[1].changed = true;
		}
	}
	else
	{
		if(e1[1].counter<MAX_ENCODER_VALUE)
		{
			e1[1].counter++;
			e1[1].changed = true;
		}
	}
}

ISR(TIMER0_COMPA_vect) 
{
	tics++;
	
	if(tics%100==0)  // flag 1
	{
		programState |= PS_START_CONV;
	}
	else if(tics%100==82) // flag 2
	{
		programState |= PS_SCAN_POT_1;
	}
	else if(tics%100==84) // flag 3
	{
		programState |= PS_SCAN_POT_2;
	}
	
	if(tics%100==90) // flag 4
	{
		programState |= PS_SCAN_POTS;
	}
}

void appendSerial(char c) 
{
	buffer[serialWritePos] = c;
	serialWritePos++;
	
	if(serialWritePos >= DATA_SIZE) 
	{
		serialWritePos = 0;
	}
}

void serialWrite (char c[]) 
{
	for (uint16_t i=0; i<strlen(c); i++) 
	{
		appendSerial(c[i]);
	}
	UCSR0B |= (1 << UDRIE0);
}

ISR(USART_UDRE_vect) 
{
	if(serialReadPos != serialWritePos)
	{
		UDR0 = buffer[serialReadPos];
		serialReadPos++;
		if(serialReadPos == serialWritePos)
		{
			UCSR0B &= ~(1 << UDRIE0);
		}
	}
	serialReadPos &= DATA_MASK;
}

	
