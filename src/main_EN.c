/**
  ******************************************************************************
  * @mainpage Measure_3D_object
  * @file    main.c
  * @author  David Halas, Miroslav Hajek; Brno University of Technology; Czechia
  * @date    Dec 3, 2018
  * @brief   Measure 3D object.
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "lcd.h"
#include "lcd_definitions.h"


/* Macros and constants ------------------------------------------------------*/
#define trig1 PC2                           // Set TRIGGER to PC2
#define echo1 PD2                           // Set ECHO to PD2 (INT0)
#define trig2 PC3                           // Set TRIGGER to PC3
#define echo2 PD3                           // Set ECHO to PD3 (INT1)

#define measure_average 10                  // Macro to set the number of the sample to measure the distance meter

#define right 0x7e				            // Macro for right arrow character
#define left 0x7f				            // Macro for left arrow character
#define stupen 0xdf				            // Macro for degree character

#define button_enter PC0                    // Macro for button ENTER
#define button_esc PC1                      // Macro for button ESC
#define button_left PB3                     // Macro for button LEFT
#define button_right PB4                    // Macro for button RIGHT

#define press_enter bit_is_clear(PINC,0)    // Macro for press the button ENTER
#define press_esc bit_is_clear(PINC,1)      // Macro for press the button ESC
#define press_left bit_is_clear(PINB,3)     // Macro for press the button LEFT
#define press_right bit_is_clear(PINB,4)    // Macro for press the button RIGHT

#define temperature_ref 25                  // Reference temperature of the thermistor used
#define resistance_ref 10000                // Reference resistance of the thermistor used
#define rezistor 10000                      // Resistance value
#define beta 3950                           // Beta factor is value from datasheet


/* Function prototypes -------------------------------------------------------*/
void setup(void);
void ultrasonic1(void);
void ultrasonic2(void);
void vypocet(void);
void speed_temperature(void);
void measure_delka_s1(void);
void measure_delka_s2(void);
void measure_delka(void);
void measure_delka_objekt(void);

void lcd_print(char line0[17],char line1[17],uint8_t arrow);



/* Global variables ----------------------------------------------------------*/
volatile uint16_t number0_of_overflow=0;        // Number of overflow counter TC0
volatile uint16_t timer0_value=0;               // Value of counter TC0

char uart_string[5];                            // String for print value to LCD

uint16_t speed;                                 // The speed of sound at temperature
float temperature;                              // Temperature of measure
float voltage;                                  // Voltage on thermistor
float distance;
uint8_t number_of_measure=0;

uint16_t delka_s1=0;                            // Distance measured by sensor 1
uint16_t delka_s2=0;                            // Distance measured by sensor 2
uint16_t delka=0;                               // Distance between sensors
uint16_t delka_objekt=0;                        // Distance object

uint16_t delka_mereni=0;                        // The length of the object being measured
uint16_t sirka_mereni=0;                        // The width of the object being measured
uint16_t vyska_mereni=0;                        // The height of the object being measured

uint8_t	EN = 0;                                 // help variable
uint8_t vykresli = 0;                           // help variable
uint8_t vykresli_objekt = 0;                    // help variable

uint8_t enter_control = 0;                      /* Variables for proper button operation  */
uint8_t esc_control = 0;
uint8_t left_control = 0;
uint8_t right_control = 0;

uint8_t enter_control_OK = 0;                  /* Variables for proper button operation  */
uint8_t esc_control_OK = 0;
uint8_t left_control_OK = 0;
uint8_t right_control_OK = 0;

typedef enum {        // state machine
	UVOD,
	JAZYK_CZ,
	JAZYK_EN,
	MERENI_1D,
	MERENI_2D,
	MERENI_3D,
	MERENI_VZDALENOSTI,
	KONEC
} menu;
menu pozice= UVOD;

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/




/* Functions main -----------------------------------------------------------------*/
int main(void)

{
	setup();        /* Initializations */
	sei();          /* Enables interrupts by setting the global interrupt mask */

_delay_ms(1200);


    while (1)       /* Forever loop */
    {
        speed_temperature();            // Functions for determining the speed of sound in dependence on the measured temperature


        switch (pozice)
		{
		case UVOD:

			lcd_print("|==============|","|**  VITEJ  ** |",3);
			_delay_ms(1500);
			lcd_clrscr();

			lcd_print(" Teplota okoli  "," je:     stupnu",3);
            lcd_gotoxy(6,1);
            itoa(temperature,uart_string,10);
            lcd_puts(uart_string);
            _delay_ms(1500);
			lcd_clrscr();

			lcd_print("OVLADANI POMOCI","|ENTER|ESC| | |",3);
			lcd_gotoxy(11,1);
			lcd_putc(right);
			lcd_gotoxy(13,1);
			lcd_putc(left);
			_delay_ms(2500);
			lcd_clrscr();

			lcd_print("|==============|","|VYBER SI JAZYK|",3);
			_delay_ms(1000);
			lcd_clrscr();

			pozice = JAZYK_CZ;
			vykresli = 1;
			break;

		case JAZYK_CZ:

			if(vykresli == 1)
			{
				lcd_print("|=============  ","|*** CZECH ***  ",1);
				vykresli = 0;
			}

			else if(enter_control == 1)
			{
				EN = 0;
				enter_control = 0;
				lcd_clrscr();
				pozice = MERENI_1D;
				vykresli = 1;
			}

			else if(right_control == 1)
			{
				right_control = 0;
				lcd_clrscr();
				pozice = JAZYK_EN;
				vykresli = 1;
			}

			else if(esc_control == 1)
			{
				esc_control = 0;
				lcd_clrscr();
				pozice = UVOD;
			}

			break;

		case JAZYK_EN:

			if(vykresli == 1)
			{
				lcd_print("  =============|","  ** ENGLISH **|",0);
				vykresli = 0;
			}

			else if(enter_control == 1)
			{
				EN = 1;
				enter_control = 0;
				lcd_clrscr();
				pozice = MERENI_1D;
				vykresli = 1;
			}

			else if(left_control == 1)
			{
				left_control = 0;
				lcd_clrscr();
				pozice = JAZYK_CZ;
				vykresli = 1;
			}

			else if(esc_control == 1)
			{
				esc_control = 0;
				lcd_clrscr();
				pozice = UVOD;
			}

			break;

		case MERENI_1D:

			if(vykresli == 1)
			{
				if(EN == 1)
					lcd_print(" MEASURE ONE    ","  DIMENSION     ",1);
				else
					lcd_print("MERENI JEDNOHO  ","   ROZMERU      ",1);
				measure_delka();
				vykresli = 0;
			}

			else if(enter_control == 1)
			{
				if(EN == 1)
					lcd_print("|==============|","| INPUT OBJECT |",3);
				else
					lcd_print("|==============|","| VLOZ OBJEKT  |",3);
				measure_delka_objekt();
				delka_mereni = delka_objekt;


				if(esc_control == 1)
				{
					esc_control = 0;
					enter_control = 0;
					pozice = MERENI_1D;
				}
			}

			else if(enter_control == 2)
			{
				if(EN == 1)
					lcd_print("|==============|"," TAKE OUT OBJECT",3);
				else
					lcd_print("|==============|","| VYJMI OBJEKT |",3);
				vykresli_objekt = 1;
			}

			else if(enter_control == 3 && vykresli_objekt == 1)
			{
				vykresli_objekt = 0;
				lcd_clrscr();
				if(EN == 1)
					lcd_print(" ** MEASURE ** ","DIMENSION:     ",3);
				else
					lcd_print("ZMERENY ROZMER ","MA VELIKOST:   ",3);
				lcd_gotoxy(12,1);
				itoa(delka_mereni,uart_string,10);
				lcd_puts(uart_string);
			}

			else if(enter_control == 4)
			{
				vykresli = 1;
				lcd_clrscr();
				enter_control = 0;
				pozice = KONEC;
			}                                       // End of function MERENI_1D

			else if(right_control == 1)
			{
				right_control = 0;
				lcd_clrscr();
				pozice = MERENI_2D;
				vykresli = 1;
			}

			else if(esc_control == 1)
			{
				EN = 0;
				esc_control = 0;
				lcd_clrscr();
				pozice = JAZYK_CZ;
				vykresli = 1;
			}

			break;

		case MERENI_2D:

			if(vykresli == 1)
			{
				measure_delka();
				if(EN == 1)
					lcd_print("   MEASURE 2D   ","     OBJECT     ",2);
				else
					lcd_print("   MERENI 2D    ","    OBJEKTU     ",2);
				vykresli = 0;
			}

			else if(enter_control == 1)
			{
				if(EN == 1)
					lcd_print("|==============|","| INPUT OBJECT |",3);
				else
					lcd_print("|==============|","| VLOZ OBJEKT  |",3);

				measure_delka_objekt();
				delka_mereni = delka_objekt;

				if(esc_control == 1)
				{
					esc_control = 0;
					enter_control = 0;
					pozice = MERENI_2D;
				}
			}

			else if(enter_control == 2)
			{
				if(EN == 1)
					lcd_print("| TURN OBJECT  |","| 90  RIGHT    |",3);
				else
					lcd_print("|OTOC OBJEKT O |","| 90  DOPRAVA  |",3);

				lcd_gotoxy(4,1);
				lcd_putc(stupen);
				measure_delka_objekt();
				sirka_mereni = delka_objekt;
				vykresli_objekt = 1;

				if(esc_control == 1)
				{
					esc_control = 0;
					enter_control --;
				}
			}

			else if(enter_control == 3)
			{
				if(EN == 1)
				lcd_print("|==============|"," TAKE OUT OBJECT",3);
				else
				lcd_print("|==============|","| VYJMI OBJEKT |",3);
				vykresli_objekt = 1;
			}

			else if(enter_control == 4 && vykresli_objekt == 1)
			{
				vykresli_objekt = 0;
				lcd_clrscr();
				if(EN == 1)
					lcd_print("SIZE OBJECT ARE","X:     Y:      ",3);
				else
					lcd_print("VELIKOST OBJEKT","X:     Y:      ",3);

				lcd_gotoxy(2,1);
				itoa(delka_mereni,uart_string,10);
				lcd_puts(uart_string);

				lcd_gotoxy(9,1);
				itoa(sirka_mereni,uart_string,10);
				lcd_puts(uart_string);
			}

			else if(enter_control == 5)
			{
				lcd_clrscr();
				enter_control = 0;
				vykresli = 1;
				pozice = KONEC;
			}                                       // End of function MERENI_2D

			else if(right_control == 1)
			{
				right_control = 0;
				lcd_clrscr();
				pozice = MERENI_3D;
				vykresli = 1;
			}

			else if(left_control == 1)
			{
				left_control = 0;
				lcd_clrscr();
				pozice = MERENI_1D;
				vykresli = 1;
			}

			else if(esc_control == 1)
			{
				EN = 0;
				esc_control = 0;
				lcd_clrscr();
				pozice = JAZYK_CZ;
				vykresli = 1;
			}

			break;

		case MERENI_3D:

			if(vykresli == 1)
			{
				measure_delka();
				if(EN == 1)
					lcd_print("   MEASURE 3D   ","     OBJECT     ",2);
				else
					lcd_print("   MERENI 3D    ","    OBJEKTU     ",2);

				vykresli = 0;
			}

			if(enter_control == 1)
			{
				if(EN == 1)
					lcd_print("|==============|","| INPUT OBJECT |",3);
				else
					lcd_print("|==============|","| VLOZ OBJEKT  |",3);

				measure_delka_objekt();
				delka_mereni = delka_objekt;

				if(esc_control == 1)
				{
					esc_control = 0;
					enter_control = 0;
					pozice = MERENI_3D;
				}
			}

			else if(enter_control == 2)
			{
				if(EN == 1)
					lcd_print("| TURN OBJECT  |","| 90  RIGHT    |",3);
				else
					lcd_print("|OTOC OBJEKT O |","| 90  DOPRAVA  |",3);

				lcd_gotoxy(4,1);
				lcd_putc(stupen);
				measure_delka_objekt();
				sirka_mereni = delka_objekt;

				if(esc_control == 1)
				{
					esc_control = 0;
					enter_control --;
				}
			}

			else if(enter_control == 3)
			{
				if(EN == 1)
					lcd_print("| OVERCLOCKING |","| OBJECT RIGHT |",3);
				else
					lcd_print("|PREKLOP OBJEKT|","| * DOPRAVA *  |",3);
				measure_delka_objekt();
				vyska_mereni = delka_objekt;

				if(esc_control == 1)
				{
					esc_control = 0;
					enter_control --;
				}

			}

			else if(enter_control == 4)
			{
				if(EN == 1)
					lcd_print("|==============|"," TAKE OUT OBJECT",3);
				else
					lcd_print("|==============|","| VYJMI OBJEKT |",3);

				vykresli_objekt = 1;

				if(esc_control == 1)
				{
					esc_control = 0;
					enter_control --;
				}
			}

			else if(enter_control == 5 && vykresli_objekt == 1)
			{
				vykresli_objekt = 0;
				lcd_clrscr();
				if(EN == 1)
					lcd_print("SIZE OBJECT ARE","X:   Y:   Z:   ",3);
				else
					lcd_print("VELIKOST OBJEKT","X:   Y:   Z:   ",3);
				lcd_gotoxy(2,1);
				itoa(delka_mereni,uart_string,10);
				lcd_puts(uart_string);

				lcd_gotoxy(7,1);
				itoa(sirka_mereni,uart_string,10);
				lcd_puts(uart_string);

				lcd_gotoxy(12,1);
				itoa(vyska_mereni,uart_string,10);
				lcd_puts(uart_string);
			}

			else if(enter_control == 6)
			{
				lcd_clrscr();
				enter_control = 0;
				vykresli = 1;
				pozice = KONEC;
			}                                       // End of function MERENI_3D

			else if(right_control == 1)
			{
				right_control = 0;
				lcd_clrscr();
				pozice = MERENI_VZDALENOSTI;
				vykresli = 1;
			}

			else if(left_control == 1)
			{
				left_control = 0;
				lcd_clrscr();
				pozice = MERENI_2D;
				vykresli = 1;
			}

			else if(esc_control == 1)
			{
				EN = 0;
				esc_control = 0;
				lcd_clrscr();
				pozice = JAZYK_CZ;
				vykresli = 1;
			}

			break;

		case MERENI_VZDALENOSTI:

			if(vykresli == 1)
			{
				if(EN == 1)
					lcd_print("    MEASURE     ","    DISTANCE    ",0);
				else
					lcd_print("     MERENI     ","   VZDALENOSTI  ",0);

				vykresli = 0;
			}

			else if(enter_control == 1)
			{
				if(EN == 1)
					lcd_print("|==============|","|PLACE SENSOR 2|",3);
				else
					lcd_print("|==============|","|UMISTI SENZOR2|",3);
				vykresli_objekt = 1;
			}

			else if(enter_control == 2 && vykresli_objekt == 1)
			{
				vykresli_objekt = 2;
				lcd_clrscr();
				if(EN == 1)
					lcd_print(" ** MEASURE ** ","DISTANCE:     ",3);
				else
					lcd_print(" ** ZMERENA ** ","VZDALENOST:   ",3);
				measure_delka_s2();
				delka_mereni = delka_s2;
				lcd_gotoxy(11,1);
				itoa(delka_mereni,uart_string,10);
				lcd_puts(uart_string);
			}

			else if(enter_control == 2 && vykresli_objekt == 2)
			{

				measure_delka_s2();
				delka_mereni = delka_s2;
				lcd_gotoxy(11,1);
				lcd_puts("     ");
				lcd_gotoxy(11,1);
				itoa(delka_mereni,uart_string,10);
				lcd_puts(uart_string);
				_delay_ms(350);
			}

			else if(enter_control == 3)
			{
				lcd_clrscr();
				enter_control = 0;
				vykresli = 1;
				pozice = KONEC;
			}                                       // End of function MERENI_VZDALENOSTI

			else if(left_control == 1)
			{
				left_control = 0;
				lcd_clrscr();
				pozice = MERENI_3D;
				vykresli = 1;
			}

			else if(esc_control == 1)
			{
				EN = 0;
				esc_control = 0;
				lcd_clrscr();
				pozice = JAZYK_CZ;
				vykresli = 1;
			}
			break;

		case KONEC:

			if(vykresli == 1)
			{
				if(EN == 1)
					lcd_print("|   THANK YOU  |","|  * FOR USE * |",3);
				else
					lcd_print("|  DEKUJEME ZA |","|  * POUZITI * |",3);

				_delay_ms(2000);
				lcd_clrscr();
				if(EN == 1)
				{
					lcd_print("|  FOR REPEAT  |","|MEASURE  ENTER|",3);
				}
				else
					lcd_print("| PRO OPETOVNE |","|MERENI - ENTER|",3);

				vykresli = 0;
			}

			if(enter_control == 1)
			{
				enter_control = 0;
				lcd_clrscr();
				pozice = MERENI_1D;
				vykresli = 1;
			}                                       //End od function KONEC
			break;

		default:        // List error message for failure

			lcd_print("|  ** ERROR ** |","| RESTART - ESC|",3);

			if(esc_control == 1)
			{
				esc_control = 0;
				pozice = UVOD;
				lcd_clrscr();
			}

		}	        // End switch-case

    }		    // End while loop

    return 0;
}			// End function main
/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/



/* Function for initializations -----------------------------------------------------------------*/
void setup(void) /* Setup all peripherals */
{
    TCCR0B |= _BV(CS00);                                        // Clock prescaler 1 => overflows every 16 us
    TIMSK0 |= _BV(TOIE0);                                       // Overflow interrupt enable TC0
    TCCR1B |= _BV(CS12);                                        // Clock prescaler 256 => overflows every 1 s
    TIMSK1 |= _BV(TOIE1);                                       // Overflow interrupt enable TC1
    TCCR2B |= (_BV(CS22)+_BV(CS21));                            // Clock prescaler 256 => overflows every 4 ms
    TIMSK2 |= _BV(TOIE0);                                       // Overflow interrupt enable TC2

    EIMSK |= _BV(INT0);                                         // Set external interrupt INT0 (INT0 is connected to PD2)
    EICRA |= _BV(ISC01) | _BV(ISC00);                           // Set interrupt INT0 for raising edge
    EIMSK |= _BV(INT1);                                         // Set external interrupt INT1 (INT1 is connected to PD3)
    EICRA |= _BV(ISC11) | _BV(ISC10);                           // Set interrupt INT1 for raising edge

    ADMUX |= _BV(REFS0);    ADMUX &= ~_BV(REFS1);               // Analog to Digital Converter
    ADMUX |= _BV(MUX2);                                         // register ADMUX: Set ADC voltage reference to AVcc with external capacitor, select input channel ADC4 (PC4)
    ADMUX &= ~(_BV(MUX0) & _BV(MUX1)  & _BV(MUX3));

    ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);             // register ADCSRA: ADC Enable, ADC Auto Trigger Enable, ADC Interrupt Enable
    ADCSRA |= _BV(ADIE) | _BV(ADEN) | _BV(ADATE);               // ADC Prescaler 128 => fadc = fcpu / 128 = 125 kHz

    ADCSRB |= _BV(ADTS1) | _BV(ADTS2);                          // register ADCSRB: Set ADC Auto Trigger Source to Timer/Counter1 overflow


	DDRC |= _BV(trig1);                                         // Set TRIG pin (1) as output
	DDRC |= _BV(trig2);                                         // Set TRIG pin (2) as output
	DDRD &= ~_BV(echo1);                                        // Set ECHO pin (1) as input
	DDRD &= ~_BV(echo2);                                        // Set ECHO pin (2) as input

	PORTC &= ~_BV(trig1);                                       // Set default logic value 0 for output pin TRIGGER from sensor 1
	PORTC &= ~_BV(trig2);                                       // Set default logic value 0 for output pin TRIGGER from sensor 2
	PORTD &= ~_BV(echo1);                                       // Set default logic value 0 for input pin ECHO from sensor 1
	PORTD &= ~_BV(echo2);                                       // Set default logic value 0 for input pin ECHO from sensor  2

	DDRC &= ~(_BV(button_enter) + _BV(button_esc));	            // Set the Pin for the input buttons
	DDRB &= ~(_BV(button_left) + _BV(button_right));
	PORTC |= (_BV(button_enter) + _BV(button_esc));	            //Active pull-up resistor
	PORTB |= (_BV(button_left) + _BV(button_right));


    lcd_init(LCD_DISP_ON);			                            // Initialize display and select type of cursor
    lcd_command(1<<LCD_CGRAM);
    lcd_clrscr();                                               // Clear display and set cursor to home position
}



/* Functions for determining the speed in dependence on the measured temperature -----------------------------------------------------------------*/
void speed_temperature(void)
{
    if (temperature<= -5)
    {
        if(EN == 1)
            lcd_print("||  At winter ||",":) NO WORKING :)",3);
		else
			lcd_print("|| Je mi zima ||",":) NEPRACUJI  :)",3);
        _delay_ms(1000);
            lcd_clrscr();
    }
    else if (temperature> -5 && temperature<=5)
        speed = 331;
    else if (temperature> 5 && temperature<=15)
        speed = 337;
    else if (temperature> 15 && temperature<=25)
        speed = 343;
    else if (temperature> 25 && temperature<=35)
        speed = 349;
    else if (temperature> 35 && temperature<=45)
        speed = 355;
    else if (temperature> 45 && temperature<=55)
        speed = 360;
    else if (temperature> 55)
    {
        if(EN == 1)
            lcd_print("| Is very HOT, |",":) NO WORKING :)",3);
		else
			lcd_print("| Je mi teplo, |",":) NEPRACUJI  :)",3);
        _delay_ms(1000);
            lcd_clrscr();
    }
}



/* Function for set TRIG1 -----------------------------------------------------------------*/
void ultrasonic1(void)
{
    /* Send the impulse for 10us to start the measurement */
        PORTC &= ~_BV(trig1);
        _delay_us(2);
        PORTC |= _BV(trig1);
        _delay_us(10);
        PORTC &= ~_BV(trig1);
}



/* Function for set TRIG2 -----------------------------------------------------------------*/
void ultrasonic2(void)
{
    /* Send the impulse for 10us to start the measurement */
        PORTC &= ~_BV(trig2);
        _delay_us(2);
        PORTC |= _BV(trig2);
        _delay_us(10);
        PORTC &= ~_BV(trig2);
}



/* Function for time transfer to distance  -----------------------------------------------------*/
void vypocet(void)
{
    distance = speed * 0.000016 * (timer0_value / 2);    // Measure distance [m] (speed*binary value of time/2
    distance = distance * 1000;                          // Transfer distance from meter to millimeter
}


/* Function for measure distance d1 ---------------------------------------------------------------------*/
void measure_delka_s1(void)
{
    delka_s1=0;
    distance=0;

        for(number_of_measure=0; number_of_measure<measure_average; number_of_measure++)
        {
            ultrasonic1();               // Set TRIGGER1
            _delay_ms(12);
            vypocet();                   // Measure distance from sensor 1

                delka_s1=delka_s1+ distance;
        }
    delka_s1 = delka_s1/number_of_measure;          // Calculation average of the measured value
    number_of_measure=0;
}

/* Function for measure distance d2 ----------------------------------------------------------------------*/
void measure_delka_s2(void)
{
    delka_s2=0;
    distance=0;

        for(number_of_measure=0; number_of_measure<measure_average; number_of_measure++)
        {
            ultrasonic2();              // Set TRIGGER2
            _delay_ms(12);
            vypocet();                  // Measure distance from sensor 2

                delka_s2=delka_s2+ distance;
        }
    delka_s2 = delka_s2/number_of_measure;          // Calculation average of the measured value
    number_of_measure=0;
}



/* Function for measure distance d -----------------------------------------------------------------------*/
void measure_delka(void)

{
    delka=0;
    measure_delka_s1();                     // Call function for measure distance from sensor 1
	measure_delka_s2();                     // Call function for measure distance from sensor 2
    delka = (delka_s1+delka_s2)/2;          // Calculation the distance between 2 sensors
    delka_s1=0;
    delka_s2=0;
}



/* Function for measure distance object ---------------------------------------------------------------------*/
void measure_delka_objekt(void)
{
   delka_objekt=0;

        measure_delka_s1();              // Measure delka_s1
        measure_delka_s2();              // Measure delka_s2

   delka_objekt = delka - delka_s1 - delka_s2;      // Calculation the distance object
}



/* Function for printing on LCD display --------------------------------------------------------------------*/
void lcd_print(char line0[17],char line1[17],uint8_t arrow)
{
	lcd_gotoxy(0,0);
	lcd_puts(line0);
	lcd_gotoxy(0,1);
	lcd_puts(line1);

	if(arrow == 0)                  // Print the arrow in the menu on the left side
	{
		lcd_gotoxy(0,0);
		lcd_putc(left);

		lcd_gotoxy(0,1);
		lcd_putc(left);
	}
	else if(arrow == 1)             // Print the arrow in the menu on the right side
	{
		lcd_gotoxy(15,0);
		lcd_putc(right);

		lcd_gotoxy(15,1);
		lcd_putc(right);
	}
	else if(arrow == 2)             // Print the arrow in the menu on the left and right side
	{
		lcd_gotoxy(0,0);
		lcd_putc(left);

		lcd_gotoxy(0,1);
		lcd_putc(left);

		lcd_gotoxy(15,0);
		lcd_putc(right);

		lcd_gotoxy(15,1);
		lcd_putc(right);
	}
}
/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/




/* Overflow function TIMER0 ------------------------------------------------------------------*/
ISR(TIMER0_OVF_vect)
{
    number0_of_overflow++;          // Calculation number of overflow counter TC0
}



/* Overflow function TIMER1 ------------------------------------------------------------------*/
ISR(TIMER1_OVF_vect)
{
    /* ADC Auto trigger is enabled, therefore a new AD conversion starts automatically */
}



/* Overflow function TIMER2 ------------------------------------------------------------------*/
ISR(TIMER2_OVF_vect)
{
   if(press_enter)                                  // Functions for proper button operation ENTER
	{
		enter_control_OK ++;
		if(enter_control_OK == 4)
			enter_control ++;
	}

	else if(press_esc)                              // Functions for proper button operation ESC
	{
		esc_control_OK ++;
		if(esc_control_OK == 4)
			esc_control = 1;
	}

	else if(press_left)                             // Functions for proper button operation LEFT
	{
		left_control_OK ++;
		if(left_control_OK == 4)
			left_control = 1;
	}

	else if(press_right)                            // Functions for proper button operation RIGHT
	{
		right_control_OK ++;
		if(right_control_OK == 4)
			right_control = 1;
	}

	else                                            // Delete the contents of all flag variables for reusable use
	{
		enter_control_OK = 0;
		esc_control_OK = 0;
		left_control_OK = 0;
		right_control_OK = 0;
	}
}
/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/




/* Function INT0 -------------------------------------------------------------------*/
ISR(INT0_vect)
{
    if (bit_is_set(PIND,2))                     // Pin ECHO(1) have a logic value 1
    {
       TCNT0 = 0;                                   // Clear register TCNT counter 0
       number0_of_overflow=0;

       EICRA |= _BV(ISC01);                         // Set interrupt INT0 for falling edge
       EICRA &= ~_BV(ISC00);
    }

    else                                        // Pin ECHO(1) have a logic value 0
        {
           timer0_value = number0_of_overflow;      // Set number of overflow TC0 to new variable

           EICRA |= _BV(ISC01) | _BV(ISC00);        // Set interrupt INT0 for raising edge
        }
}



/* Function INT1 ------------------------------------------------------------------*/
ISR(INT1_vect)
{
    if (bit_is_set(PIND,3))                     // Pin ECHO(2) have a logic value 1
    {
       TCNT0 = 0;                                   // Clear register TCNT counter 0
       number0_of_overflow=0;

       EICRA |= _BV(ISC11);                         // Set interrupt INT0 for falling edge
       EICRA &= ~_BV(ISC10);
    }

    else                                        // Pin ECHO(2) have a logic value 0
        {
           timer0_value = number0_of_overflow;      // Set number of overflow TC0 to new variable

           EICRA |= _BV(ISC11) | _BV(ISC10);        // Set interrupt INT0 for raising edge
        }
}



/* Function for A/D convert 4 -------------------------------------------------------*/
ISR(ADC_vect)                   //Temperature measurement function using a thermistor connected to the AD converter
{
    voltage = ADC;                          // Read 10-bit value from ADC
    voltage = 1023 / voltage - 1;
    voltage = rezistor / voltage;           // Calculated resistance from voltage on the thermistor


        /* Calculation of the temperature using the modified thermodynamic resistance formula */

            temperature = voltage / resistance_ref;             // (R/Ro)
            temperature = log(temperature);                     // ln(R/Ro)
            temperature /= beta;                                // 1/B * ln(R/Ro)
            temperature += 1.0 / (temperature_ref + 273.15);    // + (1/To)
            temperature = 1.0 / temperature;                    // 1/T  Temperature in Kelvin
            temperature -= 273.15;                              // Converted value to Celsius
}



/* END OF FILE ******************************************************************************************************************************************************/
