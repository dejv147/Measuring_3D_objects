/**
  ******************************************************************************
  * @mainpage Mereni_3D_objektu
  * @file    main.c
  * @author  David Halas, Miroslav Hajek; Brno University of Technology; Czechia
  * @date    Dec 3, 2018
  * @brief   Mereni 3D objektu.
  * @note    Mereni na principu vlozeni objektu mezi 2 ultrazvukove snimace.
  ******************************************************************************
  */


/* Knihovny ------------------------------------------------------------------*/
#define F_CPU 16000000UL                    /**< @brief Definice hodnoty hodinoveho signalu CLK pouziteho mikrokontroleru   */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "lcd.h"
#include "lcd_definitions.h"


/* Makra a konstanty ------------------------------------------------------*/
#define trig1 PC2                           /**< @brief Pripojeni TRIGGER na PIN2 portu C           */
#define echo1 PD2                           /**< @brief Pripojeni ECHO na PIN2 portu D (INT0)       */
#define trig2 PC3                           /**< @brief Pripojeni TRIGGER na PIN3 portu C           */
#define echo2 PD3                           /**< @brief Pripojeni ECHO na PIN3 portu D (INT1)       */

#define measure_average 10                   /**< @brief Makro pro nastaveni poctu vzorku mereni pro vypocet prumeru merene vzdalenosti      */

#define right 0x7e				            /**< @brief  Makro pro znak sipky vpravo        */
#define left 0x7f				            /**< @brief  Makro pro znak sipky vlevo         */
#define stupen 0xdf				            /**< @brief  Makro pro znak stupne              */

#define button_enter PC0                    /**< @brief Makro pro tlacitko ENTER            */
#define button_esc PC1                      /**< @brief Makro pro tlacitko ESC              */
#define button_left PB3                     /**< @brief Makro pro tlacitko VLEVO            */
#define button_right PB4                    /**< @brief Makro pro tlacitko VPRAVO           */

#define press_enter bit_is_clear(PINC,0)    /**< @brief Makro pro stisk tlacitka ENTER      */
#define press_esc bit_is_clear(PINC,1)      /**< @brief Makro pro stisk tlacitka ESC        */
#define press_left bit_is_clear(PINB,3)     /**< @brief Makro pro stisk tlacitka VLEVO      */
#define press_right bit_is_clear(PINB,4)    /**< @brief Makro pro stisk tlacitka VPRAVO     */

#define temperature_ref 25                  /**< @brief Referencni hodnota teploty pouziteho termistoru                 */
#define resistance_ref 10000                /**< @brief Referencni hodnota odporu pouziteho termistoru                  */
#define rezistor 10000                      /**< @brief Hodnota odporu pouzite pri mereni teploty pomoci termistoru     */
#define beta 3950                           /**< @brief Hodnota beta faktoru udavaneho vyrobcem pouziteho termistoru    */


/* Nacteni vsech funkci -------------------------------------------------------*/
void setup(void);                               // Funkce pro defaultni nastaveni programu
void ultrasonic1(void);                         // Funkce pro vyslani pulsu na pin TRIG snimace 1 pro zahajeni mereni
void ultrasonic2(void);                         // Funkce pro vyslani pulsu na pin TRIG snimace 1 pro zahajeni mereni
void vypocet(void);                             // Funkce pro vypocet namerene vzdalenosti snimacem
void speed_temperature(void);                   // Funkce pro urceni rychlosti zvuku v zavislosti na teplote

void measure_delka_s1(void);                    // Funkce pro mereni vzdalenosti pomoci snimace 1
void measure_delka_s2(void);                    // Funkce pro mereni vzdalenosti pomoci snimace 2
void measure_delka(void);                       // Funkce pro mereni vzdalenosti mezi 2 snimaci
void measure_delka_objekt(void);                // Funkce pro mereni 1 rozmeru mereneho objektu

void lcd_print(char line0[17],char line1[17],uint8_t arrow);    /*  Funkce pro vypis textu na LCD displej
                                                                        line0 == vypis na prvni radek displeje
                                                                        line1 == vypis na druhy radek displeje
                                                                        arrow == vypis sipek pro spravnou interpretaci aktualni pozice v menu:
                                                                            arrow = 0  ->  sipky jsou vykresleny vlevo
                                                                            arrow = 1  ->  sipky jsou vykresleny vpravo
                                                                            arrow = 2  ->  sipky jsou vykresleny na obou stranach
                                                                            arrow = 3  ->  sipky nejsou vykresleny
                                                                */



/* Definice promennych ----------------------------------------------------------*/
volatile uint16_t number0_of_overflow=0;        /**< @brief Pocet preteceni citace 0                                                            */
volatile uint16_t timer0_value=0;               /**< @brief Aktualni hodnota citace TC0                                                         */

char uart_string[5];                            /**< @brief Definice stringu pro vypis hodnot na displej pomoci UART nebo LCD                   */

uint16_t speed;                                 /**< @brief Rychlost zvuku pri dane teplote                                                     */
float temperature;                              /**< @brief Aktualni teplota pro urceni rychlosti zvuku ve vzduchu                              */
float voltage;                                  /**< @brief Pomocna promenna pro mereni napeti na termistoru snimaneho pomoci AD prevodniku     */
float distance;                                 /**< @brief Namerena vzdalenost snimace                                                         */
uint8_t number_of_measure=0;                    /**< @brief Pocet vzorku mereni                                                                 */

uint16_t delka_s1=0;                            /**< @brief Vysledna namerena vzdalenost snimace 1                                              */
uint16_t delka_s2=0;                            /**< @brief Vysledna namerena vzdalenost snimace 2                                              */
uint16_t delka=0;                               /**< @brief Vysledna vzdalenost mezi 2 snimaci                                                  */
uint16_t delka_objekt=0;                        /**< @brief Vysledny rozmer mereneho objektu                                                    */

uint16_t delka_mereni=0;                        /**< @brief Delka mereneho objektu                                                              */
uint16_t sirka_mereni=0;                        /**< @brief Sirka mereneho objektu                                                              */
uint16_t vyska_mereni=0;                        /**< @brief Vyska mereneho objektu                                                              */

uint8_t	EN = 0;                                 /**< @brief Pomocna promenna pro vyber anglickeho jazyka                                        */
uint8_t vykresli = 0;                           /**< @brief Pomocna promenna pro vypsani textu pouze 1x                                         */
uint8_t vykresli_objekt = 0;                    /**< @brief Pomocna promenna pro vypsani textu pouze 1x pri mereni vicerozmernych predmetu      */

uint8_t enter_control = 0;                      /**< @brief Indikace stisku tlacitka ENTER                                                      */
uint8_t esc_control = 0;                        /**< @brief Indikace stisku tlacitka ESC                                                        */
uint8_t left_control = 0;                       /**< @brief Indikace stisku tlacitka VLEVO                                                      */
uint8_t right_control = 0;                      /**< @brief Indikace stisku tlacitka VPRAVO                                                     */

uint8_t enter_control_OK = 0;                   /**< @brief Pomocna promenna pro osetreni tlacitka ENTER                                        */
uint8_t esc_control_OK = 0;                     /**< @brief Pomocna promenna pro osetreni tlacitka ESC                                          */
uint8_t left_control_OK = 0;                    /**< @brief Pomocna promenna pro osetreni tlacitka VLEVO                                        */
uint8_t right_control_OK = 0;                   /**< @brief Pomocna promenna pro osetreni tlacitka VPRAVO                                       */

typedef enum {                      /** @typedef Definice stavoveho automatu pro orientaci v jednotlivych funkcich hlavniho programu   */
	UVOD,
	JAZYK_CZ,
	JAZYK_EN,
	MERENI_1D,
	MERENI_2D,
	MERENI_3D,
	MERENI_VZDALENOSTI,
	KONEC
} menu;
menu pozice= UVOD;                  /**< @brief Pomocna promenna pro spravnou orientaci v jednotlivych castech stavoveho automatu       */

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/




/* Funkce pro obsluhu programu -----------------------------------------------------------------*/
int main(void)
/**
    * @brief Funkce pro obsluhu hlavniho programu (main)
    * @brief *******************************************
    * @note Na zacatku programu jsou nastaveny vsechny potrebne ridici registry a pouzivane I/O piny volanim funkce setup().
    * @note V dalsim kroku je volana funkce sei() pro povoleni vsech preruseni. Nasleduje casove zpozdeni potrebne ke zmereni teploty pro spravnou funkci mereni.
    * @note Hlavni cast programu je vykonavana v podobe stavoveho automatu v nekonecne smycke.
    * @note *  Jednotlive casti stavoveho automatu jsou popsany nize:
*/
{
	setup();        //Inicializace programu volanim funkce setup
	sei();          //Povoleni preruseni

_delay_ms(1200);    //Zpozdeni z duvodu prodlevy pri mereni teploty pro spravne zobrazeni teploty v uvodni obrazovce


    while (1)       //Nekonecna smycka
    {
        speed_temperature();            //Funkce pro urceni rychlosti zvuku v zavislosti na namerene teplote


        switch (pozice)
		{
		case UVOD:
        /**
            * @brief Uvodni sekce stavoveho automatu pro obsluhu menu programu (UVOD)
            *************************************************************************
            * @details - V teto sekci je vykreslena posloupnost nekolika uvodnich obrazovek,
            kde je uzivatel seznamen s obsluhou programu a nasledne je uzivatel vyzvan
            k vyberu jazyka hlavniho menu.
            * @details - Stavovy automat prechazi do pozice JAZYK_CZ
        */

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
        /**
             * @brief Sekce stavoveho automatu pro volbu ceskeho jazyka (JAZYK_CZ)
             *********************************************************************
             * @details - Jednou je na displej vykreslena informace o jazyku CZ
             a uzivatel muze pomoci tlacitek menu ovladat:
			 * @details - *           ENTER - Menu se nastavi na jazyk CZ a skoci na pozici MERENI_1D
             * @details - *           RIGHT - Menu skoci na pozici JAZYK_EN a ceka na stisk dalsiho tlacitka
             * @details - *           ESC - Menu skoci na pozici UVOD a uzivateli je opet vypsan zacatek menu s instrukcemi apod.
		     * @details -  Vzdy po stisku urciteho tlacitka je priznak ..._CONTROL nastaven na 0,
			 aby bylo mozne indikovat dalsi stisknuti tlacitka pro ovladani dalsi casti menu.
             * @details - Stavovy automat prechazi do pozice MERENI_1D, nebo JAZYK_EN
         */

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
        /**
            * @brief Sekce stavoveho automatu pro volbu ceskeho jazyka (JAZYK_EN)
            *********************************************************************
            * @details - Jednou je na displej vykreslena informace o jazyku EN
            a uzivatel muze MENU ovladat pomoci tlacitek:
			* @details - *         ENTER - Menu se nastavi na jazyk EN a skoci na pozici MERENI_1D
			* @details - *         LEFT - Menu skoci na pozici JAZYK_CZ a ceka na stisk dalsiho tlacitka
			* @details - *         ESC - Menu skoci na pozici UVOD a uzivateli je opet vypsan zacatek menu s instrukcemi apod
			* @details - Vzdy po stisku urciteho tlacitka je priznak ..._CONTROL nastaven na 0,
			aby bylo mozne indikovat dalsi stisknuti tlacitka pro ovladani dalsi casti menu.
            * @details - Stavovy automat prechazi do pozice MERENI_1D, nebo JAZYK_CZ
        */

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
        /**
            * @brief Sekce stavoveho automatu pro mereni 1 rozmeru (MERENI_1D)
            ******************************************************************
            * @details - Mereni jednoho rozmeru probiha v 5ti krocich.
            Vzdy po stisku tlacitka ENTER se provede nasledujici krok mereni.
            * @details -     1) V prvnim kroku se na obrazovku vypise moznost pro vyber mereni 1D objektu
            a zmeri se vzdalenost mezi obema snimaci.
            * @details -     2) Ve druhem kroku je uzivatel vyzvan k vlozeni mereneho objektu do oblasti mereni (oblast mezi 2 senzory).
            Nasledne probehne mereni vlozeneho objektu (viz measure_delka_objekt())
            a vysledny rozmer je ulozen do pomocne promenne delka_mereni.
            * @details -     3) Ve tretim kroku je uzivatel vyzvan k vyjmuti objektu z merici oblasti, coz je opet potvrzeno stiskem ENTER.
            * @details -     4) Ve ctvrtem kroku je na displej vypsana vysledna namerena hodnota ( v mm) odpovidajici velikosti mereneho objektu.
            * @details -     5) V poslednim kroku je potvrzena namerena hodnota stiskem ENTER
            a mereni je ukonceno prechodem stavoveho automatu na dalsi pozici.
            * @details -  Stavovy automat prechazi do pozice KONEC a ceka na stisk dalsiho tlacitka:
			* @details - * ESC - po stisku program skoci na pozici JAZYK_CZ a vypise uzivateli vyber jazyka
			* @details - * RIGHT - po stisku program skoci na pozici MERENI_2D a vypise se moznost vyberu 2D mereni
        */

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
			}                                       //Konec funkce MERENI_1D

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
        /**
            * @brief Sekce stavoveho automatu pro mereni 2 rozmeru (MERENI_2D)
            ******************************************************************
            * @details - Mereni dvou rozmeru probiha v 6ti krocich.
            Vzdy po stisku tlacitka ENTER se provede nasledujici krok mereni.
            * @details -     1) V prvnim kroku se na obrazovku vypise moznost pro vyber mereni 2D objektu
            a zmeri se vzdalenost mezi obema snimaci.
            * @details -     2) Ve druhem kroku je uzivatel vyzvan k vlozeni mereneho objektu do oblasti mereni (oblast mezi 2 senzory).
            Nasledne probehne mereni vlozeneho objektu (viz measure_delka_objekt())
            a vysledny rozmer je ulozen do pomocne promenne delka_mereni.
            * @details -     3) Ve tretim kroku je uzivatel vyzvan k otoceni objektu a opet probehne mereni objektu.
            Vysledny namereny rozmer je ulozen do pomocne promenne sirka_mereni.
            * @details -     4) Po potvrzeni stiskem tlacitka ENTER je uzivatel vyzvan k vyjmuti objektu z oblasti mereni.
            * @details -     5) V patem kroku je na displej vypsana vysledna namerena hodnota ( v mm) odpovidajici velikosti mereneho objektu.
            Vysledne hodnoty jsou zobrazeny pomoci souradnicovych identifikatoru X a Y.
            * @details -     6) V poslednim kroku je potvrzena namerena hodnota stiskem ENTER
            a mereni je ukonceno prechodem stavoveho automatu na dalsi pozici.
            * @details - Stavovy automat prechazi do pozice KONEC a ceka na stisk dalsiho tlacitka:
			* @details - *  ESC - po stisku program skoci na pozici JAZYK_CZ a vypise uzivateli vyber jazyka
			* @details - *  LEFT - po stisku program skoci na pozici MERENI_1D a vypise se moznost vyberu 1D mereni
			* @details - *  RIGHT - po stisku program skoci na pozici MERENI_3D a vypise se moznost vyberu 3D mereni
        */
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
			}                                       //Konec funkce MERENI_2D

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
        /**
            * @brief Sekce stavoveho automatu pro mereni 3 rozmeru (MERENI_3D)
            ******************************************************************
            * @details - Mereni dvou rozmeru probiha v 7 krocich.
            Vzdy po stisku tlacitka ENTER se provede nasledujici krok mereni.
            * @details -      1) V prvnim kroku se na obrazovku vypise moznost pro vyber mereni 2D objektu
            a zmeri se vzdalenost mezi obema snimaci.
            * @details -      2) Ve druhem kroku je uzivatel vyzvan k vlozeni mereneho objektu do oblasti mereni (oblast mezi 2 senzory).
            Nasledne probehne mereni vlozeneho objektu (viz measure_delka_objekt())
            a vysledny rozmer je ulozen do pomocne promenne delka_mereni.
            * @details -      3) Ve tretim kroku je uzivatel vyzvan k otoceni objektu a opet probehne mereni objektu.
            Vysledny namereny rozmer je ulozen do pomocne promenne sirka_mereni.
			* @details -      4) Uzivatel je vyzvan k preklopeni objektu na pravou stranu, aby bylo mozne zmerit vysku objektu
			a pote je zmereny rozmer ulozen do pomocne promenne vyska_mereni
            * @details -      5) Po potvrzeni stiskem tlacitka ENTER je uzivatel vyzvan k vyjmuti objektu z oblasti mereni.
            * @details -      6) V patem kroku je na displej vypsana vysledna namerena hodnota ( v mm) odpovidajici velikosti mereneho objektu.
            Vysledne hodnoty jsou zobrazeny pomoci souradnicovych identifikatoru X a Y a Z
            * @details -      7) V poslednim kroku je potvrzena namerena hodnota stiskem ENTER
            a mereni je ukonceno prechodem stavoveho automatu na dalsi pozici.
            * @details - Stavovy automat prechazi do pozice KONEC a ceka na stisk dalsiho tlacitka:
			* @details - *   ESC - po stisku program skoci na pozici JAZYK_CZ a vypise uzivateli vyber jazyka
			* @details - *   LEFT - po stisku program skoci na pozici MERENI_2D a vypise se moznost vyberu 2D mereni
			* @details - *   RIGHT - po stisku program skoci na pozici MERENI_VZDALENOSTI a vypise se moznost vyberu mereni vzdalenosti
        */
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
			}                                       //Konec funkce MERENI_3D

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
        /**
            * @brief Sekce stavoveho automatu pro mereni vzdalenosti pomoci 1 snimace (MERENI_VZDALENOSTI)
            **********************************************************************************************
			* @details - Pri vyberu moznosti mereni vzdalenosti je uzivatel vyzvan k umisteni senzoru 2 do stabilni polohy.
			* @details - Po stisku tlacitka ENTER se provede mereni vzdalenosti pomoci funkce measure_delka_s2() a vysledna hodnota je vypsana na obrazovku LCD.
			* @details - Mereni se provadi opakovane v urcitem casovem intervalu a na displeji se vzdy prepise minula hodnota nove namerenou aktualni hodnotou.

			* @details - Po opetovnem stisku tlacitka ENTER stavovy automat prechazi do pozice KONEC a ceka na stisk dalsiho tlacitka:
			* @details - *   ESC - po stisku program skoci na pozici JAZYK_CZ a vypise uzivateli vyber jazyka
			* @details - *   LEFT - po stisku program skoci na pozici MERENI_3D a vypise se moznost vyberu 3D mereni
        */
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
			}                                       //Konec funkce MERENI_VZDALENOSTI

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
        /**
			* @brief Sekce stavoveho automatu pro zaver menu (KONEC)
			********************************************************
			* @details - Nejprve je uzivateli podekovano za vyuziti naseho mericiho pristroje a nasledne
			  je vypsana moznost noveho mereni po stisknuti ENTER.
			* @details - Obe tyto obrazovky jsou vypsany pouze jednou a to pouzitim pomocne promenne vykresli.
			* @details - Program pote stale kontroluje zda nebylo stisknuto tlacitko ENTER.
			V pripade ze ano, bude pozice stavoveho automatu nastavena na MERENI_1D.
		*/
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
			}                                       //Konec funkce KONEC
			break;

		default:        //Vypsani chyboveho oznameni pro pripad poruchy

			lcd_print("|  ** ERROR ** |","| RESTART - ESC|",3);

			if(esc_control == 1)
			{
				esc_control = 0;
				pozice = UVOD;
				lcd_clrscr();
			}

		}	        // Konec switch-case

    }		    // Konec while cyklu

    return 0;
}			// Konec funkce main




/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/
 /*
  *  Pomocne funkce programu:
  ***************************
  *  setup() == Funkce pro defaultni nastaveni programu
  *  speed_temperature() == Funkce pro urceni rychlosti zvuku v zavislosti na teplote
  *  ultrasonic1() == Funkce pro vyslani pulsu na pin TRIG snimace 1 pro zahajeni mereni
  *  ultrasonic2() == Funkce pro vyslani pulsu na pin TRIG snimace 2 pro zahajeni mereni
  *  vypocet() == Funkce pro vypocet namerene vzdalenosti snimacem
  *  measure_delka_s1() == Funkce pro mereni vzdalenosti pomoci snimace 1
  *  measure_delka_s2() == Funkce pro mereni vzdalenosti pomoci snimace 2
  *  measure_delka() == Fnkce pro mereni vzdalenosti mezi 2 snimaci
  *  measure_delka_objekt() == Funkce pro mereni 1 rozmeru mereneho objektu
  *  lcd_print() == Funkce pro vypis textu na LCD displej
  */
/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/* Funkce pro inicializaci programu -----------------------------------------------------------------*/
void setup(void)
/**
    * @brief Funkce pro defaultni nastaveni programu (setup)
    * @brief ***********************************************
    * @details Zde jsou nastaveny ridici registry pro nastaveni potrebnych parametru citace/casovace, externiho preruseni, AD prevodniku.
    * @details Dale jsou zde nastaveny pouzite I/O piny a defaultni logicke urovne na techto pinech.
    * @details Dale je zde nastavena inicializace LCD displeje a pristup do pameti displeje.
*/
{
    TCCR0B |= _BV(CS00);                                        //Nastaveni preddelicky 1 (16us) pro TC0
    TIMSK0 |= _BV(TOIE0);                                       //Povoleni citace TC0
    TCCR1B |= _BV(CS12);                                        //Nastaveni preddelicky 256(1s) pro TC1
    TIMSK1 |= _BV(TOIE1);                                       //Povoleni citace TC1
    TCCR2B |= (_BV(CS22)+_BV(CS21));                            //Nastaveni preddelicky 256 (4ms) pro TC2
    TIMSK2 |= _BV(TOIE0);                                       //Povoleni citace TC2

    EIMSK |= _BV(INT0);                                         //Povoleni externiho preruseni INT0 (INT0 je pripojeno na PD2)
    EICRA |= _BV(ISC01) | _BV(ISC00);                           //Nastaveni reakce INT0 na nastupnou hranu pinu
    EIMSK |= _BV(INT1);                                         //Povoleni externiho preruseni INT1 (INT1 je pripojeno na PD3)
    EICRA |= _BV(ISC11) | _BV(ISC10);                           //Nastaveni reakce INT1 na nastupnou hranu pinu

    ADMUX |= _BV(REFS0);    ADMUX &= ~_BV(REFS1);               //Nastaveni externiho referencniho napeti AVcc s externi kapacitou na pinu Aref pro AD prevodnik
    ADMUX |= _BV(MUX2);                                         //Vyber AD prevodniku ADC4, ktery je pripojen na PINC4
    ADMUX &= ~(_BV(MUX0) & _BV(MUX1)  & _BV(MUX3));

    ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);             //Nastaveni delicky hodinoveho signalu (128) pro zajisteni vzorkovaciho signalu cca 125 kHz
    ADCSRA |= _BV(ADIE) | _BV(ADEN) | _BV(ADATE);               //Nastaveni pinu ADC Enable, ADC Auto Trigger Enable, ADC Interrupt Enable pro spravnou funkci AD prevodniku

    ADCSRB |= _BV(ADTS1) | _BV(ADTS2);                          //Nastaveni zahajeni AD prevodu pri preteceni citace TC1


	DDRC |= _BV(trig1);                                         //Nastaveni pinu TRIG(1) jako vystupni
	DDRC |= _BV(trig2);                                         //Nastaveni pinu TRIG(2) jako vystupni
	DDRD &= ~_BV(echo1);                                        //Nastaveni pinu ECHO(1) jako vstupni
	DDRD &= ~_BV(echo2);                                        //Nastaveni pinu ECHO(2) jako vstupni

	PORTC &= ~_BV(trig1);                                       //Nastaveni defaultni hodnoty 0 na vystupni pin pro TRIGGER snimace 1
	PORTC &= ~_BV(trig2);                                       //Nastaveni defaultni hodnoty 0 na vystupni pin pro TRIGGER snimace 2
	PORTD &= ~_BV(echo1);                                       //Nastaveni defaultni hodnoty 0 na vstupni pin pro ECHO snimace 1
	PORTD &= ~_BV(echo2);                                       //Nastaveni defaultni hodnoty 0 na vstupni pin pro ECHO snimace 2

	DDRC &= ~(_BV(button_enter) + _BV(button_esc));	            //Nastaveni Pinu pro tlacitka jako vstupni
	DDRB &= ~(_BV(button_left) + _BV(button_right));
	PORTC |= (_BV(button_enter) + _BV(button_esc));	            //Aktivace pull-up rezistoru na pinech
	PORTB |= (_BV(button_left) + _BV(button_right));


    lcd_init(LCD_DISP_ON);			                            //Inicializace displeje, bez kurzoru
    lcd_command(1<<LCD_CGRAM);                                  //Nastaveni pristupu k pameti LCD displeje
    lcd_clrscr();                                               //Smazani obsahu displeje
}



/* Funkce pro urceni rychlosti v zavislosti na namerene teplote -----------------------------------------------------------------*/
void speed_temperature(void)
/**
    * @brief Funkce pro urceni rychlosti zvuku v zavislosti na namerene teplote (speed_temperature)
    * @brief **************************************************************************************
    * @details Tato funkce slouzi k vyhodnoceni namerene teploty a nasledne urceni rychlosti sireni zvuku v prostoru,
    v zavislosti na okolni teplote ziskane merenim.
    * @details Ziskana rychlost zvuku je dale pouzita pri vypoctu merene vzdalenosti.
*/
{
    if (temperature<= -5)
    {
        if(EN == 1)
            lcd_print("||  At winter ||",":) NO WORKING :)",3);
		else
			lcd_print("|| Je mi zima ||",":) NEPRACUJI  :)",3);
        _delay_ms(1000);
            lcd_clrscr();   //Smazani obsahu displeje
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
            lcd_clrscr();   //Smazani obsahu displeje
    }
}



/* Funkce pro vyslani pulzu TRIG1 -----------------------------------------------------------------*/
void ultrasonic1(void)
/**
    * @brief Funkce pro vyslani impulzu na pin TRIGGER snimace 1 pro zahajeni mereni (ultrasonic1)
    * @brief **************************************************************************************
    * @details Pro zajisteni spravneho mereni je pin TRIG1 nastaven na uroven 0 na kratky casovy interval.
    Pote je pin TRIG1 nastaven na logickou uroven 1 a po urcitem casovem intervalu (10us)
    je tento pin opet nastaven na logickou uroven 0.
    * @details Pri nastaveni logicke urovne 1 na pin TRIG1 se nastavi pin ECHO1 na logickou uroven 1
    a z vysilace je vyslana serie pulsu. Az prijimac prijme vyslanou posloupnost zmeni pin ECHO1 na logickou uroven 0.
    * @details Doba trvani logicke urovne 1 na pinu ECHO1 odpovida casu, za ktery signal z vysilace dorazi k prekazce a zpet k prijimaci.
*/
{
    /* Vyslani impulsu o delce 10us pro zahajeni mereni */
        PORTC &= ~_BV(trig1);
        _delay_us(2);
        PORTC |= _BV(trig1);
        _delay_us(10);
        PORTC &= ~_BV(trig1);
}



/* Funkce pro vyslani pulzu TRIG2 -----------------------------------------------------------------*/
void ultrasonic2(void)
/**
    * @brief Funkce pro vyslani impulzu na pin TRIGGER snimace 2 pro zahajeni mereni (ultrasonic2)
    * @brief **************************************************************************************
    * @details Pro zajisteni spravneho mereni je pin TRIG2 nastaven na uroven 0 na kratky casovy interval.
    Pote je pin TRIG1 nastaven na logickou uroven 1 a po urcitem casovem intervalu (10us)
    je tento pin opet nastaven na logickou uroven 0.
    * @details Pri nastaveni logicke urovne 1 na pin TRIG2 se nastavi pin ECHO1 na logickou uroven 1
    a z vysilace je vyslana serie pulsu. Az prijimac prijme vyslanou posloupnost zmeni pin ECHO2 na logickou uroven 0.
    * @details Doba trvani logicke urovne 1 na pinu ECHO2 odpovida casu, za ktery signal z vysilace dorazi k prekazce a zpet k prijimaci.
*/
{
    /* Vyslani impulsu o delce 10us pro zahajeni mereni */
        PORTC &= ~_BV(trig2);
        _delay_us(2);
        PORTC |= _BV(trig2);
        _delay_us(10);
        PORTC &= ~_BV(trig2);
}



/* Funkce pro prevod casu mereni na vzdalenost -----------------------------------------------------*/
void vypocet(void)
/**
    * @brief Funkce pro vypocet namerene vzdalenosti snimacem (vypocet)
    * @brief **********************************************************
    * @details K vypoctu vzdalenosti je nutne znat rychlost sireni zvuku (ziskana pomoci funkce speed_temperature())
    a cas, za ktery merici impuls urazi merenou vzdalenost (ziskan merici funkci INT0_vect/INT1_vect).
    * @details Zvukovy signal cestuje k prekazce a zpet, proto je nutne namereny cas vydelit 2.
    * @details Vzdalenost je ziskana vynasobenim rychlosti a casu (ziskan bitovou hodnotou poctu preteceni casovace a frekvenci hodinoveho signalu).
    * @details Hodnota vzdalenosti je ulozena do pomocne promenne a nasledne prevedena na hodnotu v milimetrech pro nasledne zpracovani.
*/
{
    distance = speed * 0.000016 * (timer0_value / 2);    //Vysledna namerena vzdalenost [m] (rychlost*binarni hodnota casu vydelena 2 jelikoz signal cestuje k prekazce a zpet
    distance = distance * 1000;                          //Prevod vzdalenosti na mm
}


/* Funkce mereni vzdalenosti d1 ---------------------------------------------------------------------*/
void measure_delka_s1(void)
/**
    * @brief Funkce pro mereni vzdalenosti pomoci snimace 1 (measure_delka_s1)
    * @brief *****************************************************************
    * @details Mereni vzdalenosti probiha merenim casu, za ktery signal dojde k prekazce a zpet ke snimaci 1.
    Tato zmena je indikovana trvanim logicke hodnoty 1 na pinu ECHO(1).
    * @details Merena hodnota je prumerovana z nekolika vzorku mereni (pocet vzorku je dan konstantou measure_average).
    * @details V kazdem cyklu mereni je vyslan pozadavek na pin TRIGGER1 (pomoci funkce ultrasonic1()),
    pote se ceka na zmenu pinu ECHO(1), ktera indikuje navrat mericiho signalu vyslaneho snimacem.
    * @details Nasledne probehne vypocet vzdalenosti (pomoci funkce vypocet()) a po dokonceni vsech mericich cyklu je proveden vypocet prumeru z namerenych vzorku.
    * @details Pomocna promenna pro pocet provedenych mereni jednotlivych vzorku je nastavena na 0 pro moznost zahajeni opetovneho mereni pomoci ultrazvukoveho snimace 1.
*/
{
    delka_s1=0;
    distance=0;

        for(number_of_measure=0; number_of_measure<measure_average; number_of_measure++)
        {
            ultrasonic1();               //zjisteni doby mereni pro vypocet namerene vzdalenosti 1
            _delay_ms(12);
            vypocet();                   //vypocet namerene vzdalenosti snimace

                delka_s1=delka_s1+ distance;
        }
    delka_s1 = delka_s1/number_of_measure;          //funkce pro prumer z namerenych hodnot
    number_of_measure=0;
}

/* Funkce mereni vzdalenosti d2 ----------------------------------------------------------------------*/
void measure_delka_s2(void)
/**
    * @brief Funkce pro mereni vzdalenosti pomoci snimace 2 (measure_delka_s2)
    * @brief *****************************************************************
    * @details Mereni vzdalenosti probiha merenim casu, za ktery signal dojde k prekazce a zpet ke snimaci 2.
    Tato zmena je indikovana trvanim logicke hodnoty 1 na pinu ECHO(2).
    * @details Merena hodnota je prumerovana z nekolika vzorku mereni (pocet vzorku je dan konstantou measure_average).
    * @details V kazdem cyklu mereni je vyslan pozadavek na pin TRIGGER2 (pomoci funkce ultrasonic2()),
    pote se ceka na zmenu pinu ECHO(2), ktera indikuje navrat mericiho signalu vyslaneho snimacem.
    * @details Nasledne probehne vypocet vzdalenosti (pomoci funkce vypocet()) a po dokonceni vsech mericich cyklu je proveden vypocet prumeru z namerenych vzorku.
    * @details Pomocna promenna pro pocet provedenych mereni jednotlivych vzorku je nastavena na 0 pro moznost zahajeni opetovneho mereni pomoci ultrazvukoveho snimace 2.
*/
{
    delka_s2=0;
    distance=0;

        for(number_of_measure=0; number_of_measure<measure_average; number_of_measure++)
        {
            ultrasonic2();              //zjisteni doby mereni pro vypocet namerene vzdalenosti 2
            _delay_ms(12);
            vypocet();                  //vypocet namerene vzdalenosti snimace

                delka_s2=delka_s2+ distance;
        }
    delka_s2 = delka_s2/number_of_measure;          //funkce pro prumer z namerenych hodnot
    number_of_measure=0;
}



/* Funkce mereni vzdalenosti d -----------------------------------------------------------------------*/
void measure_delka(void)
/**
    * @brief Fnkce pro mereni vzdalenosti mezi 2 snimaci (measure_delka)
    * @brief ***********************************************************
    * @details Mereni vzdalenosti mezi 2 snimaci spociva v postupnem mereni obou snimacu a naslednem vypoctu prumeru ziskanych hodnot.
    * @details Nejprve je smazana hodnota posledniho mereni pro eliminaci chyb pri vypoctu.
    * @details Pote probehne mereni vzdalenosti pomoci snimace 1 (funkce measure_delka_s1()) a vysledek je ulozen.
    Pote probehne mereni pomoci snimace 2 (funkce measure_delka_s2()) a vysledek je opet ulozen.
    * @details Nasledne je vypocten prumer techto 2 hodnot a vysledek je ulozen do promenne delka.
    * @details V zaveru jsou hodnoty ziskane merenim jednotlivych snimacu smazany pro moznost opetovneho mereni.
*/
{
    delka=0;
    measure_delka_s1();                     //Volani funkce pro zmereni vzdalenosti snimacem 1
	measure_delka_s2();                     //Volani funkce pro zmereni vzdalenosti snimacem 2
    delka = (delka_s1+delka_s2)/2;          //Zjisteni vzdalenosti mezi 2 snimaci
    delka_s1=0;
    delka_s2=0;
}



/* Funkce mereni rozmeru objektu ---------------------------------------------------------------------*/
void measure_delka_objekt(void)
/**
    * @brief Funkce pro mereni 1 rozmeru mereneho objektu (measure_delka_objekt)
    * @brief *******************************************************************
    * @details Mereni objektu spociva v mereni vzdalenosti snimacem 1 a nasledne snimacem 2 a vypoctu vysledne hodnoty.
    * @details V prvnim kroku je smazana hodnota posledniho mereni pro eliminaci chyb pri vypoctu.
    * @details Dale probehne mereni vzdalenosti mezi objektem a snimacem 1 (pomoci funkce measure_delka_s1()) a mereni mezi objektem a snimacem 2 (pomoci funkce measure_delka_s2()).
    * @details Velikost mereneho objektu je ziskana odectenim vzdalenosti objektu od snimace 1 a od snimace 2 od celkove vzdalenosti mezi obema snimaci.
*/
{
   delka_objekt=0;

        measure_delka_s1();              //Mereni delka_s1
        measure_delka_s2();              //Mereni delka_s2

   delka_objekt = delka - delka_s1 - delka_s2;      //Vypocet delky objektu
}



/* Funkce pro vypis na LCD displej --------------------------------------------------------------------*/
void lcd_print(char line0[17],char line1[17],uint8_t arrow)
/**
    * @brief Funkce pro vypis menu na LCD displej (lcd_print)
    * @brief ************************************************
    * @details Tato funkce umoznuje vypsat oba radky LCD a soucasne
	pridat sipky po leve, nebo prave strane, pripadne na obou stranach.

	* @brief - Vstupni promenne pro tuto funkci jsou:
	* @param line0 - Pole o delce 16 znaku + nulovy znak na konec retezce pro vypis na prvni radek
    * @param line1 -  Pole o delce 16 znaku + nulovy znak na konec retezce pro vypis na druhy radek
    * @param arrow - Ciselna hodnota, ktera umoznuje pridani sipek na zacatek obrazovky, konec obrazovky, nebo na obe strany.

	* @brief Tato funkce je tedy pouzitelna bez uprav na displej 16x2 pozic.
	* @par Ciselna hodnota zadana jako treti vstupni parametr prideluje pozici sipek dle nasledujicich hodnot:
	* - 0 == sipky vlevo
	* - 1 == sipky vpravo
	* - 2 == sipky vlevo i vpravo
	* - 3 a vice == displej bez sipek
	* @warning Vstupni retezce musi mit maximalne 16 znaku
*/
{
	lcd_gotoxy(0,0);
	lcd_puts(line0);
	lcd_gotoxy(0,1);
	lcd_puts(line1);

	if(arrow == 0)                  //Vykresleni sipek v menu po leve strane
	{
		lcd_gotoxy(0,0);
		lcd_putc(left);

		lcd_gotoxy(0,1);
		lcd_putc(left);
	}
	else if(arrow == 1)             //Vykresleni sipek v menu po prave strane
	{
		lcd_gotoxy(15,0);
		lcd_putc(right);

		lcd_gotoxy(15,1);
		lcd_putc(right);
	}
	else if(arrow == 2)             //Vykresleni sipek v menu po obou stranach
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
/*
  *   Funkce pro obsluhu citace/casovace (ISR):
  *********************************************
  *   TIMER0_OVF_vect == Citac TC0 ma nastavenou preddelicku hodinoveho signalu na 1, k preteceni tedy dojde kazdych 16 us
  *   TIMER1_OVF_vect == Citac TC1 ma nastavenou preddelicku hodinoveho signalu na 256, k preteceni tedy dojde kazdych 1048 ms
  *   TIMER2_OVF_vect == Citac TC2 ma nastavenou preddelicku hodinoveho signalu na 256, k preteceni tedy dojde kazdych 4096 us
  */
/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/* Funkce TIMER0 ------------------------------------------------------------------*/
ISR(TIMER0_OVF_vect)
/**
    * @brief Funkce pro obsluhu preruseni vyvolaneho pretecenim citace TC0 (TIMER0_OVF_vect)
    * @brief *******************************************************************************
    * @details Zde je inkrementovana promenna, ktera pocita preteceni daneho citace.
    * @details K preteceni dojde kazdych 16 us. Pocet preteceni se zpracovava v podprogramu vypocet(), kde se tato promena vynuluje.
    * @details Neni tedy ovlivnena pretecenim teto promenne a je zpracovana pouze spravna hodnota poctu preteceni.
*/
{
    number0_of_overflow++;
}



/* Funkce TIMER1 ------------------------------------------------------------------*/
ISR(TIMER1_OVF_vect)
/**
    * @brief Funkce pro obsluhu preruseni vyvolaneho pretecenim citace TC1 (TIMER1_OVF_vect)
    * @brief *******************************************************************************
    * @details V pripade preteceni tohoto citace se vyvola preruseni pro obsluhu funkce AD prevodniku.
    * @details Tento citac je nastaven na preteceni kazdou 1s.
*/
{

}



/* Funkce TIMER2 ------------------------------------------------------------------*/
ISR(TIMER2_OVF_vect)
/**
    * @brief Funkce pro obsluhu preruseni vyvolaneho pretecenim citace TC2 (TIMER2_OVF_vect)
    * @brief *******************************************************************************
    * @details Tato funkce slouzi k osetreni stisku tlacitek: ENTER, ESC, VLEVO, VPRAVO.
    * @details Pokud pomocna promenna ..._OK dosahne hodnoty 4, inkrementuje se priznak ..._CONTROL,
    ktery indikuje stisk tlacitka.
    * @details Timto zpusobem se eliminuje vyvolani priznaku stisku tlacitka vyvolaneho nahodnym impulzem,
    pripadne opakovany stisk tlacitka vyvolany prechodnymi jevy pri stisku tlacitka.
    * @details V zavislosti na priznaku stisku tlacitka ..._CONTROL je v hlavnim programu vyvolana prislusna cast.
    * @details Tyto priznakove promenne jsou v hlavnim programu mazany po dokonceni obsluhy dane casti programu,
    proto aby bylo mozne indikovat stisk tlacitka v dalsich castech programu.
    * @details Diky inkrementaci priznakove promenne ..._CONTROL je mozne
    indikovat opakovany stisk tlacitka v jedne casti menu (1 pozice stavoveho automatu).
*/
{
   if(press_enter)                                  //Osetreni stisknuti tlacitka ENTER
	{
		enter_control_OK ++;
		if(enter_control_OK == 4)
			enter_control ++;
	}

	else if(press_esc)                              //Osetreni stisknuti tlacitka ESC
	{
		esc_control_OK ++;
		if(esc_control_OK == 4)
			esc_control = 1;
	}

	else if(press_left)                             //Osetreni stisknuti tlacitka VLEVO
	{
		left_control_OK ++;
		if(left_control_OK == 4)
			left_control = 1;
	}

	else if(press_right)                            //Osetreni stisknuti tlacitka VPRAVO
	{
		right_control_OK ++;
		if(right_control_OK == 4)
			right_control = 1;
	}

	else                                            //Smazani obsahu vsech priznakovych promennych pro moznost opetovneho pouziti
	{
		enter_control_OK = 0;
		esc_control_OK = 0;
		left_control_OK = 0;
		right_control_OK = 0;
	}
}




/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*
  *   Funkce pro obsluhu preruseni (ISR):
  ***************************************
  *   INT0_vect == Preruseni vyvolane zmenou logicke urovne na pinu ECHO1
  *   INT1_vect == Preruseni vyvolane zmenou logicke urovne na pinu ECHO2
  *   ADC_vect == Preruseni pro zahajeni AD prevodu a nasledne zpracovani ziskanych dat
  */
/* ------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/* Funkce INT0 -------------------------------------------------------------------*/
ISR(INT0_vect)
/**
    * @brief Preruseni vyvolane zmenou logicke urovne na pinu ECHO1 (INT0_vect)
    * @brief ******************************************************************
    * @details Toto preruseni slouzi k mereni doby trvani pinu ECHO(1) v logicke urovni 1.
    * @details Je-li pin ECHO(1) nastaven na logickou hodnotu 1, je registr citace TC0 nastaven na 0 a pomocna promenna pro pocet preteceni tohoto citace je take nastavena na 0.
    * @details Ridici registr tohoto preruseni je nastaven tak, aby preruseni bylo vyvolane zmenou urovne pinu ECHO(1) na 0 a obsluha preruseni je ukoncena.
    * @details Preruseni je opet vyvolano pri prechodu logicke urovne pinu ECHO(1) na  logickou hodnotu 0.
    * @details Pocet preteceni citace TC0 je ulozen do pomocne promenne a ridici registr je opet nastaven tak,
    aby preruseni bylo vyvolano pri prechodu logicke urovne z 0 na logickou uroven 1 na pinu ECHO(1).
    * @details Obsluha preruseni je opet ukoncena.
*/
{
    if (bit_is_set(PIND,2))                     //Pin ECHO(1) je na urovni 1 (ECHO1 je pripojeno na PD2)
    {
       TCNT0 = 0;                                   //Vynulovani casovace 0
       number0_of_overflow=0;                       //Vynulovani poctu preteceni citace 0

       EICRA |= _BV(ISC01);                         //Nastaveni reakce INT0 na sestupnou hranu pinu
       EICRA &= ~_BV(ISC00);
    }

    else                                        //Pin ECHO(1) je na urovni 1
        {
           timer0_value = number0_of_overflow;      //Nacteni poctu preteceni TC0 do pomocne promenne

           EICRA |= _BV(ISC01) | _BV(ISC00);        //Nastaveni reakce INT0 na nastupnou hranu pinu
        }
}



/* Funkce INT1 ------------------------------------------------------------------*/
ISR(INT1_vect)
/**
    * @brief Preruseni vyvolane zmenou logicke urovne na pinu ECHO2 (INT1_vect)
    * @brief ******************************************************************
    * @details Toto preruseni slouzi k mereni doby trvani pinu ECHO(2) v logicke urovni 1.
    * @details Je-li pin ECHO(1) nastaven na logickou hodnotu 1, je registr citace TC0 nastaven na 0 a pomocna promenna pro pocet preteceni tohoto citace je take nastavena na 0.
    * @details Ridici registr tohoto preruseni je nastaven tak, aby preruseni bylo vyvolane zmenou urovne pinu ECHO(2) na 0 a obsluha preruseni je ukoncena.
    * @details Preruseni je opet vyvolano pri prechodu logicke urovne pinu ECHO(2) na  logickou hodnotu 0.
    * @details Pocet preteceni citace TC0 je ulozen do pomocne promenne a ridici registr je opet nastaven tak,
    aby preruseni bylo vyvolano pri prechodu logicke urovne z 0 na logickou uroven 1 na pinu ECHO(2).
    * @details Obsluha preruseni je opet ukoncena.
*/
{
    if (bit_is_set(PIND,3))                     //Pin ECHO(2) je na urovni 1 (ECHO2 je pripojeno na PD3)
    {
       TCNT0 = 0;                                   //Vynulovani casovace 1
       number0_of_overflow=0;                       //Vynulovani poctu preteceni citace 2

       EICRA |= _BV(ISC11);                         //Nastaveni reakce INT1 na sestupnou hranu pinu
       EICRA &= ~_BV(ISC10);
    }

    else                                        //Pin ECHO(2) je na urovni 1
        {
           timer0_value = number0_of_overflow;      //Nacteni poctu preteceni TC2 do pomocne promenne

           EICRA |= _BV(ISC11) | _BV(ISC10);        //Nastaveni reakce INT1 na nastupnou hranu pinu
        }
}



/* Funkce AD prevodniku 4 -------------------------------------------------------*/
ISR(ADC_vect)                   //Funkce pro mereni teploty pomoci termistoru pripojeneho na AD prevodnik
/**
    * @brief Preruseni pro zahajeni AD prevodu a nasledne zpracovani ziskanych dat (ADC_vect)
    * @brief ********************************************************************************
    * @details Nejprve je bitova hodnota ziskana AD prevodem ulozena do pomocne promenne. Tato hodnota odpovida napeti na termistoru pripojenem na pin AD4.
    * @details Ziskana hodnota napeti je prepoctena na odpor pouziteho termistoru.
    * @details Merena teplota je ziskana vypoctem z odporu termistoru, odporu pomocneho rezistoru a materialovych vlastnosti pouziteho termistoru.
    * @details Hodnota teploty ziskana vypoctem pomoci vzorce pro termodynamicky odpor je teplota v Kelvinech, proto je hodnota prepoctena na teplotu ve stupnich Celsia.
*/
{
    voltage = ADC;                          //Nacteni 10-bitove hodnoty AD prevodniku odpovidajici hodnote napeti na tomto pinu do pomocne promenne
    voltage = 1023 / voltage - 1;
    voltage = rezistor / voltage;           //Prepocet zmereneho napeti na odpor termistoru


        /* Vypocet teploty pomoci upraveneho vzorce pro termodynamicky odpor*/

            temperature = voltage / resistance_ref;             // (R/Ro)
            temperature = log(temperature);                     // ln(R/Ro)
            temperature /= beta;                                // 1/B * ln(R/Ro)
            temperature += 1.0 / (temperature_ref + 273.15);    // + (1/To)
            temperature = 1.0 / temperature;                    // 1/T  Vypoctena hodnota teploty v Kelvinech
            temperature -= 273.15;                              // Prevod hodnoty teploty na stupne Celsia
}



/* KONEC ******************************************************************************************************************************************************/
