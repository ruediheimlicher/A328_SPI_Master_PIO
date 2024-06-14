#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
//#include "defines.h"
#include <stdint.h>
#include <util/twi.h>

#include "lcd.c"
#include "adc.c"
#include <SPI.h>
// ../../../../csrc
#include <U8x8lib.h>
#include "u8g2lib.h"
//#include <MUIU8g2.h>

#include "OLED_Driver.h"
#include "DEV_Config.h"

#define LOOPLEDPORT		PORTC
#define LOOPLEDDDR		DDRC
#define LOOPSTEP			0x0FFF

// Define fuer Slave:
#define LOOPLED			0

#define  Error   0x01
#define  Success 0x02
#define SLAVE_COMMAND 2
char     TransmitState = 0x00;
//char*    TextString    = "AVR communicating via the SPI"+0x00;
#define BUFSIZE 8

// HW SPI
#define SPI_CONTROL_DDR			DDRB
#define SPI_CONTROL_PORT		PORTB
#define SPI_CONTROL_CS			PORTB2	//CS fuer HomeCentral Master
#define SPI_CONTROL_MOSI		PORTB3
#define SPI_CONTROL_MISO		PORTB4
#define SPI_CONTROL_SCK			PORTB5

#define waitspi() while(!(SPSR&(1<<SPIF)))
volatile unsigned char incoming[BUFSIZE];
volatile short int received=0;

/*
// SW_SPI
#define SW_SPI_CONTROL_DDR		DDRD
#define SW_SPI_CONTROL_PORT	PORTD
#define SW_SPI_CONTROL_CS		PORTD4	//CS fuer HomeCentral Master
#define SW_SPI_CONTROL_MOSI		PORTD5
#define SW_SPI_CONTROL_MISO		PORTD6
#define SW_SPI_CONTROL_SCK		PORTD7
*/
//U8G2_SSD1327_WS_128X128_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);

//https://github.com/olikraus/u8g2/issues/1711


//U8X8_SSD1327_WS_128X128_SW_I2C u8x8(A4,A5);

// https://github.com/olikraus/u8g2/wiki/u8x8setupcpp#ssd1327-ws_128x128

U8X8_SSD1327_WS_128X128_HW_I2C u8x8(A4,A5);

uint16_t loopcounter0 = 0;
uint16_t loopcounter1 = 0;
uint8_t h = 60;
uint8_t wertcounter = 0;
uint8_t wert = 0;
uint8_t waitcounter = 0;

void Init_Master_IntContr (void)
{
	volatile char IOReg;
	SPI_CONTROL_DDR    &= ~(1<<SPI_CONTROL_MISO);	// MISO als Input
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_MISO);		// HI

	SPI_CONTROL_DDR	|= (1<<SPI_CONTROL_MOSI);	// MOSI als Ausgang
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_MOSI);		// HI

	
	SPI_CONTROL_DDR	|= (1<<SPI_CONTROL_CS);	// Chip Select als Ausgang
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_CS);		// HI
	SPI_CONTROL_DDR	|= (1<<SPI_CONTROL_SCK);	// SCK Ausgang
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_SCK);		// HI
	
	// Enable SPI Interrupt and SPI in Master Mode with SCK = CK/4
	SPCR = (1<<SPE)|(1<<MSTR);
	IOReg   = SPSR;                         // Clear SPIF bit in SPSR
	IOReg   = SPDR;
	lcd_gotoxy(0,0);
	lcd_puts("SPI Init A\0");

	//DDRD	= 0xFF;	
	// Set Port D as output
	sei(); // Enable global interrupts
}

unsigned char spi_tranceiver (unsigned char data)
{
	SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_CS);		
    // Load data into the buffer
    SPDR = data;
		waitcounter = 0;
    //Wait until transmission complete
    while(!(SPSR & (1<<SPIF))&& waitcounter < 0xFFF)
		{
			waitcounter++;
		}   // Return received data
	SPI_CONTROL_PORT |=(1<<SPI_CONTROL_CS);		
  return(SPDR);
}

uint8_t received_from_spi(uint8_t data)
{
  SPDR = data;
  return SPDR;
}

void parse_message()
{

 switch(incoming[0]) 
 {
 case SLAVE_COMMAND:
   //flash_led(incoming[1])
	;
   break;
 default:
   PORTD ^=(1<<1);//LED 1 toggeln
	;
 }

}

/*
// Interrupt Routine Slave Mode (interrupt controlled)
 
// called by the SPI system when there is data ready.
// Just store the incoming data in a buffer, when we receive a
// terminating byte (0x00) call parse_message to process the data received
ISR( SPI_STC_vect )
{
	lcd_clr_line(1);
	lcd_gotoxy(8,1);
	lcd_puts("ISR \0");
	lcd_gotoxy(12,1);
	lcd_puts("    \0");
	lcd_gotoxy(12,1);
	PORTD ^=(1<<0);//LED 0 toggeln
	lcd_puthex(received_from_spi(0x00));
	received++;
	lcd_puthex(received);
  incoming[received++] = received_from_spi(0x00);
  
  if (received >= BUFSIZE ) //|| incoming[received-1] == 0x00) 
  {
    parse_message();
    received = 0;
  }

 


}
*/

void slaveinit(void)
{
   /*
 	DDRD |= (1<<DDD0);		//Pin 0 von PORT D als Ausgang fuer Schalter: ON		
	DDRD |= (1<<DDD1);		//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
	DDRD |= (1<<DDD2);		//Pin 2 von PORT D als Ausgang fuer Buzzer
 	DDRD |= (1<<DDD3);		//Pin 3 von PORT D als Ausgang fuer LED TWI
	DDRD |= (1<<DDD4);		//Pin 4 von PORT D als Ausgang fuer LED
    */
   LOOPLEDDDR |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop
	LOOPLEDPORT |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop

	/*
	DDRB &= ~(1<<PB0);	//Bit 0 von PORT B als Eingang für Taste 1
	PORTB |= (1<<PB0);	//Pull-up

	DDRB &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang für Taste 2
	PORTB |= (1<<PB1);	//Pull-up
	*/

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
/*
	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC0); //Pull-up
	DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC3); //Pull-up
	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
//	PORTC |= (1<<DDC3); //Pull-up
*/


	
	
}


int main (void) 
{
    wdt_disable();
		/*
    MCUSR &= ~(1<<WDRF);
    wdt_reset();
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 0x00;
*/
	  slaveinit();
	   
	  //uint16_t ADC_Wert= readKanal(0);
		
	  // initialize the LCD 
	  lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	  //lcd_puts("Guten Tag I2C");
	  //_delay_ms(1000);
	 
	 Init_Master_IntContr();


		 System_Init();
  //Serial.print(F("OLED_Init()...\r\n"));
	lcd_puts("OLED_Init");
  //OLED_1in5_Init();
  //_delay_ms(500); 
  //OLED_1in5_Clear();  

	uint16_t loopcount0 = 0;
	uint16_t loopcount1 = 0;
	uint16_t loopcount2 = 0;
 uint16_t spi_sendcounter = 0;
 uint8_t spidata = 0;
	   //timer2();
   
	//u8x8.begin();
	while (1)
	{
      //PORTD ^= (1<<0);
      //_delay_ms(50);
     // PORTD &= ~(1<<0);
		//Blinkanzeige
      wdt_reset();
		loopcount0++;
		if (loopcount0>LOOPSTEP)
		{
			loopcount0=0;
			//LOOPLEDPORT ^=(1<<LOOPLED);
			
      loopcount1++;
      if(loopcount1 > 0x2F)
         {
            LOOPLEDPORT ^=(1<<LOOPLED);
            loopcount1 = 0;
            loopcount2++;
            lcd_gotoxy(0,2);
            lcd_putint(loopcount2);
						lcd_putc(' ');
						lcd_putint12(spidata);
						uint8_t spi_result = spi_tranceiver(spidata++);
         		lcd_putc(' ');
						lcd_putint(spi_result);
						lcd_putc(' ');
						lcd_putint(waitcounter);
						//u8x8.setFont(u8x8_font_chroma48medium8_r);
  					//u8x8.drawString(0,1,"Hello World!");
				 }
		}
		

		
	}//while


 return 0;
}// main
