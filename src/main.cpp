#include <Arduino.h> 
 #include <avr/io.h>
 #include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>
//#include "defines.h"
#include <stdint.h>

#include "lcd.c" // LCD-Stuff

#include <U8g2lib.h>
#include <U8x8lib.h>

//U8X8_SSD1327_WS_128X128_HW_I2C u8x8(A4,A5); // ok

U8X8_SSD1327_WS_128X128_HW_I2C u8x8(U8X8_PIN_NONE);

#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR      DDRD
#define LOOPLED			5

volatile uint8_t   loopstatus=0x00;            

uint16_t loopcount0=0;
uint16_t loopcount1=0;
uint16_t loopcount2=0;

uint16_t refreshtakt = 0xBF;



void slaveinit(void)
{   
   LOOPLEDDDR |= (1<<LOOPLED);

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

}  
    
int main (void) 
{
	slaveinit();  
 	// initialize the LCD 
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("A328_PIO_Start");
	
	_delay_ms(200);
  u8x8.begin(); // blocks loop
  _delay_ms(200);
   u8x8.setFont(u8x8_font_5x7_f);
  sei();
	while (1)
       {  
         loopcount0++;
         if (loopcount0>=refreshtakt)
         {
            loopcount0=0;
            loopcount1++;
            if (loopcount1>=refreshtakt)
            {
							
  						u8x8.drawString(0,0,"A");
  						//delay(1000);
               loopcount1 = 0;
               LOOPLEDPORT ^= (1<<LOOPLED); 
               lcd_gotoxy(0,1);
               lcd_putint(loopcount2);
               loopcount2++;
            }           
            loopcount0=0;
         }  // loopcount0>=refreshtakt
   }//while
 return 0;
}