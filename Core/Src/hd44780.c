
#include "hd44780.h"
#include "main.h"


#define LCD_PORT7 D7_LCD_GPIO_Port
#define LCD_PORT4 D4_LCD_GPIO_Port
#define LCD_PORT56 D6_LCD_GPIO_Port //pines D5 y D6 en el mismpo puerto, GPIOB

#define RS_PORT RS_LCD_GPIO_Port
#define CLOCK_PORT E_LCD_GPIO_Port

#define LCD_RS RS_LCD_Pin
#define LCD_CLOCK E_LCD_Pin
#define LCD_4 D4_LCD_Pin
#define LCD_5 D5_LCD_Pin
#define LCD_6 D6_LCD_Pin
#define LCD_7 D7_LCD_Pin



// Various displays exist, don't make assumptions
uint8_t lcd_chars = 0;
uint8_t lcd_lines = 0;
uint8_t *lcd_line_addresses = 0;
// "Private" globals
uint8_t _lcd_char = 0;
uint8_t _lcd_line = 0;



void lcd_clock(void)
{
    // Pulse clock
	HAL_GPIO_WritePin(CLOCK_PORT, LCD_CLOCK, 1);

	HAL_Delay(1);
    HAL_GPIO_WritePin(CLOCK_PORT, LCD_CLOCK, 0);
    HAL_Delay(1);
}

void lcd_reset(void)
{
    // Resets display from any state to 4-bit mode, first nibble.

    // Set everything low first
	HAL_GPIO_WritePin(RS_PORT, LCD_RS, 0);

	HAL_GPIO_WritePin(LCD_PORT7, LCD_7, 0);
	HAL_GPIO_WritePin(LCD_PORT4, LCD_4, 0);
	HAL_GPIO_WritePin(LCD_PORT56, LCD_5, 0);
	HAL_GPIO_WritePin(LCD_PORT56, LCD_6, 0);
	HAL_GPIO_WritePin(CLOCK_PORT, LCD_CLOCK, 0);

    // Reset strategy below based on Wikipedia description, should recover
    // from any setting

    // Write 0b0011 three times
    // (Everyday Practical Electronics says 3 times, Wikipedia says 2 times,
    // 3 seems to work better).
	HAL_GPIO_WritePin(LCD_PORT4, LCD_4, 1);
	HAL_GPIO_WritePin(LCD_PORT56, LCD_5, 1);
    lcd_clock();
    lcd_clock();
    lcd_clock();
    // LCD now guaranteed to be in 8-bit state
    // Now write 0b0010 (set to 4-bit mode, ready for first nibble)
    HAL_GPIO_WritePin(LCD_PORT4, LCD_4, 0);
    lcd_clock();

    HAL_GPIO_WritePin(Led_LCD_GPIO_Port, Led_LCD_Pin, 1);
}

/* TODO This function should achieve the same as the lcd_write below, however
 * it appears to be a little problematic.
 * Rather than the LCD_4 and LCD_RS defines, direct integers have to be used
 * for proper masks to be calculated.
 * Aside from this, setting the RS bit seems to go wrong.
*/

void lcd_write(uint8_t byte, uint8_t rs)
{
    // Writes a byte to the display (rs must be either 0 or 1)
	//rs=0 comando;; rs=1 dato
    // Write second nibble and set RS

    if((byte >> 4 ) & 1)
    	HAL_GPIO_WritePin(LCD_PORT4, LCD_4, 1);
    else
    	HAL_GPIO_WritePin(LCD_PORT4, LCD_4, 0);

    if((byte >> 5 ) & 1)
    	HAL_GPIO_WritePin(LCD_PORT56, LCD_5, 1);
    else
    	HAL_GPIO_WritePin(LCD_PORT56, LCD_5, 0);

    if((byte >> 6 ) & 1)
    	HAL_GPIO_WritePin(LCD_PORT56, LCD_6, 1);
    else
    	HAL_GPIO_WritePin(LCD_PORT56, LCD_6, 0);

    if((byte >> 7 ) & 1)
    	HAL_GPIO_WritePin(LCD_PORT7, LCD_7, 1);
    else
    	HAL_GPIO_WritePin(LCD_PORT7, LCD_7, 0);

    if(rs)
    	HAL_GPIO_WritePin(RS_PORT, LCD_RS, 1);
    else
    	HAL_GPIO_WritePin(RS_PORT, LCD_RS, 0);

    lcd_clock();

    // Write first nibble

    if(byte & 1)
    	HAL_GPIO_WritePin(LCD_PORT4, LCD_4, 1);
    else
    	HAL_GPIO_WritePin(LCD_PORT4, LCD_4, 0);

    if((byte >> 1 ) & 1)
    	HAL_GPIO_WritePin(LCD_PORT56, LCD_5, 1);
    else
    	HAL_GPIO_WritePin(LCD_PORT56, LCD_5, 0);

    if((byte >> 2 ) & 1)
    	HAL_GPIO_WritePin(LCD_PORT56, LCD_6, 1);
    else
    	HAL_GPIO_WritePin(LCD_PORT56, LCD_6, 0);

    if((byte >> 3 ) & 1)
    	HAL_GPIO_WritePin(LCD_PORT7, LCD_7, 1);
    else
    	HAL_GPIO_WritePin(LCD_PORT7, LCD_7, 0);

    lcd_clock();
}

void lcd_clear(void)
{
    // Clears display, resets cursor
    lcd_write(0b00000001, 0);
    _lcd_char = 0;
    _lcd_line = 0;
}

void lcd_display_settings(uint8_t on, uint8_t underline, uint8_t blink)
{
    // "Display On/Off & Cursor" command. All parameters must be either 0 or 1

    lcd_write(0b00001000 | (on << 2) | (underline << 1) | blink, 0);
}

void lcd_display_address(uint8_t address)
{
    lcd_write(0b10000000 | address, 0);
}

void lcd_cgram_address(uint8_t address)
{
	lcd_write(0b01000000 | address, 0);
}

void lcd_print(char string[])
{
    uint8_t i;
    for(i = 0; string[i] != 0; i++) {
        // If we know the display properties and a newline character is
        // present, print the rest of the string on the new line.
        if(lcd_lines && string[i] == '\n') {
            if(_lcd_line < lcd_lines) {
                lcd_display_address(lcd_line_addresses[_lcd_line++]);
                _lcd_char = 0;
            }
        }
        else {
            // If we know the display properties and have reached the end of
            // line, print the rest on the next line
            if(lcd_chars)
                if((_lcd_char == lcd_chars) && (_lcd_line < lcd_lines)) {
                    lcd_display_address(lcd_line_addresses[_lcd_line++]);
                    _lcd_char = 0;
                }
            lcd_write(string[i], 1);
            if(lcd_chars) _lcd_char++;
        }
    }
}

void writeIntegerToLCD(int integer)
{
	//	Break down the original number into the thousands, hundreds, tens,
	//	and ones places and then immediately write that value to the LCD
	unsigned char thousands = integer / 1000;
	lcd_write( thousands + 0x30,1);

	unsigned char hundreds = (integer - thousands*1000) / 100;
	lcd_write( hundreds + 0x30,1);

	unsigned char tens = (integer - thousands*1000 - hundreds*100 ) / 10;
	lcd_write( tens + 0x30,1);

	unsigned char ones = (integer - thousands*1000 - hundreds*100 - tens*10);
	lcd_write( ones + 0x30,1);
}

void moveToXY(unsigned char row, unsigned char column)
{
	//	Determine the new position
	int position = (row * 16) + column;

	//	Send the correct commands to the command register of the LCD
	if(position < 16)
		lcd_write( 0x80 | position,0);
	else if(position >= 16 && position < 32)
		lcd_write( 0x80 | (position % 16 + 0x40),0);
	else if(position >= 41 && position < 60)
		lcd_write( 0x80 | (position % 40 + 0x14),0);
	else if(position >= 20 && position < 40)
		lcd_write( 0x80 | (position % 60 + 0x54),0);
}



