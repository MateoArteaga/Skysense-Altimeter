#include "main.h"
#include "font.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver_bmp390.h"
#include <stdarg.h>
#include <math.h>
#include "screen.h"

#define SLAVE_ADDRESS  0x78

uint8_t* buffer;

uint8_t screen_init()
{
	  if ((!buffer) && !(buffer = (uint8_t *)malloc(513)))
	  {
	    return 0;
	  }
	  uint8_t data_buffer[7];
	  data_buffer[1] = 0xAE; //display off
	  data_buffer[2] = 0xD5; //Clock Ratio
	  data_buffer[3] = 0x80; //suggested clock ratio
	  data_buffer[4] = 0xA8; //Set multiplex
	  data_buffer[5] = 0x1F; //31 (height -1 of display)
	  screen_commands(data_buffer, 5);
	  data_buffer[1] = 0xD3; //set display offset
	  data_buffer[2] = 0x00; //set it to no offset
	  data_buffer[3] = 0x40; //Start line #0
	  data_buffer[4] = 0x8D; //disable charge pump
	  data_buffer[5] = 0x14; //enable charge pump
	  screen_commands(data_buffer, 5);
	  data_buffer[1] = 0x20; //memory mode
	  data_buffer[2] = 0x00; //Horizontal addressing mode
	  data_buffer[3] = 0xA1; //column address 127 is mapped to SEG0
	  data_buffer[4] = 0xC0; //COM output scan direction from COM[N-1] to COM0
	  screen_commands(data_buffer, 4);
	  screen_command(0xDA); //set COM pins
	  screen_command(0x02); //COM pins sequential, Disable re-map.
	  screen_command(0x81); //set contrast
	  screen_command(0x8F); //contrast to 143 / 256
	  screen_command(0xD9); //set pre-charge period
	  screen_command(0xF1); //internal VCC
	  data_buffer[1] = 0xD8; //SETVCOMDETECT
	  data_buffer[2] = 0x40; //set vcomdetect
	  data_buffer[3] = 0xA4; //display all on resume
	  data_buffer[4] = 0xA6; //normal display
	  data_buffer[5] = 0x2E; //stop scroll
	  data_buffer[6] = 0xAF; //Display on
	  screen_commands(data_buffer, 6);
	  return 1;
}
void screen_command(uint8_t x)
{
	uint8_t command[2];
	command[0] = 0x00;
	command[1] = x;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVE_ADDRESS, command,2 , HAL_MAX_DELAY);
}
void screen_commands(uint8_t* buffer, uint8_t size)
{
	buffer[0] = 0x00;
	uint8_t amount_data = size + 1;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVE_ADDRESS, buffer, amount_data , HAL_MAX_DELAY);
}
void screen_data(uint8_t* buffer, uint8_t size)
{
	buffer[0] = 0x40;
	uint8_t amount_data = size + 1;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SLAVE_ADDRESS, buffer, amount_data, HAL_MAX_DELAY);
}
void screen_display(void)
{
	uint8_t data_buffer[7] = {
			0x00, //control byte
			0x22, //set page address
			0x00,
			0xFF,
			0x21, //set column address
			0x00,
			0x7F //end column address 127d
	};
	screen_commands(data_buffer, 6);
    for (int i = 0; i < 512; i += 16) {
    		screen_data(&buffer[i], 16);
    	}
}
void screen_clear(void)
{
	memset(buffer, 0, 513); //fill buffer with 0's
}

void  screen_drawbignums(uint8_t x, uint8_t line, char *n)
{
	  while (n[0] != 0) {
		screen_drawbignum(x, line, n[0]);
	    n++;
	    x += 17; // 17 pixels wide
	    if (x + 17 >= 128) {
	      x = 0;    // ran out of this line
	      line++;
	    }
	    if (line >= (32/8))
	      return;        // ran out of space :(
	  }
}
void  screen_drawbignum(uint8_t x, uint8_t line, char n) {
  uint8_t i;
  uint8_t start = x;
  uint8_t start1 = x;
  if (line > 2)
  {
	  line = 2;
  }
  for (i =0; i<45; i++ ) {
	  if (i < 15)
	  {
		  	buffer[x + (128* (4 - line)) ] = bigfont[(((unsigned char)n - 48) * 45) + i];
		  	x++;
	  }
	  else if (i < 30)
	  {
		    buffer[start + (128* (3 - line)) ] = bigfont[(((unsigned char)n - 48) * 45) + i];
		    start++;
	  }
	  else if (i < 45)
	  {
		    buffer[start1 + (128* (2 - line)) ] = bigfont[(((unsigned char)n - 48) * 45) + i];
		    start1++;
	  }
  }
}

void  screen_drawchar(uint8_t x, uint8_t line, char c)
{
  if (line > 4)
  {
	  line = 4;
  }
  if (x > 123)
  {
	  x = 123;
  }
  uint8_t i;
  for (i =0; i<5; i++ ) {
    buffer[x + (128* ( 4 - line))] = font[((unsigned char)c * 5) + i];
    x++;
}
}


void screen_drawstring(uint8_t x, uint8_t line, const char *str) {
  char c;
  while (1) {
    c = *str++;
    if (! c)
      return;
    screen_drawchar(x, line, c);
    x += 6; // 6 pixels wide
    if (x + 6 >= 128) {
      x = 0;    // ran out of this line
      line++;
    }
    if (line > 4)
      return;        // ran out of space :(
  }
}
