#ifndef SCREEN_H
#define SCREEN_H

void screen_command(uint8_t);
void screen_commands(uint8_t* x, uint8_t size);
void screen_data(uint8_t* data, uint8_t size);
void screen_display(void);
uint8_t screen_init(void);
void screen_clear(void);
void  screen_drawchar(uint8_t x, uint8_t line, char c);
void screen_drawstring(uint8_t x, uint8_t line, const char *str);
void screen_drawbignum(uint8_t x, uint8_t line, char n);
void screen_drawbignums(uint8_t x, uint8_t line, char* n);

#endif //SCREEN_H
