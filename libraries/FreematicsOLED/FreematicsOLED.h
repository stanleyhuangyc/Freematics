/*************************************************************************
* Simple OLED Text & Bitmap Display Library for Arduino
* Distributed under BSD
* (C)2013-2018 Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>

typedef enum {
    FONT_SIZE_SMALL = 0,
    FONT_SIZE_MEDIUM,
    FONT_SIZE_LARGE,
    FONT_SIZE_XLARGE
} FONT_SIZE;

#define FLAG_PAD_ZERO 1
#define FLAG_PIXEL_DOUBLE_H 2
#define FLAG_PIXEL_DOUBLE_V 4
#define FLAG_PIXEL_DOUBLE (FLAG_PIXEL_DOUBLE_H | FLAG_PIXEL_DOUBLE_V)

extern const unsigned char font5x8[][5];
extern const unsigned char digits8x8[][8] ;
extern const unsigned char digits16x16[][32];
extern const unsigned char digits16x24[][48];
extern const unsigned char font8x16_doslike[][16];
extern const unsigned char font8x16_terminal[][16];
extern const uint8_t bitmap_tick[];
extern const uint8_t bitmap_cross[];

class LCD_Common
{
public:
    void setFontSize(FONT_SIZE size) { m_font = size; }
    void setFlags(byte flags) { m_flags = flags; }
    virtual void backlight(bool on) {}
    virtual void draw(const byte* buffer, byte width, byte height) {}
    void printInt(uint16_t value, int8_t padding = -1);
    void printLong(uint32_t value, int8_t padding = -1);
protected:
    virtual void writeDigit(byte n) {}
    byte m_font = FONT_SIZE_SMALL;
    byte m_flags = 0;
};

class OLED_SH1106 : public LCD_Common, public Print
{
public:
    void begin();
    void setCursor(byte column, byte line);
    void draw(const byte* buffer, byte width, byte height);
    size_t write(uint8_t c);
    void clear(byte x = 0, byte y = 0, byte width = 128, byte height = 64);
    void clearLine(byte line);
    byte getLines() { return 21; }
    byte getCols() { return 8; }
private:
    void WriteCommand(unsigned char ins);
    void WriteData(unsigned char dat);
    void writeDigit(byte n);
    byte m_col = 0;
    byte m_row = 0;
};

