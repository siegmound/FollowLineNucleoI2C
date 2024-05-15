#include "liquidcrystal_i2c.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t dpFunction;
uint8_t dpControl;
uint8_t dpMode;
uint8_t dpRows;
uint8_t dpBacklight;

static void SendCommand(uint8_t);
static void SendChar(uint8_t);
static void Send(uint8_t, uint8_t);
static void Write4Bits(uint8_t);
static void ExpanderWrite(uint8_t);
static void PulseEnable(uint8_t);
static void DelayInit(void);
static void DelayUS(uint32_t);

uint8_t arrowL1[8] = {
        0b00000,
        0b00001,
        0b00011,
        0b01111,
        0b11111,
        0b01111,
        0b00011,
        0b00001
};

uint8_t arrowL2[8] = {
        0b00000,
        0b00000,
        0b00000,
        0b11000,
        0b11100,
        0b11110,
        0b11110,
        0b01110
};
uint8_t VPiece[8] = {
		0b01110,
		0b01110,
		0b01110,
		0b01110,
		0b01110,
		0b01110,
		0b01110,
		0b01110

};
uint8_t HPiece[8] = {
		0b00000,
		0b00000,
		0b00000,
		0b11111,
		0b11111,
		0b11111,
		0b00000,
		0b00000
};
uint8_t full[8] = {
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b11111
};
uint8_t mup[8] = {
		0b11111,
		0b11111,
		0b11111,
		0b11111,
		0b00000,
		0b00000,
		0b00000,
		0b00000
};
uint8_t mdown[8] = {
		0b00000,
		0b00000,
		0b00000,
		0b00000,
		0b11111,
		0b11111,
		0b11111,
		0b11111
};
uint8_t rside[8] = {
		0b00111,
		0b00111,
		0b00111,
		0b00111,
		0b00111,
		0b00111,
		0b00111,
		0b00111
};
uint8_t lside[8] = {
		0b11100,
		0b11100,
		0b11100,
		0b11100,
		0b11100,
		0b11100,
		0b11100,
		0b11100
};
uint8_t mario6[8] = {
		0b11100,
		0b01010,
		0b11001,
		0b10101,
		0b10010,
		0b00010,
		0b00100,
		0b11000
};
uint8_t bird[8] = {
		0b00000,
		0b00000,
		0b00010,
		0b00110,
		0b11111,
		0b00110,
		0b00010,
		0b00000
};
uint8_t moob[8] = {
		0b00000,
		0b01110,
		0b11111,
		0b10101,
		0b11111,
		0b10001,
		0b01110,
		0b11011
};

void HD44780_Init(uint8_t rows)
{
  dpRows = rows;

  dpBacklight = LCD_BACKLIGHT;

  dpFunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;

  if (dpRows > 1)
  {
    dpFunction |= LCD_2LINE;
  }
  else
  {
    dpFunction |= LCD_5x10DOTS;
  }

  /* Wait for initialization */
  DelayInit();
  HAL_Delay(50);

  ExpanderWrite(dpBacklight);
  HAL_Delay(1000);

  /* 4bit Mode */
  Write4Bits(0x03 << 4);
  DelayUS(4500);

  Write4Bits(0x03 << 4);
  DelayUS(4500);

  Write4Bits(0x03 << 4);
  DelayUS(4500);

  Write4Bits(0x02 << 4);
  DelayUS(100);

  /* Display Control */
  SendCommand(LCD_FUNCTIONSET | dpFunction);

  dpControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  HD44780_Display();
  HD44780_Clear();

  /* Display Mode */
  dpMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
  DelayUS(4500);

  HD44780_CreateSpecialChar(0, rside);
  HD44780_CreateSpecialChar(1, lside);
  HD44780_CreateSpecialChar(2, arrowL2);
  HD44780_CreateSpecialChar(3, VPiece);
  HD44780_CreateSpecialChar(4, HPiece);
  HD44780_CreateSpecialChar(5, full);
  HD44780_CreateSpecialChar(6, mup);
  HD44780_CreateSpecialChar(7, mdown);

  HD44780_Home();
}

void HD44780_Clear()
{
  SendCommand(LCD_CLEARDISPLAY);
  DelayUS(2000);
}

void HD44780_Home()
{
  SendCommand(LCD_RETURNHOME);
  DelayUS(2000);
}

void HD44780_SetCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if (row >= dpRows)
  {
    row = dpRows-1;
  }
  SendCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void HD44780_NoDisplay()
{
  dpControl &= ~LCD_DISPLAYON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_Display()
{
  dpControl |= LCD_DISPLAYON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_NoCursor()
{
  dpControl &= ~LCD_CURSORON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_Cursor()
{
  dpControl |= LCD_CURSORON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_NoBlink()
{
  dpControl &= ~LCD_BLINKON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_Blink()
{
  dpControl |= LCD_BLINKON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_ScrollDisplayLeft(void)
{
  SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void HD44780_ScrollDisplayRight(void)
{
  SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void HD44780_LeftToRight(void)
{
  dpMode |= LCD_ENTRYLEFT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_RightToLeft(void)
{
  dpMode &= ~LCD_ENTRYLEFT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_AutoScroll(void)
{
  dpMode |= LCD_ENTRYSHIFTINCREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_NoAutoScroll(void)
{
  dpMode &= ~LCD_ENTRYSHIFTINCREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_CreateSpecialChar(uint8_t location, uint8_t charmap[])
{
  location &= 0x7;
  SendCommand(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++)
  {
    SendChar(charmap[i]);
  }
}

void HD44780_Create1(uint8_t location, uint8_t charmap[][8]) {
    location &= 0x7;
    SendCommand(LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 8; j++) {
            SendChar(charmap[i][j]);
        }
    }
}
void HD44780_PrintSpecialChar(uint8_t index)
{
  SendChar(index);
}

void HD44780_LoadCustomCharacter(uint8_t char_num, uint8_t *rows)
{
  HD44780_CreateSpecialChar(char_num, rows);
}

void HD44780_PrintStr(const char c[])
{
  while(*c) SendChar(*c++);
}

void HD44780_SetBacklight(uint8_t new_val)
{
  if(new_val) HD44780_Backlight();
  else HD44780_NoBacklight();
}

void HD44780_NoBacklight(void)
{
  dpBacklight=LCD_NOBACKLIGHT;
  ExpanderWrite(0);
}

void HD44780_Backlight(void)
{
  dpBacklight=LCD_BACKLIGHT;
  ExpanderWrite(0);
}

static void SendCommand(uint8_t cmd)
{
  Send(cmd, 0);
}

static void SendChar(uint8_t ch)
{
  Send(ch, RS);
}

static void Send(uint8_t value, uint8_t mode)
{
  uint8_t highnib = value & 0xF0;
  uint8_t lownib = (value<<4) & 0xF0;
  Write4Bits((highnib)|mode);
  Write4Bits((lownib)|mode);
}

static void Write4Bits(uint8_t value)
{
  ExpanderWrite(value);
  PulseEnable(value);
}

static void ExpanderWrite(uint8_t _data)
{
  uint8_t data = _data | dpBacklight;
  HAL_I2C_Master_Transmit(&hi2c1, DEVICE_ADDR, (uint8_t*)&data, 1, 10);
}

static void PulseEnable(uint8_t _data)
{
  ExpanderWrite(_data | ENABLE);
  DelayUS(20);

  ExpanderWrite(_data & ~ENABLE);
  DelayUS(20);
}

static void DelayInit(void)
{
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;

  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  DWT->CYCCNT = 0;

  /* 3 NO OPERATION instructions */
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
}

static void DelayUS(uint32_t us) {
  uint32_t cycles = (SystemCoreClock/1000000L)*us;
  uint32_t start = DWT->CYCCNT;
  volatile uint32_t cnt;

  do
  {
    cnt = DWT->CYCCNT - start;
  } while(cnt < cycles);
}
