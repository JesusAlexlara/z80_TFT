#ifndef TFTz80_H_
#define TFTz80_H_

#define TFTWIDTH 240
#define TFTHEIGHT 320

#define TFTLCD_DELAY 0xFF

#define TOUCHA PP1_PORTB |= BV(PB7)
#define TOUCHB PP1_PORTB |= BV(PB6)

#define RESET_LOW PPI_PORTA &= ~BV(PA4)
#define RESET_HIGH PPI_PORTA |= BV(PA4)
#define RD_ACTIVE PPI_PORTA &= ~BV(PA0)
#define RD_IDLE PPI_PORTA |= BV(PA0)
#define WR_ACTIVE PPI_PORTA &= ~BV(PA1)
#define WR_IDLE PPI_PORTA |= BV(PA1)
#define CD_COMMAND PPI_PORTA &= ~BV(PA2)
#define CD_DATA PPI_PORTA |= BV(PA2)
#define CS_ACTIVE PPI_PORTA &= ~BV(PA3)
#define CS_IDLE PPI_PORTA |= BV(PA3)
#define RD_STROBE { RD_ACTIVE; RD_IDLE; }
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }


#define	BLACK   0x0000  // Definimos los colores para poder referirnos a ellos con su nombre                  
#define	RED     0xF800  // en lugar de usar el código hexadecimal de cada uno. 
#define	GREEN   0x07E0 
#define WHITE   0xFFFF  
#define BLUE    0x001F 
#define CYAN    0x07FF
#define YELLOW  0xFFE0
#define MAGENTA 0xF81F


static const uint16_t PROGMEM[] = {
  ILI932X_START_OSC        , 0x0001, // Start oscillator //00
  TFTLCD_DELAY             , 50,     // 50 millisecond delay 01
  ILI932X_DRIV_OUT_CTRL    , 0x0100,	//01
  ILI932X_DRIV_WAV_CTRL    , 0x0700,	//02
  ILI932X_ENTRY_MOD        , 0x1030,	//03
  ILI932X_RESIZE_CTRL      , 0x0000,	//04
  ILI932X_DISP_CTRL2       , 0x0202,	//08
  ILI932X_DISP_CTRL3       , 0x0000,	//09
  ILI932X_DISP_CTRL4       , 0x0000,	//0A
  ILI932X_RGB_DISP_IF_CTRL1, 0x0,		//0C
  ILI932X_FRM_MARKER_POS   , 0x0,		//0D
  ILI932X_RGB_DISP_IF_CTRL2, 0x0,		//0F
  ILI932X_POW_CTRL1        , 0x0000,	//10
  ILI932X_POW_CTRL2        , 0x0007,	//11
  ILI932X_POW_CTRL3        , 0x0000,	//12
  ILI932X_POW_CTRL4        , 0x0000,	//13
  TFTLCD_DELAY             , 200,		//
  ILI932X_POW_CTRL1        , 0x1690,	//10
  ILI932X_POW_CTRL2        , 0x0227,	//11
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL3        , 0x001A,	//12
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL4        , 0x1800,	//13
  ILI932X_POW_CTRL7        , 0x002A,	//29
  TFTLCD_DELAY             , 50,
  ILI932X_GAMMA_CTRL1      , 0x0000,	//30
  ILI932X_GAMMA_CTRL2      , 0x0000,	//31
  ILI932X_GAMMA_CTRL3      , 0x0000,	//32
  ILI932X_GAMMA_CTRL4      , 0x0206,	//35
  ILI932X_GAMMA_CTRL5      , 0x0808,	//36
  ILI932X_GAMMA_CTRL6      , 0x0007,	//37
  ILI932X_GAMMA_CTRL7      , 0x0201,	//38
  ILI932X_GAMMA_CTRL8      , 0x0000,	//39
  ILI932X_GAMMA_CTRL9      , 0x0000,	//3C
  ILI932X_GAMMA_CTRL10     , 0x0000,	//3D
  ILI932X_GRAM_HOR_AD      , 0x0000,	//20
  ILI932X_GRAM_VER_AD      , 0x0000,	//21
  ILI932X_HOR_START_AD     , 0x0000,	//50
  ILI932X_HOR_END_AD       , 0x00EF,	//51
  ILI932X_VER_START_AD     , 0X0000,	//52
  ILI932X_VER_END_AD       , 0x013F,	//53
  ILI932X_GATE_SCAN_CTRL1  , 0xA700, // 60 Driver Output Control (R60h)
  ILI932X_GATE_SCAN_CTRL2  , 0x0003, // 61 Driver Output Control (R61h)
  ILI932X_GATE_SCAN_CTRL3  , 0x0000, // 6A Driver Output Control (R62h)
  ILI932X_PANEL_IF_CTRL1   , 0X0010, // 90 Panel Interface Control 1 (R90h)
  ILI932X_PANEL_IF_CTRL2   , 0X0000,	//92
  ILI932X_PANEL_IF_CTRL3   , 0X0003,	//93
  ILI932X_PANEL_IF_CTRL4   , 0X1100,	//95
  ILI932X_PANEL_IF_CTRL5   , 0X0000,	//97
  ILI932X_PANEL_IF_CTRL6   , 0X0000,	//98
  ILI932X_DISP_CTRL1       , 0x0133, //07 Main screen turn on
};

void system_init();
void lcd_init();
void writeRegister16(uint16_t a, uint16_t d);
void write8(uint8_t d);
void writeRegister24(uint8_t r, uint32_t d);
void writeRegister32(uint8_t r, uint32_t d);
void setAddrWindow(int x1, int y1, int x2, int y2);
void flood(uint16_t color, uint32_t len);
void fillScreen(uint16_t color);
void reset();

//pintar pantalla
void init_pantalla();
void pintaSprite(int _x, int _y, int spr);
void pintaAzul();
void pintaNegro();
void pintaBarra(int i);
void drawPixel(int16_t x, int16_t y, uint16_t color);
void fillRect(int16_t x1, int16_t y1, int16_t w, int16_t h, 
uint16_t fillcolor); 
uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
void drawGraf();

void repinta_rana(int i);

#endif
