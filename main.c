/***************************************************************************
 *  main.c
 *
 *  Plantilla para un programa en C para le microprocesador Z80 con
 *  el compilador SDCC.
 *  
 *  Se incluye la librería smz80.h, que es una API con funciones para
 *  el PPI 8255, la UART 8250, funciones para puertos de E/S, funciones
 *  de delay y macros con instrucciones para el Z80 a bajo nivel. 
 *
 ***************************************************************************
 *
 *      Autor:  Alfredo Orozco de la Paz
 *      e-mail: alfredoopa@gmail.com 
 *                                                                  *FRED*
 ***************************************************************************
 */

/**
 * Definicion de las direcciones para el PPI y la UART
 * Si no se definen, no se pueden usar las funciones de la librería smz80.h.
 * 
 * se deben definir antes de incluir la librería.
 */
#define PPI_BASE_ADDR  0x40
#define UART_BASE_ADDR 0x10

#include "smz80.h"
#include "registers.h"

#define TFTWIDTH 240
#define TFTHEIGHT 320

#define TFTLCD_DELAY 0xFF

#define RESET_LOW PPI_PORTA &= ~PA4
#define RESET_HIGH PPI_PORTA |= PA4
#define RD_ACTIVE PPI_PORTA &= ~PA0
#define RD_IDLE PPI_PORTA |= PA0
#define WR_ACTIVE PPI_PORTA &= ~PA1
#define WR_IDLE PPI_PORTA |= PA1
#define CD_COMMAND PPI_PORTA &= ~PA2
#define CD_DATA PPI_PORTA |= PA2
#define CS_ACTIVE PPI_PORTA &= ~PA3
#define CS_IDLE PPI_PORTA |= PA3
#define RD_STROBE { RD_ACTIVE; RD_IDLE; }
#define WR_STROBE { WR_ACTIVE; WR_IDLE; }

static const uint16_t PROGMEM[] = {
  ILI932X_START_OSC        , 0x0001,
  TFTLCD_DELAY             , 50,
  ILI932X_DRIV_OUT_CTRL    , 0x0100,
  ILI932X_DRIV_WAV_CTRL    , 0x0700,
  ILI932X_ENTRY_MOD        , 0x1030,
  ILI932X_RESIZE_CTRL      , 0x0000,
  ILI932X_DISP_CTRL2       , 0x0202,
  ILI932X_DISP_CTRL3       , 0x0000,
  ILI932X_DISP_CTRL4       , 0x0000,
  ILI932X_RGB_DISP_IF_CTRL1, 0x0,
  ILI932X_FRM_MARKER_POS   , 0x0,
  ILI932X_RGB_DISP_IF_CTRL2, 0x0,
  ILI932X_POW_CTRL1        , 0x0000,
  ILI932X_POW_CTRL2        , 0x0007,
  ILI932X_POW_CTRL3        , 0x0000,
  ILI932X_POW_CTRL4        , 0x0000,
  TFTLCD_DELAY             , 200,
  ILI932X_POW_CTRL1        , 0x1690,
  ILI932X_POW_CTRL2        , 0x0227,
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL3        , 0x001A,
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL4        , 0x1800,
  ILI932X_POW_CTRL7        , 0x002A,
  TFTLCD_DELAY             , 50,
  ILI932X_GAMMA_CTRL1      , 0x0000,
  ILI932X_GAMMA_CTRL2      , 0x0000,
  ILI932X_GAMMA_CTRL3      , 0x0000,
  ILI932X_GAMMA_CTRL4      , 0x0206,
  ILI932X_GAMMA_CTRL5      , 0x0808,
  ILI932X_GAMMA_CTRL6      , 0x0007,
  ILI932X_GAMMA_CTRL7      , 0x0201,
  ILI932X_GAMMA_CTRL8      , 0x0000,
  ILI932X_GAMMA_CTRL9      , 0x0000,
  ILI932X_GAMMA_CTRL10     , 0x0000,
  ILI932X_GRAM_HOR_AD      , 0x0000,
  ILI932X_GRAM_VER_AD      , 0x0000,
  ILI932X_HOR_START_AD     , 0x0000,
  ILI932X_HOR_END_AD       , 0x00EF,
  ILI932X_VER_START_AD     , 0X0000,
  ILI932X_VER_END_AD       , 0x013F,
  ILI932X_GATE_SCAN_CTRL1  , 0xA700, // Driver Output Control (R60h)
  ILI932X_GATE_SCAN_CTRL2  , 0x0003, // Driver Output Control (R61h)
  ILI932X_GATE_SCAN_CTRL3  , 0x0000, // Driver Output Control (R62h)
  ILI932X_PANEL_IF_CTRL1   , 0X0010, // Panel Interface Control 1 (R90h)
  ILI932X_PANEL_IF_CTRL2   , 0X0000,
  ILI932X_PANEL_IF_CTRL3   , 0X0003,
  ILI932X_PANEL_IF_CTRL4   , 0X1100,
  ILI932X_PANEL_IF_CTRL5   , 0X0000,
  ILI932X_PANEL_IF_CTRL6   , 0X0000,
  ILI932X_DISP_CTRL1       , 0x0133, // Main screen turn on
};

void system_init();
void lcd_init();
void writeRegister16(uint8_t a, uint8_t d);
void write8(uint8_t d);
void writeRegister24(uint8_t r, uint32_t d);
void writeRegister32(uint8_t r, uint32_t d);
void setAddrWindow(int x1, int y1, int x2, int y2);
void reset();

ISR_NMI(){

}

ISR_INT_38(){
       
}


int main(){
	
    // Inicialización del sistema.
    system_init(); 

    // Ciclo infinito del programa.
    while(TRUE){
        sleep();    //Entra en modo sleep (HALT)
    }      

}

void system_init()
{
	PPI_CTRL = 0x89; //Palabra de control PA y PB salida PCH entrada PCD salida
	lcd_init();   
}

void lcd_init()
{
	uint8_t i = 0;
	uint16_t a, d;
	
	//reset();
	delay_ms(200);
	
    CS_ACTIVE;
    while(i < sizeof(PROGMEM) / sizeof(uint16_t)) 
    {
		a = PROGMEM[i++];
		d = PROGMEM[i++];
		
		if(a == TFTLCD_DELAY)
		{
			delay_ms(d);
		}
		else
		{                  
			writeRegister16(a, d);
		}
    }
    CS_ACTIVE;
    a = 0x1030;
    writeRegister16(0x0003, a);
    setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);
}


void writeRegister24(uint8_t r, uint32_t d) 
{
	CS_ACTIVE;
	CD_COMMAND;
	write8(r);
	CD_DATA;
	delay_10us();
	write8(d >> 16);
	delay_10us();
	write8(d >> 8);
	delay_10us();
	write8(d);
	CS_IDLE;
}


void writeRegister32(uint8_t r, uint32_t d) 
{
	CS_ACTIVE;
	CD_COMMAND;
	write8(r);
	CD_DATA;
	delay_10us();
	write8(d >> 24);
	delay_10us();
	write8(d >> 16);
	delay_10us();
	write8(d >> 8);
	delay_10us();
	write8(d);
	CS_IDLE;
}


void writeRegister16(uint8_t a, uint8_t d) 
{
	uint8_t hi, lo;
	
	hi = (a) >> 8; 
	lo = (a); 
	CD_COMMAND; 
	write8(hi); 
	write8(lo); 
	hi = (d) >> 8; 
	lo = (d); 
	CD_DATA; 
	write8(hi); 
	write8(lo); 
}

void write8(uint8_t d) 
{
	PPI_PORTA = d;
	WR_STROBE; 
}

void setAddrWindow(int x1, int y1, int x2, int y2) 
{
	int x, y, t;
	CS_ACTIVE;
	x  = x1;
	y  = y1;
      
    writeRegister16(0x0050, x1); 
    writeRegister16(0x0051, x2);
    writeRegister16(0x0052, y1);
    writeRegister16(0x0053, y2);
    writeRegister16(0x0020, x );
    writeRegister16(0x0021, y );

}

void reset() 
{
	uint8_t i;
	
	CS_IDLE;
	WR_IDLE;
	RD_IDLE;
	
	RESET_LOW;
	delay_ms(2);
	RESET_HIGH;
	
	CS_ACTIVE;
	CD_COMMAND;
	write8(0x00);
	
	for(i=0; i<3; i++)
	{ 
		WR_STROBE;
	}
	CS_IDLE;
	
	delay_ms(500);

}



