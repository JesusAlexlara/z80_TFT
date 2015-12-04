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
#include "TFT_z80.h"
#include "morado.h"
#include "rana.h"
#include "rana2.h"
#include "rana3.h"
#include "rana4.h"

typedef struct
{
	int altura;
	int x;
	int y;
}RANA;

volatile uint16_t cuadroMorado[20][20];
volatile uint16_t rana[20][20];
volatile uint16_t rana2[20][20];
volatile uint16_t rana3[20][20];
volatile uint16_t rana4[20][20];
RANA jug;
	
ISR_NMI()
{

}

ISR_INT_38()
{
       
}


int main()
{  
	jug.x = 20;
	jug.y = 300;
    system_init(); 
    PPI_PORTA = 0xff;
    
    fillScreen(BLACK);
    init_pantalla();
    
    
    while(TRUE)
    {
		SLEEP();
    }      
}

void system_init()
{
	PPI_CTRL = 0x89;
	PPI_PORTA = 0xff;

	lcd_init();   
}

void lcd_init()
{
	uint8_t i = 0;
	uint16_t a, d;
	
	CS_IDLE;
	WR_IDLE;
	RD_IDLE;
	CD_DATA;
	
	reset();
	
    CS_ACTIVE;
    while(i < sizeof(PROGMEM) / sizeof(uint16_t)) 
    {
		a = PROGMEM[i++];
		d = PROGMEM[i++];
		
		if(a == TFTLCD_DELAY)
		{
			delay_ms(d);
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
	write8(d >> 16);
	write8(d >> 8);
	write8(d);
	CS_IDLE;
}


void writeRegister32(uint8_t r, uint32_t d) 
{
	CS_ACTIVE;
	CD_COMMAND;
	write8(r);
	CD_DATA;
	write8(d >> 24);
	write8(d >> 16);
	write8(d >> 8);
	write8(d);
	CS_IDLE;
}


void writeRegister16(uint16_t a, uint16_t d) 
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
	PPI_PORTB = d;
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

void fillScreen(uint16_t color) {

    uint16_t x, y;
    
	x = 0;
	y = 0;
     
    CS_ACTIVE;
    writeRegister16(0x0020, x);
    writeRegister16(0x0021, y);
    
    flood(color, (long)TFTWIDTH * (long)TFTHEIGHT);
}


void flood(uint16_t color, uint32_t len) 
{
	uint16_t blocks;
	uint8_t  i, hi, lo;
	hi= color >> 8;
	lo=color;         

	CS_ACTIVE;
	CD_COMMAND;

    write8(0x00);
    write8(0x22);
    
	CD_DATA;
	write8(hi);
	write8(lo);
	len--;
	
	blocks = (uint16_t)(len / 64);
	if(hi == lo) 
	{
		while(blocks--) 
		{
			i = 16;
			do 
			{
				WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE;
				WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; 
			}while(--i);
		}
		for(i = (uint8_t)len & 63; i--; ) 
		{
			WR_STROBE;
			WR_STROBE;
		}
	} 
	else 
	{
		while(blocks--) 
		{
			i = 16;
			do 
			{
				write8(hi); write8(lo); write8(hi); write8(lo);
				write8(hi); write8(lo); write8(hi); write8(lo);
			}while(--i);
		}
		for(i = (uint8_t)len & 63; i--; ) {
			write8(hi);
			write8(lo);
		}
	}
	CS_IDLE;
}

void drawPixel(int16_t x, int16_t y, uint16_t color) 
{
	if((x < 0) || (y < 0) || (x >= TFTWIDTH) || (y >= TFTHEIGHT))
	{ 
		return;
	}
	CS_ACTIVE;
    writeRegister16(0x0020, x);
    writeRegister16(0x0021, y);
    writeRegister16(0x0022, color);
    CS_IDLE;
}

void fillRect(int16_t x1, int16_t y1, int16_t w, int16_t h,
  uint16_t fillcolor) {
  int16_t  x2, y2;

  if( (w <= 0 ) ||  (h  <= 0 ) || (x1 >= TFTWIDTH) ||  (y1 >= TFTHEIGHT) || ((x2 = x1+w-1) <  0 ) || ((y2  = y1+h-1) <  0 ))
   return;
  if(x1 < 0) 
  {
    w += x1;
    x1 = 0;
   }
  if(y1 < 0) 
  {
    h += y1;
    y1 = 0;
  }
  if(x2 >= TFTWIDTH) {
    x2 = TFTWIDTH - 1;
    w  = x2 - x1 + 1;
  }
  if(y2 >= TFTHEIGHT) {
    y2 = TFTHEIGHT - 1;
    h  = y2 - y1 + 1;
  }
  setAddrWindow(x1, y1, x2, y2);
  flood(fillcolor, (uint32_t)w * (uint32_t)h);
  setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
}

void drawGraf()
{
	uint16_t colorAux;
	uint8_t pixeles[3];
	int i, y;  
	
	for(i = 0; i < 20; i++)
	{
		for(y = 0; y < 20; y++)
		{
			HEADER_PIXEL(header_data,pixeles);
			colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
			cuadroMorado[i][y] = colorAux;
		}
	}
	for(i = 0; i < 20; i++)
	{
		for(y = 0; y < 20; y++)
		{
			HEADER_PIXEL(header_Rana,pixeles);
			colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
			rana[i][y] = colorAux;
		}
	}
	for(i = 0; i < 20; i++)
	{
		for(y = 0; y < 20; y++)
		{
			HEADER_PIXEL(header_Rana2,pixeles);
			colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
			rana2[i][y] = colorAux;
		}
	}
	for(i = 0; i < 20; i++)
	{
		for(y = 0; y < 20; y++)
		{
			HEADER_PIXEL(header_Rana3,pixeles);
			colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
			rana3[i][y] = colorAux;
		}
	}
	for(i = 0; i < 20; i++)
	{
		for(y = 0; y < 20; y++)
		{
			HEADER_PIXEL(header_Rana4,pixeles);
			colorAux = color565(pixeles[0],pixeles[1],pixeles[2]);
			rana4[i][y] = colorAux;
		}
	}
}

void pintaSprite(int _x, int _y, int spr)
{
	int x, y;
	
	if(spr == 1)
	{
		for(y = 0; y < 20; y++)
		{
			for(x = 0; x < 20; x++)
			{
				drawPixel(x+_x, y+_y, cuadroMorado[y][x]);
			}	
		}
	}
	if(spr == 2)
	{
		for(y = 0; y < 20; y++)
		{
			for(x = 0; x < 20; x++)
			{
				if(rana[y][x] != 0x0000)
					drawPixel(x+_x, y+_y, rana[y][x]);
			}	
		}
	}
	if(spr == 3)
	{
		for(y = 0; y < 20; y++)
		{
			for(x = 0; x < 20; x++)
			{
				if(rana2[y][x] != 0x0000)
					drawPixel(x+_x, y+_y, rana2[y][x]);
			}	
		}
	}
	if(spr == 4)
	{
		for(y = 0; y < 20; y++)
		{
			for(x = 0; x < 20; x++)
			{
				if(rana3[y][x] != 0x0000)
					drawPixel(x+_x, y+_y, rana3[y][x]);
			}	
		}
	}
	if(spr == 5)
	{
		for(y = 0; y < 20; y++)
		{
			for(x = 0; x < 20; x++)
			{
				if(rana4[y][x] != 0x0000)
					drawPixel(x+_x, y+_y, rana4[y][x]);
			}	
		}
	}
}

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void init_pantalla()
{
	drawGraf();
	
	pintaAzul();
	pintaNegro();
	
	pintaBarra(0);
	pintaSprite(jug.x, jug.y, 2);
}

void pintaAzul()
{
	fillRect(0, 0, 240, 160, BLUE);
}

void pintaNegro()
{
	fillRect(0, 160, 240, 320, BLACK);
}

void pintaBarra(int i)
{	
	if(i == 0)
		for(i=0; i < 12; i++){
			pintaSprite(i*20, 280, 1);
			pintaSprite(i*20, 160, 1);
		}
}

void repinta_rana(int i)
{
	if(jug.altura <= 7)
	{
		fillRect(jug.x, jug.y, 20, 20, BLUE);
	}
	else{
		fillRect(jug.x, jug.y, 20, 20, BLACK);
	}
	switch(jug.altura)
	{
		case 15:
		case 8:
			pintaSprite(jug.x, jug.y, 1);
		break;
	}
	//aumenta y repinta		
}
	

