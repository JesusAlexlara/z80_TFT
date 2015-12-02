/***************************************************************************
 * 	main.c
 *
 * 	Programa  sencillo  de  ejemplo  para  el z80 escrito en C
 * 	con el compilador SDCC.
 *
 * 	Es necesario  contar con los archivos Makefile, sysmap.asm
 *  y el programa hex2bin para la compilacion y generación del
 *  codigo hexadecimal y el archivo ejecutable.
 *
 * 	Se incluye  la  libreria  z80io.h que proporciona macros y
 *  definiciones de los registros para el PPI y la UART.
 *
 ***************************************************************************
 *
 *		Autor:	Alfredo Orozco de la Paz
 *		e-mail:	alfredoopa@gmail.com
 * 																	*FRED*
 ***************************************************************************
 */
#define BAUD 9600
#define F_UART 4000000
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "z80utils.h"
#include "z80interrupt.h"
#include "z80ppi.h"
#include "z80io.h"
#include "z80uart.h"

void system_init();
void delay_ms(int milis);
void teclado_read();
unsigned char corrimiento=0;

ISR_NMI(){

    /* Código de servicio de la interrupción NMI.*/
}

ISR_INT_38(){
	teclado_read();
	EI();
    /* Código de servicio de la interrupción INT en modo 1 (Vector 0x38).*/
}


int main(){
	char KEYS[4][4] = 
		{{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}};
	corrimiento=0X10;
	
    system_init();

    while(TRUE)
    {
		PPI_PORTC = corrimiento;
    	if(PPI_PORTC==0x0)
    	{
    		corrimiento=0x10;
    	}
    	else
    	{
    		corrimiento<<=1;

    	}
    	//printf("Letra: ");
    }

}


void system_init(){
  //uint8_t es igual a unsigned char
  PPI_CTRL = 0x81; //Salida
  PPI_PORTA = 0;
  PPI_PORTB = 0;
  PPI_PORTC = 0;
  
  UartInit();
  IM(1);
  EI();
  
}

void delay_ms(int milis){
	int i;
	while(milis--){
		for(i = 0; i<0x10A; i++){
			NOP();
		}
	}
}

void teclado_read()
{
	int RES;
	char letra;
	RES  =  PPI_PORTC & 0x0F;
	RES = RES | corrimiento;
	switch(RES)
	{
		case 0x81:
		letra='1';
		printf(" %c", letra);
		break;

		case 0x41:		
		letra='2';
		printf(" %c", letra);
		break;

		case 0x21:
		letra='3';
		printf(" %c", letra);
		break;

		case 0x82:
		letra='4';
		printf(" %c", letra);
		break;

	    case 0x42:
	    letra='5';
		printf(" %c", letra);
		break;

	    case 0x22:
	    letra='6';
		printf(" %c", letra);
		break;

	    case 0x84:
	    letra='7';
		printf(" %c", letra);
		break;

        case 0x44:
        letra='8';
		printf(" %c", letra);
		break;

		case 0x24:
		letra='9';
		printf(" %c", letra);
		break;

	    case 0x11:
	    letra='A';
		printf(" %c", letra);
		break;

	    case 0x12:
	    letra='B';
		printf(" %c", letra);
		break;

		case 0x14:
		letra='C';
		printf(" %c", letra);
		break;

		case 0x18:
		letra='D';
		printf(" %c", letra);
		break;

		case 0x88:
		letra='E';
		printf(" %c", letra);
		break;

	    case 0x28:
	    letra='F';
		printf(" %c", letra);
		break;

	    case 0x48:
	    letra='0';
		printf(" %c", letra);
		break;
	    
	}
}
























