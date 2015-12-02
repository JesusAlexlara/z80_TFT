/***************************************************************************
 * 	z80uart.h
 *
 * 	Librería para el manejo de la UART en un sistema mínimo con
 * 	procesadpr z80 en C para el compilador SDCC
 *
 *  Los registros de la UART estan definidos en el archivo z80io.h
 *  de acuerdo al direccionamiento que se le haya asignado. Este
 *  direccionamiento puede modificarse en el archivo z80io.h.
 *
 *  Para usar la librería se debe definir la frecuencia a la que
 *  trabaja la UART como F_UART (en MHz) y el baudaje que usará
 *  con BAUD. Si se omiten estas definiciones el compilador 
 *  marcaŕa un error, ya que estos parametros son necesarios para
 *  calcular el divisor de baudaje interno de la UART.
 *  
 *  Funciones de la librería:
 *
 *  void UartInit()
 *      Inicializa la UART con 8 bits de dato, 1 bit de stop, sin
 *      paridad y con el baudaje especificado por BAUD.
 *  
 *
 *  void UartWrite(char c)
 *      Envia un caracter por la UART. La función espera a que el
 *      registro transmisor este vacío para enviar el nuevo dato.
 *
 *  char UartRead()
 *      Lee un dato recibído por la UART. La fución espera a que
 *      un nuevo dato este disponible en la UART para ser leido
 *      y devuelto.
 *
 *  void UartPrint(const char* string)
 *      Envía una cadena por la UART.
 *
 ***************************************************************************
 *
 *		Autor:	Alfredo Orozco de la Paz
 *		e-mail:	alfredoopa@gmail.com 
 * 																	*FRED*
 ***************************************************************************/

#ifndef _Z80UART_H_
#define _Z80UART_H_

#include <ctype.h>

 
#ifndef F_UART
#error "No se definio la frecuencia de la UART: F_UART"
#endif

#ifndef BAUD
#error "No se definio el baudaje para la UART: BAUD"
#endif

#define UART_DIV_VALUE() ( (F_UART) / ((uint32_t)(BAUD)*16) )

/* Número máximo de caracteres a leer por UartReadLine*/
#define MAXREAD 100
/*
 * Por default 8 bits de dato, 1 bit de parada, sin paridad.
 */
void UartInit(){

    // DLAB = 1
    ULCR = UDLAB;
    // Divisor de baudaje bajo
    UDLL = UART_DIV_VALUE();
    // Divisor de baudaje alto
    UDLM = UART_DIV_VALUE() >> 8;
    // DLAB = 0, 8 bits de dato, sin paridad.
    ULCR = UWLS0 | UWLS1;
    
    // Interrupción por dato recibido habilitada.
    UIER = UERBFI;

}

void UartWrite(char c){
	int i;
	
    // Mientras el registro transmisor no esté vacío...
    while( !(io_read(UART_LSR_ADDR) & UTEMT) );
    NOP();    
    // Cargamos el dato a enviar en el registro transmisor.
    io_write(UART_THR_ADDR,c);
	
	
        for(i=0;i<0xF0;i++)
            __asm__("nop");
}

char UartRead(){

    // Mientras no se haya recibido un dato...
    while( !(io_read(UART_LSR_ADDR) & UDR) );    
    NOP();
    // Devuelve el dato recibido.
    return io_read(UART_RBR_ADDR);
}

// Envía una cadena de caracteres por la UART
void UartPrint(const char* str){

    while(*str)       
       putchar(*str++);
    
}

void UartReadLine(char* str){

    int n=0;
    char c;
    while(n<MAXREAD-1 && (c=getchar()) != '\n' && c !='\r'){
        // Si es DEL (0x7f) o Backspace (oxo8)
        if(c == 0x7f || c == 0x08){
		
			if(n>0){
				str[--n]='\0';
				putchar(c);
				putchar(' ');
				putchar(c);
			}
        }
        else
        if(isprint(c))
        {
            str[n++]=c;
            putchar(c);
        }
    }       
       
    putchar('\n');
}
void putchar(char c ){

    if(c=='\n')
        UartWrite('\r');
    
    UartWrite(c);
}

char getchar(){
    return UartRead();
}

#endif  // Fin _Z80UART_H_
