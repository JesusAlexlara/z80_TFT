/***************************************************************************
 *  Ejemplo PPI 8255.
 *  
 *  Programa de ejemplo para configuración y uso del PPI 8255 con la
 *  API smz80.
 *  
 *  El programa configura el PPI 8255 en modo 0 cons el puerto A, B y C 
 *  de salida, haciendo parpadear leds conectados a todos sus puertos
 *  cada 500 milisegundos (BLINK).  
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
#define PPI_BASE_ADDR  0x00
#define UART_BASE_ADDR 0x10

#include "smz80.h"

ISR_NMI(){

    /* Código de servicio de la interrupción NMI.*/
}

ISR_INT_38(){
    
    /* Código de servicio de la interrupción INT en modo 1 (Vector 0x38).*/
    
}


void system_init(){

    // Programa la palabra de control para el PPI.
    PPI_CTRL = 0x80;
    
}

int main(){

    // Inicialización del sistema.
    system_init(); 

    // Ciclo infinito del programa.
    while(TRUE){

        // Apaga los LEDs de los puertos.
        PPI_PORTA = 0;
        PPI_PORTB = 0;
        PPI_PORTC = 0;

        // Espera 500 milisegundos
        delay_ms(500);

        // Enciende los LEDs de los puertos.
        PPI_PORTA = 255;
        PPI_PORTB = 0xFF;
        PPI_PORTC = 0b11111111;

        // Espera 500 milisegundos
        delay_ms(500);
    }      

}





