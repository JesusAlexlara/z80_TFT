/***************************************************************************
 *  Ejemplo UART 8250.
 *  
 *  Programa de ejemplo para configuración y uso de la UART 8250 con la
 *  API smz80.
 *  
 *  El programa configura la UART como 8N1 a 9600 baudios con la interrupcion
 *  por dato recibido habilitada por la interrupción INT del Z80 en modo 1.
 *  
 *  Cada que se recibe un caracter, se envia de regreso (ECO).
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
    
    char c;

    c=uart_read();  // Lee el dato recibido
    uart_write(c);  // Envía el dato recibido
    EI();   // vuelve a activar la interrupcion INT
    
}


void system_init(){

    // Estructura de configuración para la UART.
    uart_cfg_t uart_config;

    // Configura los aprametros de la UART.
    uart_config.baudrate    = UART_BAUDRATE_9600;   // Baudaje a 1200.
    uart_config.stop_bits   = UART_STOP_BITS_1;     // 1 bit de parada.
    uart_config.parity      = UART_PARITY_NONE;     // Sin paridad
    uart_config.word_length = UART_WORD_LENGTH_8;   // Dato de 8 bits
    uart_config.interrupt   = UART_INTERRUPT_RX;    // Interrupción por dato recibido activada.

    // Inicializa la UART con su configuración.
    uart_init(&uart_config);

    IM(1);  // Modo de interrupcion 1 en el Z80.
    EI();   // Habilita la interrupción INT en el Z80
    
}

int main(){

    // Inicialización del sistema.
    system_init(); 

    // Ciclo infinito del programa.
    while(TRUE){

        halt(); // Entra en modo HALT.
        
    }      

}