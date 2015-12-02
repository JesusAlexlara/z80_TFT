/***************************************************************************
 *  smz80.h
 *  
 *         Versión:  1.0
 *
 *           Autor:  Alfredo Orozco de la Paz
 *         Archivo:  smz80.h
 *           Fecha:  Nov 23, 2015
 *
 *      Procesador:  Z80 CPU
 *      Compilador:  SDCC
 *
 *    ############################# Última Modficación #############################
 *      
 *    ##############################################################################
 ***************************************************************************************
 *  Descripción:
 *  
 *      Api con funciones, definiciones, estructuras de datos y macros
 *      para la programación de aplicaciones en C para el microprocesador
 *      Z80 con el compilador SDCC.
 *      
 *      Esta API cuenta con funciones relacionadas con el PPI 8255, la 
 *      UART 8250 de Intel y el mismo microprocesador Z80.
 *      
 *      Funciones que implementa la API:
 *      
 *       Para la UART:
 *       
 *              Funciones:
 *                  void uart_init(uart_cfg_t *uart_config)                         // Configura la UART
 *                  void uart_write(char c)                                         // Escribe un dato por la UART.
 *                  char uart_read();                                               // Lee un dato de la UART
 *                  char uart_print(const char *s);                                 // Imprime una cadena de caracteres por la UART 
 *                  int uart_read_line(const char *line);                           // Lee una linea de texto de la UART.
 *                  void uart_set_baudrate(uart_baudrate_t baudrate);               // Cambia el baudrate del la UART.
 *                  void uart_enable_interrupts(uart_interrupt_t interrupts);       // Habilita interrupciones de la UART.
 *                  void uart_disable_interrupts();                                 // Desabilita todas las interrupciones de la UART.
 *                 
 *                  UART_DIV_VALUE(BAUDRATE)                // Calcula el divisor de baudaje para la UART
 *                  
 *              Definiciones de variables a puertos de E/S:
 *               
 *                  URRBR       // Variable del Registro RX (DLAB=0).
 *                  URTHR       // Variable del Registro TX (DLAB=0).
 *                  URIER       // Variable del Registro Interrupt Enable.
 *                  URIIR       // Variable del Registro Interrupt Identification.
 *                  URLCR       // Variable del Registro Line Control.
 *                  URMCR       // Variable del Registro Modem Control.
 *                  URLSR       // Variable del Registro Line Status
 *                  URMSR       // Variable del Registro Modem Status
 *                  URDLL       // Variable del Registro Divisor Latch LSB (DLAB=1)
 *                  URDLM       // Variable del Registro Divisor Latch MSB (DLAB=1)
 *                  
 *                  
 *       Para el PPI:
 *       
 *              Funciones:
 *                  void ppi_init(ppi_cfg_t *ppi_config);       // Inicializa el PPI
 *                  void ppi_set_portc_bit(char bit_num);       // Pone a 1 un bit del puerto C del PPI
 *                  void ppi_clear_portc_bit(char bit_num);     // Pone a 0 un bit del puerto C del PPI
 *              
 *              Definiciones de variables a puertos de E/S:
 *               
 *                  PPI_PORTA       // Variable al puerto A del PPI.      
 *                  PPI_PORTB       // Variable al puerto B del PPI.      
 *                  PPI_PORTC       // Variable al puerto C del PPI.      
 *                  PPI_CTRL        // Variable al puerto de CONTROL del PPI.               
 *                  
 *                  
 *       Para el Z80:
 *       
 *              Funciones:
 *                  void io_write(char port_addr, char data);                               // Envia un dato a un puerto de E/S del Z80.
 *                  void io_write_buffer(char port_addr, char* buffer_out, char count);     // Envia un buffer de datos a un puerto de E/S del Z80.
 *                  char io_read(char port_adrr);                                           // Lee un dato de un puerto de E/S del Z80.
 *                  void io_read_buffer(char port_addr, char* buffer_in, char count);       // Lee un buffer de datos de un puerto de E/S del Z80.
 *                  
 *                  void delay_10us()       // Hace un delay de 10 microsegundos a 4 Mhz
 *                  void delay_100us()      // Hace un delay de 100 microsegundos a 4 Mhz
 *                  void delay_ms(int ms)   // Hace un delay de milisegundos microsegundos a 4 Mhz
 *                  
 *                  HALT()                      Ejecuta la instrucción HALT
 *                  halt()                      Ejecuta la instrucción HALT
 *                  SLEEP()                     Ejecuta la instrucción HALT
 *                  sleep()                     Ejecuta la instrucción HALT
 *                  NOP()                       Ejecuta la instrucción NOP
 *                  nop()                       Ejecuta la instrucción NOP
 *                  EI()                        Habilita interrupciones: instruccion EI.
 *                  DI()                        Deshabilita interrupciones: instruccion DI
 *                  IM(MODE)                    Cambia el modo de interrupcion INT: instruccion IM
 *                  RST(VECT)                   Ejecuta la instrucción RST al vector VECT.
 *                  SFR_IO port_addr var_name   Macro para crear una variabale a un puerto de E/S (debe ser variable global)
 *                  BV(BIT)                     Macro para obtener el valor hexadecimal de un BIT en un registro.  
 *                  
 *                  
 *             
 *  
 ***************************************************************************************
 *
 *    Copyright:
 *
 *        Copyrigth© 2015
 *        Alfredo Orozco de la Paz   *FRED*
 *        e-mail:  alfredoopa@gmail.com
 *        www.vidaembebida.wordpress.com                                        
 *
 *     Licencia:
 *
 *        Este programa es software libre; usted lo puede distribuir y/o modificar
 *        bajo los terminos   de la General Public License GNU, según es publicada
 *        en la  Free  Software  Foundation;  ya se la versión 2 de la Licencia, o
 *        (a su elección) una versión posterior.
 *
 *        Este  programa se  distribuye  con la  esperanza  de que sea  útil, pero
 *        SIN   NINGUNA   GARANTÍA,  incluso   sin   la   garantía   implícita  de
 *        COMERCIALIZACIÓN  o  IDONEIDAD PARA UN PROPÓSITO PARTICULAR. consulte la
 *        Licencia Pública General de GNU para más detalles.
 *
 *        Debería haber recibido una copia de  la  Licencia Pública General de GNU
 *        junto con este programa; si no, visite http://www.gnu.org/licenses.
 *      
 ***************************************************************************************/

#ifndef SMZ80_H_
#define SMZ80_H_

#include <stdint.h>
#include <stdio.h>
#include <ctype.h> 

/************************************************************************
*                       DEFINICION DE CONSTANTES
************************************************************************/
// Direccionamiento del PPI
#ifndef PPI_BASE_ADDR
#warning "No se definio la direccion base del PPI"
#warning "Para usar las funciones con el PPI defina:      #define PPI_BASE_ADDR 0xNN, antes de incluir smz80.h"
#endif 

#ifndef UART_BASE_ADDR
#warning "No se definio la direccion base de la UART"
#warning "Para usar las funciones con la UART defina:     #define UART_BASE_ADDR 0xNN, antes de incluir smz80.h"
#endif

#define TRUE 1      ///< Definición para verdadero.
#define FALSE 0     ///< Definición para falso.

#define INPUT TRUE      ///< Definición para Entrada.
#define OUTPUT FALSE    ///< Definición para Salida.

#define LOW FALSE       ///< Definición para Bajo.
#define HIGH TRUE       ///< Definición para Alto.

#define MAXLINE 100     /**< Numero maximo de caracteres a leer por ReadLine() */
/***********************************************************************/


/************************************************************************
*          DEFINICION DE FUNCIONES A VECTORES DE INTERRUPCION
************************************************************************/
#define ISR_NMI()     void isr_vector66()  __critical __interrupt   /**< Función para el vector de interrupción NO ENMASCARABLE 0x66 */
#define ISR_INT_08()  void isr_vector08()  __interrupt 1    /**< Función para el vector de interrupción 0x08 */
#define ISR_INT_10()  void isr_vector10()  __interrupt 2    /**< Función para el vector de interrupción 0x10 */
#define ISR_INT_18()  void isr_vector18()  __interrupt 3    /**< Función para el vector de interrupción 0x18 */
#define ISR_INT_20()  void isr_vector20()  __interrupt 4    /**< Función para el vector de interrupción 0x20 */
#define ISR_INT_28()  void isr_vector28()  __interrupt 5    /**< Función para el vector de interrupción 0x28 */
#define ISR_INT_30()  void isr_vector30()  __interrupt 6    /**< Función para el vector de interrupción 0x30 */
#define ISR_INT_38()  void isr_vector38()  __interrupt 7    /**< Función para el vector de interrupción 0x38 */
/***********************************************************************/


/************************************************************************
*                       DEFINICION DE MACROS
************************************************************************/
#define HALT()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define halt()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define NOP()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define nop()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define SLEEP() __asm__ ("HALT")    ///< Macro para poner al Z80 en modo Sleep.
#define sleep() __asm__ ("HALT")    ///< Macro para poner al Z80 en modo Sleep.

#define BV(BIT) (1<<(BIT))          ///< Macro que obtiene la mascara de bits para el bit BIT.
#define SFR_IO __sfr __at           ///< Macro para declarar variables a puertos de E/S.

#define RST(VECTR) __asm\
                     RST VECTR\
                   __endasm        ///< Macro para ejecutar la instrucción RST a un vector de interrupción del Z80.

#define EI()    __asm__ ("EI")  /**< Habilita interrupción INT */
#define DI()    __asm__ ("DI")  /**< Deshabilita interrupción INT */
#define IM(MODE) __asm\
                    IM MODE\
                __endasm        /**< Cambia el modo de la interrupción INT */


#define UART_DIV_VALUE(F_UART,BAUDRATE) ((uint32_t)(F_UART) / ((uint32_t)(BAUDRATE)*16)) /**< Macro para calcular el valor de los registros DLL y DLM de la UART*/
/***********************************************************************/


/************************************************************************
*                       DEFINICION DE TIPOS DE DATOS
************************************************************************/
typedef uint8_t     Byte;   ///< Definición de tipo de dato byte.
typedef uint8_t     byte;   ///< Definición de tipo de dato byte.
typedef uint16_t    Word;  ///< Definición de tipo de dato word.
typedef uint16_t    word;  ///< Definición de tipo de dato word.
typedef uint32_t    DWord; ///< Definición de tipo de dato double word.
typedef uint32_t    dword; ///< Definición de tipo de dato double word.
/***********************************************************************/

#if defined (PPI_BASE_ADDR)
/************************************************************************
*                     DEFINICION DE REGISTROS DEL PPI
************************************************************************/
#define PPI_PORTA_ADDR      PPI_BASE_ADDR + 0x00      /**< Dirección del Puerto A del PPI */
#define PPI_PORTB_ADDR      PPI_BASE_ADDR + 0x01      /**< Dirección del Puerto B del PPI */
#define PPI_PORTC_ADDR      PPI_BASE_ADDR + 0x02      /**< Dirección del Puerto C del PPI */
#define PPI_CTRL_ADDR       PPI_BASE_ADDR + 0x03      /**< Dirección del Registro de Control del PPI */
/***********************************************************************/
#endif

#if defined (UART_BASE_ADDR)
/************************************************************************
*                   DEFINICION DE REGISTROS DE LA UART
************************************************************************/
#define UART_RBR_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro RX (DLAB=0) */
#define UART_THR_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro TX (DLAB=0) */
#define UART_IER_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro Interrupt Enable (DLAB=0) */
#define UART_IIR_ADDR   UART_BASE_ADDR + 0x02    /**< Dirección del Registro Interrupt Identification */
#define UART_LCR_ADDR   UART_BASE_ADDR + 0x03    /**< Dirección del Registro Line Control */
#define UART_MCR_ADDR   UART_BASE_ADDR + 0x04    /**< Dirección del Registro Modem Control */
#define UART_LSR_ADDR   UART_BASE_ADDR + 0x05    /**< Dirección del Registro Line Status */
#define UART_MSR_ADDR   UART_BASE_ADDR + 0x06    /**< Dirección del Registro Modem Status */
#define UART_DLL_ADDR   UART_BASE_ADDR + 0x00    /**< Dirección del Registro Divisor Latch LSB (DLAB=1) */
#define UART_DLM_ADDR   UART_BASE_ADDR + 0x01    /**< Dirección del Registro Divisor Latch MSB (DLAB=1) */
/***********************************************************************/

/************************************************************************
*                 VARIABLES A LOS REGISTROS DE LA UART
************************************************************************/
SFR_IO  UART_RBR_ADDR  URRBR;   /**< Registro RBR - Receiver Buffer Register */
SFR_IO  UART_THR_ADDR  URTHR;   /**< Registro THR - Transmit Hold Register */

SFR_IO  UART_IER_ADDR  URIER;   /**< Registro IER - Interrupt Enable Register */
#define UERBFI   0              /**< Número de bit para el campo Enabled Received Data Availabre Interrupt */
#define UETBEI   1              /**< Número de bit para el campo Transmitter Holding Register Empty Interrupt */
#define UELSI    2              /**< Número de bit para el campo Error Receiver Line Status Interrupt */
#define UEDSSI   3              /**< Número de bit para el campo Enable MODEM Status Interrupt */

SFR_IO  UART_IIR_ADDR  URIIR;   /**< Registro IIR - Interrupt Identification Register */
#define UIP      0              /**< Número de bit para el campo Interrupt Pending */
#define UIID0    1              /**< Número de bit para el campo Interrupt ID bit 0 */
#define UIID1    2              /**< Número de bit para el campo Interrupt ID bit 1 */

SFR_IO  UART_LCR_ADDR  URLCR;   /**< Registro LCR - Line Control Register */
#define UWLS0    0              /**< Número de bit para el campo Word Length Select Bit 0 */
#define UWLS1    1              /**< Número de bit para el campo Word Length Select Bit 1 */
#define USTB     2              /**< Número de bit para el campo Number of Stop Bits */
#define UPEN     3              /**< Número de bit para el campo Parity Enable */
#define UEPS     4              /**< Número de bit para el campo Even Parity Select */
#define USTP     5              /**< Número de bit para el campo Stick Parity */
#define USBK     6              /**< Número de bit para el campo Set Break */
#define UDLAB    7              /**< Número de bit para el campo Divisor Latch Acces Bit */

SFR_IO  UART_LSR_ADDR  URLSR;   /**< Registro LSR - Line Status Register */
#define UDR      0              /**< Número de bit para el campo Data Ready */
#define UOR      1              /**< Número de bit para el campo Overrun Error */
#define UPE      2              /**< Número de bit para el campo Parity Error */
#define UFE      3              /**< Número de bit para el campo Framming Error */
#define UBI      4              /**< Número de bit para el campo Break Interrupt */
#define UTHRE    5              /**< Número de bit para el campo Transmitter Holding Register Empty */
#define UTSRE    6              /**< Número de bit para el campo Transmitter Register Empty */

SFR_IO  UART_MCR_ADDR  URMCR;   /**< Registro MCR - Modem Control Register */
#define UDTR     0              /**< Número de bit para el campo Data Terminal Ready */
#define URTS     1              /**< Número de bit para el campo Request to Send */
#define UOUT1    2              /**< Número de bit para el campo Out 1 */
#define UOUT2    3              /**< Número de bit para el campo Out 2 */
#define ULOOP    4              /**< Número de bit para el campo Loop */

SFR_IO  UART_MSR_ADDR  URMSR;   /**< Registro MSR - Modem Status Register */
SFR_IO  UART_DLL_ADDR  URDLL;   /**< Registro DLL - Divisor Latch LSB */
SFR_IO  UART_DLM_ADDR  URDLM;   /**< Registro DLM - Divisor Latch MSB */
/***********************************************************************/
#endif


#if defined (PPI_BASE_ADDR)
/************************************************************************
*                   VARIABLES A LOS REGISTROS DEL PPI
************************************************************************/
// Registro del Puerto A
SFR_IO  PPI_PORTA_ADDR    PPI_PORTA;    /**< Variable del Puerto A*/
// Bits del PUERTO A
#define PA0     0       /**< Numero de bit para el puerto PA0*/
#define PA1     1       /**< Numero de bit para el puerto PA1*/
#define PA2     2       /**< Numero de bit para el puerto PA2*/
#define PA3     3       /**< Numero de bit para el puerto PA3*/
#define PA4     4       /**< Numero de bit para el puerto PA4*/
#define PA5     5       /**< Numero de bit para el puerto PA5*/
#define PA6     6       /**< Numero de bit para el puerto PA6*/
#define PA7     7       /**< Numero de bit para el puerto PA7*/

// Registro del Puerto B
SFR_IO  PPI_PORTB_ADDR    PPI_PORTB;    /**< Variable del Puerto B*/
// Bits del PUERTO B
#define PB0     0       /**< Numero de bit para el puerto PB0*/
#define PB1     1       /**< Numero de bit para el puerto PB1*/
#define PB2     2       /**< Numero de bit para el puerto PB2*/
#define PB3     3       /**< Numero de bit para el puerto PB3*/
#define PB4     4       /**< Numero de bit para el puerto PB4*/
#define PB5     5       /**< Numero de bit para el puerto PB5*/
#define PB6     6       /**< Numero de bit para el puerto PB6*/
#define PB7     7       /**< Numero de bit para el puerto PB7*/

// Registro del Puerto C
SFR_IO  PPI_PORTC_ADDR    PPI_PORTC;    /**< Variable del Puerto C*/
// Bits del PUERTO C
#define PC0     0       /**< Numero de bit para el puerto PC0*/
#define PC1     1       /**< Numero de bit para el puerto PC1*/
#define PC2     2       /**< Numero de bit para el puerto PC2*/
#define PC3     3       /**< Numero de bit para el puerto PC3*/
#define PC4     4       /**< Numero de bit para el puerto PC4*/
#define PC5     5       /**< Numero de bit para el puerto PC5*/
#define PC6     6       /**< Numero de bit para el puerto PC6*/
#define PC7     7       /**< Numero de bit para el puerto PC7*/

// Registro de control
SFR_IO  PPI_CTRL_ADDR  PPI_CTRL;   /**< Variable del Registro de Control */

#define PCPCL   0       /**< Numero de bit para el Control Word PCL bit */
#define PCPCH   3       /**< Numero de bit para el Control Word PCA bit */
#define PCPA    4       /**< Numero de bit para el PPI Control Word PA bit */
#define PCPB    1       /**< Numero de bit para el PPI Control Word PB bit */
#define PCMB    2       /**< Numero de bit para el PPI Control Word Mode Group B bit */
#define PCMA0   5       /**< Numero de bit para el PPI Control Word Mode Group A bit 0 */
#define PCMA1   6       /**< Numero de bit para el PPI Control Word Mode Group A bit 1 */
#define PCME    7       /**< Numero de bit para el PPI Control Word Mode Enable */
/***********************************************************************/
#endif


#if defined (UART_BASE_ADDR)
/************************************************************************
*                          ENUMERACIONES
************************************************************************/
/**
 *  @brief Enumeración de valores para el divisor de baudaje alto y bajo para una frecuencia de 4 Mhz. 
 *         Nota: Cambiar estos valores para una frecuencia de reloj diferente a 4 Mhz
 */
typedef enum{
    UART_BAUDRATE_1200    =  0x00D0, /**<Valor para un baudaje de 1200*/
    UART_BAUDRATE_2400    =  0x0068, /**<Valor para un baudaje de 2400*/
    UART_BAUDRATE_4800    =  0x0034, /**<Valor para un baudaje de 4800*/
    UART_BAUDRATE_9600    =  0x001A, /**<Valor para un baudaje de 9600*/
    UART_BAUDRATE_19200   =  0x000D, /**<Valor para un baudaje de 19200*/
    UART_BAUDRATE_38400_1 =  0x0006, /**<Valor para un baudaje de 38400*/
    UART_BAUDRATE_38400_2 =  0x0007, /**<Valor para un baudaje de 38400*/
    UART_BAUDRATE_38400_3 =  0x0005  /**<Valor para un baudaje de 38400*/
}uart_baudrate_t;

/**
 * @brief Enumeración de valores para en numero de bits de parada.
 * @details Definiciones para 1 o 2 bits de parada en la comunicación
 *          de la UART.
 */
typedef enum{
    UART_STOP_BITS_1    = 0,         /**< 1 bit de parada*/
    UART_STOP_BITS_2    = BV(USTB)   /**< 2 bits de parada*/
}uart_stopbits_t;

/**
 * @brief   Enumeración de valores para la paridad.
 * @details Define los tipos de paridad a usar en la comunicación de
 *          la UART.
 */
typedef enum{
    UART_PARITY_NONE    = 0,                                 /**< Sin paridad*/
    UART_PARITY_ODD     = BV(UPEN),                          /**< Paridad par*/
    UART_PARITY_EVEN    = BV(UPEN) | BV(UEPS),               /**< Paridad impar*/
    UART_PARITY_MARK    = BV(UPEN) | BV(USTP),               /**< Marca*/
    UART_PARITY_SPACE   = BV(UPEN) | BV(UEPS) | BV(USTP)     /**< Espacio*/
}uart_parity_t;

/**
 * @brief Enumeración de valores para el tamaño de dato de la UART.
 * @details Define los tamaños de dato a ser usados por la UART.
 */
typedef enum{
    UART_WORD_LENGTH_5   = 0,                         /**< Dato de 5 bits*/
    UART_WORD_LENGTH_6   = BV(UWLS0),                 /**< Dato de 6 bits*/
    UART_WORD_LENGTH_7   = BV(UWLS1),                 /**< Dato de 7 bits*/
    UART_WORD_LENGTH_8   = BV(UWLS0) | BV(UWLS1),     /**< Dato de 8 bits*/
}uart_word_length_t;

/**
 * @brief Enumeración de valores para los tipos de interrupción.
 * @details Define los valores para habilitar las interrupciones de la UART.
 */
typedef enum{
    UART_INTERRUPT_NONE  = 0,                            /**< No habilita interrupciones de la UART.*/
    UART_INTERRUPT_TX    = BV(UETBEI),                   /**< Habilita la interrupción por transmisor vacío.*/
    UART_INTERRUPT_RX    = BV(UERBFI),                   /**< Habilita la interrupción por dato disponible.*/
    UART_INTERRUPT_ERROR = BV(UELSI),                    /**< Habilita la interrupción por error de recepción.*/
    UART_INTERRUPT_MSR   = BV(UEDSSI),                   /**< Habilita la interrupción por cambio en el registro MSR.*/
    UART_INTERRUPT_RXTX  = BV(UETBEI)|BV(UERBFI),        /**< Habilita la interrupciones de dato disponible y transmisor vacío.*/
    UART_INTERRUPT_RX_ERROR  = BV(UERBFI)|BV(UELSI),     /**< Habilita la interrupciones de dato disponible y error de recepción.*/
    UART_INTERRUPT_TX_ERROR  = BV(UETBEI)|BV(UELSI),     /**< Habilita la interrupciones de transmisor vacío y error de recepción.*/
    UART_INTERRUPT_RXTX_ERROR  = BV(UETBEI)|BV(UERBFI)|BV(UELSI),   /**< Habilita la interrupciones de dato disponible, transmisor vacío y transmisor vacío.*/
    UART_INTERRUPT_ALL  = BV(UELSI)                    /**< Habilita todas las interrupciones de la UART.*/
}uart_interrupt_t;
#endif


/**
 * @brief Enumeración de modos para el PPI 8250
 * @details Define los modos de operación del PPI 8250.
 */
typedef enum{
    PPI_MODE_0  = 0x00,     /**< Modo 0 del PPI*/
    PPI_MODE_1  = 0x24,     /**< Modo 1 del PPI*/
    PPI_MODE_2  = 0x40      /**< Modo 2 del PPI*/
}ppi_mode_t;
/***********************************************************************/

#if defined (UART_BASE_ADDR)
/************************************************************************
*                      ESTRUCTURAS DE DATOS 
************************************************************************/
/**
 * @brief Estructura para configuración de la UART 8250.
 * @details Contiene los parámetros de configuración de la
 *          UART 8250.
 */
typedef struct{
    
    uart_baudrate_t     baudrate;           /**< Baudaje de la UART*/
    uart_stopbits_t     stop_bits;          /**< Bits de parada*/
    uart_parity_t       parity;             /**< Paridad*/
    uart_word_length_t  word_length;        /**< Tamaño de palabra (5, 6, 7 u 8 bits)*/
    uart_interrupt_t    interrupt;         /**< Tipo de interrupciones usadas */
    
}uart_cfg_t;


#define UART_DEFAULT_CONFIG {   \
    UART_BAUDRATE_9600,         \
    UART_STOP_BITS_1,           \
    UART_PARITY_NONE,           \
    UART_WORD_LENGTH_8,          \
    UART_INTERRUPT_NONE }

#endif

/**
 * @brief Estructura para configuración del PPI 8255.
 * @details Contiene los parámetros de configuración del
 *          PPI 8255.
 */
typedef struct{

    ppi_mode_t          mode;           /**< Modo del PPI*/
    char                pa_dir;         /**< Direccion del puerto A*/
    char                pb_dir;         /**< Direccion del puerto B*/
    char                pcl_dir;        /**< Direccion del puerto C bajo*/
    char                pch_dir;        /**< Direccion del puerto C alto*/

}ppi_cfg_t;
/***********************************************************************/



/**
 * @brief    Escritura en un puerto. 
 * @details  Envía un dato a un puerto específico.
 * 
 * @param addr Dirección del puerto.
 * @param data Dato a enviar.
 */ 
void io_write(char port_addr, char data);

/**
 * @brief   Lectura de un puerto.
 * @details Lee un dato de un puerto específico.
 * 
 * @param addr Dirección del puerto a leer.
 * @return  Dalo leído por el puerto.
 */
char io_read(char port_addr);

/**
 * @brief Envio de un buffer de datos por un puerto.
 * @details Envía un buffer de datos de 8 bits por un puerto
 *          específico.
 * 
 * @param port_addr              Dirección del puerto a escribir.
 * @param[in] buffer_out    Buffer de datos.
 * @param count             Número de datos a enviar.
 */
void io_write_buffer(char port_addr, char* buffer_out, char count);

/**
 * @brief   Lee un buffer de datos por un puerto.
 * @details Lee N datos de un puerto específico y los guarda en buffer_in.
 * 
 * @param port_addr         Dirección del puerto a leer.
 * @param[out] buffer_in    Buffer donde se guardarán los datos
 * @param count             Número de datos a leer.
 */
void io_read_buffer(char port_addr, char* buffer_in, char count);
#if defined (UART_BASE_ADDR)
/**
 * @brief   Inicializa la UART 8250
 * @details Inicializa la UART con la configuración definida en uart_config.
 *     
 * @param uart_config Estructura de configuración de la UART.
 */
void uart_init(const uart_cfg_t *uart_config);

/**
 * @brief Programa el Baudrate de la UART
 * @details Realiza la programación del baudrate programando el bit DLAB del
 *          registro LCR y los registros DLL y DLM
 * 
 * @param baudarte Velocidad de transmisión de la UART.
 */
void uart_set_baudrate(const uart_baudrate_t baudarte);

/**
 * @brief   Envia un dato por la UART.
 * @details Envia un dato mediante la UART.
 *          La funcion espera a que el registro transmisor
 *          este vacio para enviar el dato.
 * 
 * @param c     Dato a enviar.
 */
void uart_write(char c);

/**
 * @brief   Lectura de un dato en la UART.
 * @details 
 *     Lee un dato del registro receptor de la UART.
 *     Espera a que haya disponible un dato, lo lee
 *     y lo devuelve.
 *     
 * @return      Dato Recibido.
 */
char uart_read();

/**
 * @brief   Envio de una cadena de caracteres.
 * @details Envia una cadena de caracteres por la UART.
 *          La cadena debe tener terminación '\0'.
 * 
 * @param str   Cadena de caracteres a enviar.
 */
void uart_print(const char* str);
/**
 * @brief   Lectura de una linea de texto.
 * @details Lee una linea de texto con terminacion '\\n' o '\\r'
 *          de un tamaño maximo definido por MAXLINE.
 * 
 * @param str   Apuntador donde se guardará el texto.
 */
int uart_read_line(char* str);

/**
 * @brief Desabilitación de interrupciones.
 * @details Deshabilita todas las interrupciones de la UART.
 */
void uart_disable_interrupts();

/**
 * @brief Habilitación de interrupciones.
 * @details Habilita las interrupciones de la UART indicadas por int_cfg
 * 
 * @param int_cfg Interrupciones a habilitar.
 */
void uart_enable_interrupts(uart_interrupt_t int_cfg);
#endif
/**
 * @brief   Inicializa el PPI 8255
 * @details Inicializa el PPI 8255 con la configuración definida en ppi_config.
 *     
 * @param PPI_config Estructura de configuración del PPI.
 */
#if defined (PPI_BASE_ADDR)
void ppi_init(const ppi_cfg_t *ppi_config);

/**
 * @brief   Setea un bit del Puerto C.
 * @details Pone en estado alto (1) un bit del Puerto C del PPI 8255.
 *     
 * @param bit   Numero de bit a setear.
 */
void ppi_set_portc_bit(const char bit);

/**
 * @brief   Limpia un bit del Puerto C.
 * @details Pone en estado bajo (0) un bit del Puerto C del PPI 8255.
 *     
 * @param bit   Numero de bit a limpiar.
 */
void ppi_clear_portc_bit(const char bit);
#endif
/**
 * @brief   Retardo de 10 microsegundos.
 * @details Hace un delay de 10 microsegundos a una frecuencia de 4 MHz.
 */
void delay_10us();

/**
 * @brief   Retardo de 100 microsegundos.
 * @details Hace un delay de 100 microsegundos a una frecuencia de 4 MHz.
 */
void delay_100us();

/**
 * @brief   Retardo en milisegundos.
 * @details Hace un retardo en milisegundos a una frecuencia de 4 MHz.
 *     
 * @param ms   Numero de milisegundos a demorar.
 */
void delay_ms(int ms);

/************************************************************************
*                      IMPLEMENTACIÓN DE FUNCIONES                      *
************************************************************************/

void io_write(char port_addr, char data){

    port_addr = port_addr;
    data = data;  
     __asm
        ld  ix, #2
        add ix,sp
        ld  c, (ix)
        inc ix
        ld  a,(ix)
        out (c), a        
    __endasm;
}

///@cond INTERNAL   // Para excluir de la documentación de doxygen
char __ret_aux;     // Variable ausxiliar para devolver el dato de io_read.
///@endcond

char io_read(char port_addr){

    // Se antepone un '_' a los nombres de las
    // variables en C que se vallan a usar en
    // código ensamblador  
    port_addr = port_addr;   
     __asm
        LD  IX, #2
        ADD IX,SP
        LD  C, (IX)
        IN  A,(C)
        LD  (___ret_aux),A 
    __endasm;

    return __ret_aux;
}

void io_write_buffer(char port_addr, char* buffer_out, char count){

    port_addr = port_addr;
    buffer_out = buffer_out;
    count = count;
    __asm
        LD  IX, #2
        ADD IX,SP
        LD  C, (IX)
        INC IX
        LD  L,(IX)
        INC IX
        LD  H,(IX)
        INC IX
        LD  B,(IX)
        OTIR
    __endasm;
}

void io_read_buffer(char port_addr, char* buffer_in, char count){
  
    port_addr = port_addr;
    buffer_in = buffer_in;
    count = count;

     __asm
        LD  IX, #2
        ADD IX,SP
        LD  C, (IX)
        INC IX
        LD  L,(IX)
        INC IX
        LD  H,(IX)
        INC IX
        LD  B,(IX)
        INIR
    __endasm;
}
#if defined (UART_BASE_ADDR)
void uart_init(const uart_cfg_t *uart_config){

    // Programa el Baudrate en la UART
    uart_set_baudrate(uart_config->baudrate);
    // Programa las interrupciones
    URIER = uart_config->interrupt;
    // Programa los parámetros de la UART (tamaño de dato, paridad, bits de parada)
    URLCR = (uart_config->stop_bits) | (uart_config->parity) | (uart_config->word_length);
}

void uart_set_baudrate(const uart_baudrate_t baudrate){

    // DLAB = 1
    URLCR |= BV(UDLAB);
    // Divisor de baudaje bajo
    URDLL = baudrate;
    // Divisor de baudaje alto
    URDLM = ((uint16_t)baudrate)>>8;
    // DLAB = 0
    URLCR &= ~BV(UDLAB);
}

void uart_write(char c){

    // Mientras el registro transmisor no esté vacío...
    while( !(URLSR & BV(UTHRE)))
        NOP();    
    // Cargamos el dato a enviar en el registro transmisor.
    URTHR = c;
}

char uart_read(){

    // Mientras no se haya recibido un dato... 
    while(!(URLSR & BV(UDR))) 
        NOP();
    // Devuelve el dato recibido.
    return URRBR;
}

void uart_print(const char* str){

    // Mientras no sea \0
    while(*str)       
       putchar(*str++); // envía el siguiente caracter. 
}

int uart_read_line(char* str){

    int n=0;
    char c;
    while(n<MAXLINE-1 && (c=getchar()) != '\n' && c !='\r'){
        // Si es BACKSPACE (0x7F o 0x08)
        if(c == 0x7F || c==0x08){
        
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
       
    str[n]='\0';     
    putchar('\n');
    return n;
}

void uart_disable_interrupts(){

    URIER = 0;
}

void uart_enable_interrupts(uart_interrupt_t int_cfg){

    URIER = int_cfg;
}
#endif

#if defined (PPI_BASE_ADDR)
void ppi_init(const ppi_cfg_t *ppi_config){

    PPI_CTRL = 0x80 | ppi_config->mode | (ppi_config->pcl_dir << PCPCL) | (ppi_config->pch_dir << PCPCH) | (ppi_config->pa_dir << PCPA) | (ppi_config->pb_dir << PCPB);
}

void ppi_set_portc_bit(const char bit){

    PPI_CTRL = 1 | bit << 1;
}

void ppi_clear_portc_bit(const char bit){
  
    PPI_CTRL = bit << 1;
}
#endif
void delay_10us(){

    __asm
            EXX
            EX      AF,AF'
            LD      B,#0x4
    LOOP_10:
            DJNZ    LOOP_10
            EX      AF,AF'
            EXX
    __endasm;
}


void delay_100us(){

    __asm
            EXX
            EX      AF,AF'
            LD      B,#0x3A
    LOOP_100:
            DJNZ    LOOP_100
            EX      AF,AF'
            EXX
    __endasm;
}
 

void delay_ms(int ms){

    int i;

    while(ms--)
        for(i=0;i<0x10A;i++)
            __asm__("nop");
}


/*
    Implementación de funciones putchar y getchar para ser usadas con la libreria stdio.h y la UART
*/
void putchar(char c){

    #if defined (UART_BASE_ADDR)
    if(c=='\n')
        uart_write('\r');
    uart_write(c);
    #endif
}


char getchar(){

    #if defined (UART_BASE_ADDR)
    return uart_read();
    #else
    return 0;
    #endif
}
/*******************************************************************************/
#endif // SMZ80_H_
