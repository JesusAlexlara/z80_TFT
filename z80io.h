
/***************************************************************
*   z80io.h
*
*   Libreria  con  definiciones  de  los  puertos y registros
*   del PPI 8255 y de la  UART 8250 con una interfaz  para el 
*   z80 y el compilador SDCC.
*
*   La libreria declara las variables de acceso a los puertos
*   del z80. Estas variables estan declaradas con valores por
*   default para un mapa  de direcciones  donde los registros 
*   del PPI se encuentran en las direcciones 0x00 a 0x03 y de
*   la UART en las direcciones 0x10 a 0x16. En  este  caso la
*   direccion de seleccion del PPI ex 0x00 y de la UART 0x10.
*
*   Si el mapeo se encuentra en otras direcciones  distitas a 
*   las declaradas por default, debe ser necesario definirlas
*   antes de hacer la inclusion de la libreria.
*   
*   Ejemplo:
*
*   #define PPI_ADDR  0x50   // Direccionamiento del PPI en la dirección 0x50
*
*   #define UART_ADDR 0x30   // Direccionamiento de la UART en la dirección 0x30
*           
*   #include "z80ppi.h"
*           .
*           .
*
*   
*
*   La  librería incluye macros  para  declarar  variables  a
*   puertos  de entrada y salida,  habilitar  y  deshabilitar
*   interrupciones, cambiar el modo de interrupciones, entrar
*   en modo HALT y macros a funciones de servicion de interr-
*   upción.
*
*   
*   USO:
*
*   Macros para interrupciones:
*   
*       EI();      Habilita los flags de interrupciones INT.
*       DI();      Deshabilita las interrupciones INT.
*       IM(MODE)   Cambiael modo de interrupcion INT.
*
*   Macro para declara una variable a puerto de E/S:
*
*   SFR_IO  (PORT_ADDRES)  VAR_NAME;
*       -> PORT_ADDRES : direccion del puerto de E/S.
*       -> VAR_NAME    : nombre de la variable.
*   
*   Macros para funciones a servicio de interrupción:
*   
*   ISR_NMI()
*       -> Declara una función para atender la interrupción NMI
*          (vector 0x66).
*   
*   ISR_INT_xx()
*       -> Declara una función para atender una interrupción INT
*          xx representa el vector de la interrupción.
*            
*                  xx = puede ser 08,10,28,20,28,30 o 38
*
*
*       Ej:
*           ISR_NMI()
*           {
*               int a = 5;
*               int b = 6;
*               a = a*b;
*           }
*
*
*           ISR_INT_38()
*           {
*               int x = 2;
*               int y = 45;
*               x *= y;
*
*               EI();
*           }
*
*
*   Nota: Para hacer uso de las  interrupciones es  necesario
*   descomentar el o los vectores de interrupción de  interés
*   del archivo "sysmap.asm"
*
****************************************************************
*
*   Escrito por:    Alfredo Orozco de la Paz
*   e-mail:         alfredoopa@gmail.com
*
**************************************************************/

#ifndef _Z80IO_H_
#define _Z80IO_H_

#include <stdint.h>

#define TRUE 1
#define FALSE 0

#define INPUT TRUE
#define OUTPUT FALSE

#define LOW FALSE
#define HIGH TRUE

//#define HALT() __asm__ ("halt")

#define EI() __asm__ ("EI")
#define DI() __asm__ ("DI")
#define NOP() __asm__ ("NOP")

#define IM(MODE) __asm\
                    IM MODE\
                __endasm

/*#define RST(ADDR) __asm\
                    RST ADDR\
                __endasm*/

#define SFR_IO __sfr __at

#define ISR_NMI()     void isr_vector66()  __critical __interrupt
#define ISR_INT_08()  void isr_vector08()  __interrupt 1
#define ISR_INT_10()  void isr_vector10()  __interrupt 2
#define ISR_INT_18()  void isr_vector18()  __interrupt 3
#define ISR_INT_20()  void isr_vector20()  __interrupt 4
#define ISR_INT_28()  void isr_vector28()  __interrupt 5
#define ISR_INT_30()  void isr_vector30()  __interrupt 6
#define ISR_INT_38()  void isr_vector38()  __interrupt 7


typedef uint8_t Byte;
typedef uint8_t byte;
typedef uint16_t Word;
typedef uint16_t word;
typedef uint32_t DWord;
typedef uint32_t dword;



#ifndef PPI_ADDR
#define PPI_ADDR    0x70   // Direccionamiento del PPI
#endif


#ifndef UART_ADDR
#define UART_ADDR   0x20   // Direccionamiento de la UART
#endif


/***********************************************
 *   Direcciones de los Registros de la UART
 ***********************************************/
#define UART_RBR_ADDR	UART_ADDR | 0x00// Dir. Registro RX (DLAB=0)
#define UART_THR_ADDR	UART_ADDR | 0x00// Dir. Registro TX (DLAB=0)
#define UART_IER_ADDR	UART_ADDR | 0x01// Dir. Registro Interrupt Enable (DLAB=0)
#define UART_IIR_ADDR	UART_ADDR | 0x02// Dir. Registro Interrupt Identification
#define UART_LCR_ADDR	UART_ADDR | 0x03// Dir. Registro Line Control
#define UART_MCR_ADDR	UART_ADDR | 0x04// Dir. Registro Modem Control
#define UART_LSR_ADDR	UART_ADDR | 0x05// Dir. Registro Line Status
#define UART_MSR_ADDR	UART_ADDR | 0x06// Dir. Registro Modem Status
#define UART_DLL_ADDR	UART_ADDR | 0x00// Dir. Registro Divisor Latch LSB (DLAB=1)
#define UART_DLM_ADDR	UART_ADDR | 0x01// Dir. Registro Divisor Latch MSB (DLAB=1)
/***********************************************/

/***********************************************
 *          Variables de los registros     
 ***********************************************/
// Registro RBR - Receiver Buffer Register
SFR_IO  UART_RBR_ADDR  URBR; 
// Registro THR - Transmit Hold Register
SFR_IO  UART_THR_ADDR  UTHR;

// Registro IER - Interrupt Enable Register
SFR_IO  UART_IER_ADDR  UIER;  
#define UERBFI   0x01 // Enabled Received Data Availabre Interrupt
#define UETBEI   0x02 // Transmitter Holding Register Empty Interrupt
#define UELSI    0x04 // Enable Receiver Line Status Interrupt
#define UEDSSI   0x08 // Enable MODEM Status Interrupt

// Registro IIR - Interrupt Identification Register
SFR_IO  UART_IIR_ADDR  UIIR;  
#define UIP      0x01 // Interrupt Pending
#define UIID0    0x02 // Interrupt ID bit 0
#define UIID1    0x04 // Interrupt ID bit 1

// Registro LCR - Line Control Register
SFR_IO  UART_LCR_ADDR  ULCR;
#define UWLS0    0x01 // Word Length Select Bit 0
#define UWLS1    0x02 // Word Length Select Bit 1
#define USTB     0x04 // Number of Stop Bits
#define UPEN     0x08 // Parity Enable
#define UEPS     0x10 // Even Parity Select
#define USTP     0x20 // Stick Parity
#define USBK     0x40 // Set Break
#define UDLAB    0x80 // Divisor Latch Acces Bit

// Registro LSR - Line Status Register
SFR_IO  UART_LSR_ADDR  ULSR;  
#define UDR      0x01 // Data Ready
#define UOR      0x02 // Overrun Error
#define UPE      0x04 // Parity Error
#define UFE      0x08 // Framming Error
#define UBI      0x10 // Break Interrupt
#define UTHRE    0x20 // Transmitter Holding Register Empty
#define UTSRE    0x40 // Transmitter Shift Register Empty
#define UTEMT    0x40 // Transmitter Register Empty

// Registro MCR - Modem Control Register
SFR_IO  UART_MCR_ADDR  UMCR;  
#define UDTR      0x01 // Data Terminal Ready
#define URTS      0x02 // Request to Send
#define UOUT1     0x04 // Out 1
#define UOUT2     0x08 // Out 2
#define ULOOP     0x10 // Loop

// Registro MSR - Modem Status Register
SFR_IO  UART_MSR_ADDR  UMSR;

// Registro DLL - Divisor Latch LSB
SFR_IO  UART_DLL_ADDR  UDLL;  

// Registro DLM - Divisor Latch MSB
SFR_IO  UART_DLM_ADDR  UDLM;  
/***********************************************/



 
 
/***********************************************
 *     Direcciones de los Registros del PPI
 ***********************************************/
//#define PPI_PA_ADDR    PPI_ADDR | 0x00// Dir. Puerto A
//#define PPI_PB_ADDR    PPI_ADDR | 0x01// Dir. Puerto B
//#define PPI_PC_ADDR    PPI_ADDR | 0x02// Dir. Puerto C
//#define PPI_CTRL_ADDR  PPI_ADDR | 0x03// Dir. Registro de Control
/***********************************************/

/***********************************************
 *          Variables de los registros     
 ***********************************************/
//SFR_IO  PPI_PA_ADDR    PPA;    // Registro del Puerto A
//SFR_IO  PPI_PB_ADDR    PPB;    // Registro del Puerto B
//SFR_IO  PPI_PC_ADDR    PPC;    // Registro del Puerto C

//SFR_IO  PPI_CTRL_ADDR  PCTRL;  // Registro de control
//#define PCPCL   0x01 // PPI Control Word PCL bit
//#define PCPCH   0x08 // PPI Control Word PCA bit
//#define PCPA    0x10 // PPI Control Word PA bit
//#define PCPB    0x02 // PPI Control Word PB bit
//#define PCMB    0x04 // PPI Control Word Mode Group B bit
//#define PCMA0   0x20 // PPI Control Word Mode Group A bit 0
//#define PCMA1   0x40 // PPI Control Word Mode Group A bit 1
//#define PCME    0x80 // PPI Control Word Mode Enable
/***********************************************/


//###############################################################

/***********************************************
 *  Funciones a bajo nivel para envío de
 *  datos por un puerto.
 ********************************************/
static byte io_val;
static byte io_addr;

void io_write(byte addr, byte data){

    io_addr=addr;
    io_val=data;

    // Se antepone un '_' a los nombres de las
    // variables en C que se vallan a usar en
    // código ensamblador
    
     __asm
        EXX
        EX AF,AF'
        LD A,(_io_val)
        LD BC,(_io_addr)
        OUT (C),A
        EX AF,AF'
        EXX
    __endasm;
}

char io_read(byte addr){

    io_addr=addr;

    // Se antepone un '_' a los nombres de las
    // variables en C que se vallan a usar en
    // código ensamblador
    
     __asm
        EXX
        EX AF,AF'
        LD BC,(_io_addr)
        IN A,(C)
        LD (_io_val),A
        EX AF,AF'
        EXX
    __endasm;

    return io_val;
}



//###############################################################
#endif
