 /***************************************************************************************
 *   z80interrupt.h
 *
 *         Versión:  1.0
 *
 *           Autor:  Alfredo Orozco de la Paz
 *         Archivo:  z80interrupt.h
 *           Fecha:  Agosto 15, 2014
 *
 *      Procesador:  Z80 CPU
 *         Familia:  Zilog
 *      Compilador:  SDCC
 *
 *  ############################# Última Modficación #############################
 *      
 *  ##############################################################################
 *  
 ***************************************************************************************
 *
 *   La  librería incluye macros  y funciones para manejar las interrupciones del 
 *   Z80.
 *
 *   Define las funciones de servicio de interrupción, macros de habilitación,
 *   deshabilitación y cambio de modo de las interrupciones.
 * 
 *   USO:
 *
 *   Macros para interrupciones:
 *   
 *       EI()       Macro que habilita la interrupción INT.
 *       DI()       Macro que deshabilita la interrupción INT.
 *       IM(MODE)   Macro para cambiar el modo de interrupción INT.
 *
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
 *   Nota: Para hacer uso de las  interrupciones es  necesario descomentar el  o  los
 *   vectores y subrutinas de interrupción de interés del archivo "sysmap.asm".
 *
 ***************************************************************************************/
 
#ifndef _Z80INTERRUPT_H_
#define _Z80INTERRUPT_H_

/** @file
 *
 * @defgroup z80interrupt Z80 Interrupt
 * @{
 * @ingroup z80lib
 *
 * @brief   Driver para el chip UART 8250 de Intel.
 *
 * @details Este archivo contiene declaraciones de funciones, definiciones de tipos,
 *          enumeraciones y macros usadas para la programación y uso de la UART 8250
 *          de Intel.
 *          
 *          Nota: Para hacer uso de las interrupciones es necesario descomentar el o 
 *          los vectores y subrutinas de interrupción de interés en el archivo "sysmap.asm".
 *          
 *          Versión 1.0
 *          
 *          Autor:  Alfredo Orozco de la Paz    \n
 *          e-mail: alfredoopa@gmail.com        \n
 *          www.vidaembevida.wordpress.com
 */


#define ISR_NMI()     void isr_vector66()  __critical __interrupt   /**< Función para el vector de interrupción NO ENMASCARABLE 0x66 */
#define ISR_INT_08()  void isr_vector08()  __interrupt 1    /**< Función para el vector de interrupción 0x08 */
#define ISR_INT_10()  void isr_vector10()  __interrupt 2    /**< Función para el vector de interrupción 0x10 */
#define ISR_INT_18()  void isr_vector18()  __interrupt 3    /**< Función para el vector de interrupción 0x18 */
#define ISR_INT_20()  void isr_vector20()  __interrupt 4    /**< Función para el vector de interrupción 0x20 */
#define ISR_INT_28()  void isr_vector28()  __interrupt 5    /**< Función para el vector de interrupción 0x28 */
#define ISR_INT_30()  void isr_vector30()  __interrupt 6    /**< Función para el vector de interrupción 0x30 */
#define ISR_INT_38()  void isr_vector38()  __interrupt 7    /**< Función para el vector de interrupción 0x38 */

#define EI()    __asm__ ("EI")  /**< Habilita interrupción INT */
#define DI()    __asm__ ("DI")  /**< Deshabilita interrupción INT */
#define IM(MODE) __asm\
                    IM MODE\
                __endasm        /**< Cambia el modo de la interrupción INT */

#endif // _Z80INTERRUPT_H_

