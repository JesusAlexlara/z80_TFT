 /***************************************************************************************
 *   z80utils.h
 *
 *         Versión:  1.0
 *
 *           Autor:  Alfredo Orozco de la Paz
 *         Archivo:  z80utils.h
 *           Fecha:  Agosto 15, 2014
 *
 *      Procesador:  Z80 CPU
 *         Familia:  Zilog
 *      Compilador:  SDCC
 *
 *  ############################# Última Modficación #############################     
 *  Diciembre 10 del 2014
 *      - Agregada la macro RST(VECTR) para la instruccion RST en ensamblador.
 *      - Agregada la macro BV(BIT), que devuelve la máscara de un
 *        bit en un registro. EJ: BV(2) = (1<<2).      
 *  ##############################################################################
 *  
 ***************************************************************************************
 *   La  librería incluye macros  para  declarar  variables  a puertos  de entrada y 
 *   salida, entrar en modo HALT, operaciones NOP, definiciones de tipos de datos
 *   entre otras utlidades.
 *
 *
 *   Macros:
 *   
 *       NOP()          Macro que ejecuta una instrucción NOP en ensamblador.
 *       HALT()         Macro que ejecuta la instrucción HALT en ensamblador.
 *       Sleep()        Macro que ejecuta la instrucción HALT en ensamblador.
 *       RST(VECTR)     Ejecuta la instruccion RST al vector VECTR.
 *       BV(NUM_BIT)    Obtiene la máscara de la posición de un bit.
 *                       BV(4) =  1 << 4 =  00010000;
 *
 *   Tipos de datos:
 *       byte -> 8 bits
 *       Byte -> 8 bits
 *       Word -> 16 bits
 *       word -> 16 bits
 *       DWord -> 32 bits
 *       dword -> 32 bits
 *       
 *   Macro para declara una variable a puerto de E/S:
 *
 *   SFR_IO  (PORT_ADDRES)  VAR_NAME;
 *       -> PORT_ADDRES : direccion del puerto de E/S.
 *       -> VAR_NAME    : nombre de la variable.
 *   
 ****************************************************************************************
 *    Copyright:
 *
 *        Copyrigth© 2014
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

#ifndef _Z80UTILS_H_
#define _Z80UTILS_H_

/** @file
 *
 * @defgroup z80utils Z80 Utils
 * @{
 * @ingroup z80lib
 *
 * @brief   Definiciones de funciones y utilidades para el Z80 CPU
 *
 * @details Este archivo contiene declaraciones de macros y definiciones
 *          para realizar algunas operaciones específicas del Z80 CPU.
 *          
 *          Versión 1.0
 *          
 *          Autor:  Alfredo Orozco de la Paz    \n
 *          e-mail: alfredoopa@gmail.com        \n
 *          www.vidaembevida.wordpress.com
 */

#include <stdint.h>

#define TRUE 1      ///< Definición para verdadero.
#define FALSE 0     ///< Definición para falso.

#define INPUT TRUE      ///< Definición para Entrada.
#define OUTPUT FALSE    ///< Definición para Salida.

#define LOW FALSE       ///< Definición para Bajo.
#define HIGH TRUE       ///< Definición para Alto.

#define HALT()  __asm__ ("HALT")    ///< Macro para ejecutar la instrucción HALT del Z80.
#define NOP()   __asm__ ("NOP")     ///< Macro para ejecutar la instrucción NOP del Z80.
#define sleep() __asm__ ("HALT")    ///< Macro para poner al Z80 en modo Sleep.

#define BV(BIT) (1<<(BIT))          ///< Macro que obtiene la mascara de bits para el bit BIT.
#define SFR_IO __sfr __at           ///< Macro para declarar variables a puertos de E/S.

#define RST(VECTR) __asm\
                     RST VECTR\
                   __endasm        ///< Macro para ejecutar la instrucción RST a un vector de interrupción del Z80.

/*typedef uint8_t Byte;   ///< Definición de tipo de dato byte.
typedef uint8_t byte;   ///< Definición de tipo de dato byte.
typedef uint16_t Word;  ///< Definición de tipo de dato word.
typedef uint16_t word;  ///< Definición de tipo de dato word.
typedef uint32_t DWord; ///< Definición de tipo de dato double word.
typedef uint32_t dword; ///< Definición de tipo de dato double word.*/


#endif // _Z80BASICS_H_
