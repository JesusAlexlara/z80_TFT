/***************************************************************************
 *  z80ppi.h
 *
 *         Versión:  1.0
 *
 *           Autor:  Alfredo Orozco de la Paz
 *         Archivo:  z80ppi.h
 *           Fecha:  Oct 15, 2014
 *
 *      Procesador:  Z80 CPU
 *         Familia:  Zilog
 *      Compilador:  SDCC
 *
 *    ############################# Última Modficación #############################
 *
 *    ##############################################################################
 ***************************************************************************************
 *  Descripción:
 *
 *      Librería  con definiciones para el manejo de un PPI 8255 de intel.
 *      La librería define constantes para el direccionamento del PPI y sus
 *      puertos, define variables para leer y escribir en los puertos asi
 *      como sus bits correspondientes a cada puerto.
 *
 ***************************************************************************************
 *
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

#ifndef _Z80PPI_H_
#define _Z80PPI_H_

/** @file
 *
 * @defgroup z80ppi Z80 PPI 8255
 * @{
 * @ingroup z80lib
 *
 * @brief   Driver para el chip PPI 8255 de Intel.
 *
 * @details Librería  con definiciones para el manejo de un PPI 8255 de intel.\n
 *          La librería define constantes para el direccionamento del PPI y sus
 *          puertos, define variables para leer y escribir en los puertos asi
 *          como sus bits correspondientes a cada puerto.
 *
 *          Versión 1.0
 *
 *          Autor:  Alfredo Orozco de la Paz    \n
 *          e-mail: alfredoopa@gmail.com        \n
 *          www.vidaembevida.wordpress.com
 */


/************************************************************************
*                       INCLUSION DE LIBRERIAS
************************************************************************/
#include "z80utils.h"

/************************************************************************
*                       DEFINICION DE CONSTANTES
************************************************************************/
// Direccionamiento del PPI
#ifndef PPI_BASE_ADDR
#define PPI_BASE_ADDR    0x40   /**< Definición de la dirección base del PPI 8255.\n Si no se define se usa 0x00 por default.*/
#endif

/***********************************************
 *     Direcciones de los Registros del PPI
 ***********************************************/
#define PPI_PORTA_ADDR      PPI_BASE_ADDR + 0x00      /**< Dirección del Puerto A del PPI */
#define PPI_PORTB_ADDR      PPI_BASE_ADDR + 0x01      /**< Dirección del Puerto B del PPI */
#define PPI_PORTC_ADDR      PPI_BASE_ADDR + 0x02      /**< Dirección del Puerto C del PPI */
#define PPI_CTRL_ADDR       PPI_BASE_ADDR + 0x03      /**< Dirección del Registro de Control del PPI */
/***********************************************/

/***********************************************
 *      Variables de los registros del PPI
 ***********************************************/

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

#define PCPCL   0   /**< Numero de bit para el Control Word PCL bit */
#define PCPCH   3   /**< Numero de bit para el Control Word PCA bit */
#define PCPA    4   /**< Numero de bit para el PPI Control Word PA bit */
#define PCPB    1   /**< Numero de bit para el PPI Control Word PB bit */
#define PCMB    2   /**< Numero de bit para el PPI Control Word Mode Group B bit */
#define PCMA0   5   /**< Numero de bit para el PPI Control Word Mode Group A bit 0 */
#define PCMA1   6   /**< Numero de bit para el PPI Control Word Mode Group A bit 1 */
#define PCME    7   /**< Numero de bit para el PPI Control Word Mode Enable */



#endif  // Fin _Z80PPI_H_
