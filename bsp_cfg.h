/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*                          (c) Copyright 2009-2014; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                         BOARD SUPPORT PACKAGE
*
*                                            Renesas RX63N
*                                               on the
*                                              YRDKRX63N
*                                          Evaluation Board
*
* Filename      : bsp_cfg.h
* Version       : V1.00
* Programmer(s) : JM
*********************************************************************************************************
*/

#ifndef  BSP_CFG_H_
#define  BSP_CFG_H_


/*
*********************************************************************************************************
*                                            MODULES ENABLE
*********************************************************************************************************
*/

#define  BSP_CFG_CAN_EN                     0    /* Enable (1) or Disable (0) uC/CAN                   */
#define  BSP_CFG_CAN_OPEN_EN                0    /* Enable (1) or Disable (0) uC/CANopen               */
#define  BSP_CFG_FS_EN                      0    /* Enable (1) or Disable (0) uC/FS                    */
#define  BSP_CFG_GUI_EN                     0    /* Enable (1) or Disable (0) uC/GUI                   */
#define  BSP_CFG_MBM_EN                     0    /* Enable (1) or Disable (0) uC/Modbus-Master         */
#define  BSP_CFG_MBS_EN                     0    /* Enable (1) or Disable (0) uC/Modbus-Slave          */
#define  BSP_CFG_NET_EN                     0    /* Enable (1) or Disable (0) uC/TCP-IP                */
#define  BSP_CFG_OS2_EN                     0    /* Enable (1) or Disable (0) uC/OS-II                 */
#define  BSP_CFG_OS3_EN                     1    /* Enable (1) or Disable (0) uC/OS-III                */
#define  BSP_CFG_USBD_EN                    0    /* Enable (1) or Disable (0) uC/USB-D                 */
#define  BSP_CFG_USBH_EN                    0    /* Enable (1) or Disable (0) uC/USB-H                 */


/*
*********************************************************************************************************
*                                        INTERRUPT VECTOR TABLE
*********************************************************************************************************
*/

#define  BSP_CFG_INT_VECT_TBL_RAM_EN        0    /* Enable (1) vector table in RAM or in ROM (0)       */
#define  BSP_CFG_INT_VECT_TBL_SIZE        256    /* Max. number of entries in the interrupt vector tbl */


/*
*********************************************************************************************************
*                                          CLOCK MANAGEMENT
*********************************************************************************************************
*/

#define  BSP_CFG_SYS_EXT_CLK_FREQ    12000000uL


/*
*********************************************************************************************************
*                                        BOARD SPECIFIC BSPs
*********************************************************************************************************
*/

#define  BSP_CFG_SER_EN                     0    /* Enable (1) or Disable (0) Serial port BSP          */

#define  BSP_CFG_LED_EN                     1    /* Enable (1) or Disable (0) LEDs                     */

#define  BSP_CFG_GRAPH_LCD_EN               1

#define  BSP_CFG_MISC_EN                    1


/*
*********************************************************************************************************
*                                     INTERRUPT PRIORITY LEVELS
*********************************************************************************************************
*/

#define  BSP_CFG_OS_TICK_IPL                3

#if      BSP_CFG_SER_EN > 0
#define  BSP_CFG_SER_IPL                    4
#endif
#endif
