/** \file pushbutton.h
*
* @brief Button Debouncer
*
* @par
* COPYRIGHT NOTICE: (C) 2011 Netrino, LLC.
* All rights reserved.
*/

#ifndef _PUSHBUTTON_H
#define _PUSHBUTTON_H

extern OS_SEM g_sw1_sem;
extern OS_SEM g_sw2_sem;

void  debounce_task(void * p_arg);

#endif /* _PUSHBUTTON_H */
