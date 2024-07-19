/** \file main.c
*
* @brief Embedded Software Boot Camp
*
* @par
* COPYRIGHT NOTICE: (C) 2014 Barr Group, LLC.
* All rights reserved.
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  "cpu_cfg.h"
#include  "bsp_cfg.h"
#include  "assert.h"

#include  <cpu_core.h>
#include  <os.h>
#include <stdio.h>
#include  <bsp_glcd.h>

#include  "bsp.h"
#include  "bsp_int_vect_tbl.h"
#include  "bsp_led.h"
#include  "os_app_hooks.h"

#include "pushbutton.h"
#include "adc.h"
#include "alarm.h"


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

// Relative Task Priorities (0 = highest; 15 = idle task)
#define  LED5_PRIO              14   // Every 500 ms, in a timed loop.

// Allocate Task Stacks
#define  TASK_STACK_SIZE      128

static CPU_STK  g_led4_stack[TASK_STACK_SIZE]; // global array with a size of 128
static CPU_STK g_led6_stack[TASK_STACK_SIZE]; 
static CPU_STK g_debounce_stack[TASK_STACK_SIZE];
static CPU_STK g_button1_stack[TASK_STACK_SIZE];
static CPU_STK g_button2_stack[TASK_STACK_SIZE];
static CPU_STK g_adc_stack[TASK_STACK_SIZE];
static CPU_STK g_alarm_stack[TASK_STACK_SIZE];
static CPU_STK g_startup_stack[TASK_STACK_SIZE];
static CPU_STK g_dive_stack[TASK_STACK_SIZE];

// Allocate Task Control Blocks
static OS_TCB   g_led4_tcb; // global variable for every task
static OS_TCB g_led6_tcb;
static OS_TCB g_debounce_tcb;
static OS_TCB g_button1_tcb;
static OS_TCB g_button2_tcb;
static OS_TCB g_adc_tcb;
static OS_TCB g_alarm_tcb;
static OS_TCB g_startup_tcb;
static OS_TCB g_dive_tcb;

static OS_MUTEX g_led_mutex;

uint8_t g_display_feet = 0;
static uint32_t g_elapsed_dive_time = 0;
extern int32_t g_depth_mm;
int32_t g_air_tank_level = 50;


/*
*********************************************************************************************************
*                                            LOCAL MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void dive_task(void *p_arg) {
    OS_ERR err;
    char p_str[13];

    for (;;) {
        OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err); // Wait for 1 second

        // Diver goes underwater
        if (g_depth_mm < 0) { 
            g_elapsed_dive_time++;
            
            // Subtract some amount of air
            uint32_t used_air = (gas_rate_in_cl(g_depth_mm)) / 100;
            g_air_tank_level -= used_air;
            if (g_air_tank_level <= 0) {
              g_air_tank_level = 0;
            }

            // Display elapsed dive time
            sprintf(p_str, "EDT: %02d:%02d:%02d", 
                    g_elapsed_dive_time / 3600,
                    (g_elapsed_dive_time % 3600) / 60,
                    g_elapsed_dive_time % 60);
            BSP_GraphLCD_String(7, p_str);
        }
        
        // Diver is at the surface
        if (g_depth_mm == 0) {
          g_elapsed_dive_time = 0;
          
          // Reset elapsed dive time
          sprintf(p_str, "EDT: %02d:%02d:%02d", 
                    g_elapsed_dive_time / 3600,
                    (g_elapsed_dive_time % 3600) / 60,
                    g_elapsed_dive_time % 60);
            BSP_GraphLCD_String(7, p_str);
        }
    }
}

void button1_task (void * p_arg) { // receives button 1 press and reacts to it
  OS_ERR err;
  
  for (;;) {
    // SW1 has been pressed
    OSSemPend(&g_sw1_sem, 0, OS_OPT_PEND_BLOCKING, (void *)0, &err);
    
    // Add air into the tank when button is pressed
    if (g_depth_mm == 0) {
      g_air_tank_level += 5;
    }
    
    // Limit the maximum amount of air to 2000L
    if (g_air_tank_level >= 2000) {
      g_air_tank_level = 2000;
    }

  }
}

void button2_task (void * p_arg) { // receives button 2 press and reacts to it
  OS_ERR err;
  
  for (;;) {
    // SW2 has been pressed
    OSSemPend(&g_sw2_sem, 0, OS_OPT_PEND_BLOCKING, (void *)0, &err);
    g_display_feet++;
    
    g_display_feet = g_display_feet % 2;
  }
}

void led4_task (void * p_arg) // flashes LED 5 
{
    OS_ERR  err;
    (void)p_arg;    // NOTE: Silence compiler warning about unused param.

    for (;;)
    {
      // add mutex
        OSMutexPend(&g_led_mutex, 0, OS_OPT_PEND_BLOCKING, (void *)0, &err);
        
        // Flash LED at 1 Hz.
	BSP_LED_Toggle(4);
        OSTimeDlyHMSM(0, 0, 0, 333, OS_OPT_TIME_HMSM_STRICT, &err);
        
        // remove mutex
        OSMutexPost(&g_led_mutex, OS_OPT_POST_NONE, &err);
        
        
    }
}

void led6_task (void * p_arg) {
  OS_ERR err;
  (void)p_arg;
  
  for (;;) {
    // add mutex
    OSMutexPend(&g_led_mutex, 0, OS_OPT_PEND_BLOCKING, (void *)0, &err);
    
    // Flash LED at 3 Hz.
    BSP_LED_Toggle(6);
    OSTimeDlyHMSM(0, 0, 0, 333, OS_OPT_TIME_HMSM_STRICT, &err);
    
    // remove mutex
    OSMutexPost(&g_led_mutex, OS_OPT_POST_NONE, &err);
  }
}

void startup_task (void * p_arg) {
  OS_ERR err;
    
  // hardware initialization
  BSP_Init();
  CPU_Init();
  Mem_Init();
  BSP_GraphLCD_SetFont(GLYPH_FONT_8_BY_8);
  
  // create mutex
  OSMutexCreate(&g_led_mutex, "LED Mutex", &err);
  
  OSSemCreate(&g_sw1_sem, "Semaphore Switch 1", 0, &err);
  OSSemCreate(&g_sw2_sem, "Semaphore Switch 2", 0, &err);
  
  // create LED4 task
  OSTaskCreate(&g_led4_tcb, // Address of the TCB assigned to the task
                 "LED4 Task", // Name of the task
                 led4_task,    // Address of the task itself
                 (void *)0,    // task does not need data, null pointer assigned
                 14,           // Priority of the task
                 &g_led4_stack[0], // Base address of the task's stack
                 10,                // Watermark limit for stack growth
                 128,          // Stack size
                 5,            // Size of task message queue
                 10,           // Time quanta (in number of ticks)
                 (void *)0,    // Extension pointer is not used
                 OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR, // Options
                 &err);         // error code
  
  // create LED6 task
  OSTaskCreate(&g_led6_tcb,
               "LED6 Task",
               led6_task,
               (void *)0,
               13,
               &g_led6_stack[0],
               10,
               128,
               5,
               10,
               (void *)0,
               OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR,
               &err);
  
  // create button 1 task
  OSTaskCreate(&g_button1_tcb,
               "Button1 Task",
               button1_task,
               (void *)0,
               12,
               &g_button1_stack[0],
               10,
               TASK_STACK_SIZE,
               5,
               10,
               (void *)0,
               OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR,
               &err);
  
  // create button 2 task
  OSTaskCreate(&g_button2_tcb,
               "Button2 Task",
               button2_task,
               (void *)0,
               12,
               &g_button2_stack[0],
               10,
               128,
               5,
               10,
               (void *)0,
               OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR,
               &err);
  
    // create button debounce task
  OSTaskCreate(&g_debounce_tcb,
               "Debouncing Task",
               debounce_task,
               (void *)0,
               11,
               &g_debounce_stack[0],
               10,
               128,
               5,
               10,
               (void *)0,
               OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR,
               &err);
  
  // create adc task
  OSTaskCreate(&g_adc_tcb,
               "ADC Task",
               adc_task,
               (void *)0,
               8,
               &g_adc_stack[0],
               10,
               128,
               5,
               10,
               (void *)0,
               OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR,
               &err);
  
  // create alarm task
  OSTaskCreate(&g_alarm_tcb,
               "Alarm Task",
               alarm_task,
               (void *)0,
               10,
               &g_alarm_stack[0],
               10,
               128,
               5,
               10,
               (void *)0,
               OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR,
               &err); 
  
  OSTaskCreate(&g_dive_tcb,
             "Dive Task",
             dive_task,
             (void *)0,
             9, // Adjust priority as needed
             &g_dive_stack[0],
             10,
             TASK_STACK_SIZE,
             5,
             10,
             (void *)0,
             OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR,
             &err);

  // delete startup task after initialization
  OSTaskDel((OS_TCB *)0, &err);
}


/*
*********************************************************************************************************
*                                               main()
*
* Description : Entry point for C code.
*
* Arguments   : none.
*
* Returns     : none.
*
* Note(s)     : (1) It is assumed that your code will call main() once you have performed all necessary
*                   initialization.
*********************************************************************************************************
*/

void  main (void)
{
    OS_ERR  err;

    CPU_IntDis();                                               /* Disable all interrupts.                              */

    BSP_IntVectSet(27, (CPU_FNCT_VOID)OSCtxSwISR);              /* Setup kernel context switch                          */

    // Initialize the operating system's internal data structures
    OSInit(&err);
    
    // Create startup task
    OSTaskCreate(&g_startup_tcb,
                 "Startup Task",
                 startup_task,
                 (void *)0,
                 14,
                 &g_startup_stack[0],
                 10,
                 128,
                 5,
                 10,
                 (void *)0,
                 OS_OPT_TASK_STK_CHK + OS_OPT_TASK_STK_CLR,
                 &err);
    
    // TODO: Start multitasking
    OSStart(&err);

    // We should never get here.
    assert(0);
}

