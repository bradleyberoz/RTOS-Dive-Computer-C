/** \file alarm.c
*
* @brief Alarm Manager
*
* @par
* COPYRIGHT NOTICE: (c) 2014 Barr Group, LLC.
* All rights reserved.
*/

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
			
#include "os.h"
#include "iorx63n.h"

#include "alarm.h"


// PWM Prescalars
typedef enum { TONE_HI = 450, TONE_MED = 700, TONE_LO = 950 } pwm_t;

// Waveform Data Structure
typedef struct
{
	pwm_t	    tone1;      // PWM prescalar for Tone 1.
	pwm_t	    tone2;      // PWM prescalar for Tone 2.
	uint16_t    interval;   // Seconds to play each tone, unless chopped.
	uint8_t	    b_chopped;  // Ignore interval, use on/off chop instead.

} wave_t;

const wave_t   alarm_low    = { TONE_LO,  TONE_MED, 2, 0};
const wave_t   alarm_medium = { TONE_MED, TONE_HI,  1, 0};
const wave_t   alarm_high   = { TONE_LO,  TONE_HI,  1, 1};


// Speaker Task Items
#define SPEAKER_PRIO        11    // WARNING: Hard coded here; not using main.
#define TASK_STACK_SIZE     128

static CPU_STK g_speaker_stack[TASK_STACK_SIZE];
static OS_TCB  g_speaker_tcb;

/*!
*
* @brief Configure a timer to drive the speaker via PWM.
*/
static void
speaker_config (void)
{
  SYSTEM.PRCR.WORD = 0xA50B; /* Protect off */
  
  SYSTEM.MSTPCRA.BIT.MSTPA13 = 0;  // Enable TPU0.
  
  SYSTEM.PRCR.WORD = 0xA500; /* Protect on  */
  
  TPUA.TSTR.BIT.CST0 = 0;     /* Stop TPU 0 */
  
  MPC.PWPR.BIT.B0WI  = 0;  /* En writing to PFS registers              */
  MPC.PWPR.BIT.PFSWE = 1;
  MPC.P17PFS.BYTE  = 0x03; /* 3 defines P17 to be TIOCB0, with no IRQ. */
  MPC.PWPR.BIT.PFSWE = 0;  /* Dis writing to PFS registers             */
  MPC.PWPR.BIT.B0WI  = 1;

  // Configure the hardware in preparation for speaker tones.
  // The TIOCB0 output goes to port 1 bit 7 (the AUDIO_R signal)
  // No need to explicitly set this as an output since the MTU does that
  TPU0.TCR.BIT.TPSC = 0x03;       // Prescale by 64. (p. 854)
  TPU0.TCR.BIT.CCLR = 0x01;       // Select clear counter on TGRA match. (p. 855)
  TPU0.TIORH.BIT.IOA = 0x0;       // TIOCA0 output disabled
  TPU0.TIORH.BIT.IOB = 0x02;        // TIOCB0 pin initial low, high on match. (p. 873)
  TPU0.TIORL.BIT.IOC = 0x04;       // TIOCC0 output disabled
  TPU0.TIORL.BIT.IOD = 0x04;       // TIOCD0 output disabled
  TPU0.TMDR.BIT.MD  = 0x03; // PWM mode 2 - page 859
  TPU0.TGRA = 400;          // frequency of 1000 Hz
  TPU0.TGRB = 375;          // 50% duty cycle

  PORT1.PDR.BIT.B7 = 1;    /* Set P17 as output. */
  PORT1.PMR.BIT.B7 = 1;    /* Set P17 as peripheral function bit */

 // Ensure that the speaker is off.
  TPUA.TSTR.BIT.CST0 = 0;     /* Stop TPU 0 */
}

/*!
* @brief Speaker Task
* @param[in] p_arg Waveform to play.
*/
void speaker_task (void * p_arg)
{
    wave_t *  p_waveform = (wave_t *) p_arg;
    OS_ERR    err;
	

    if (p_waveform->b_chopped)
    {
        // Create a chopped sound.
	    for (;;)
        {
            // Play Tone 1.
	        TPU0.TGRA = p_waveform->tone1;
	        TPUA.TSTR.BIT.CST0 = 1;	// On
	        OSTimeDlyHMSM(0, 0, 0, 125, OS_OPT_TIME_HMSM_STRICT, &err);
	        TPUA.TSTR.BIT.CST0 = 0;	// Off
	        OSTimeDlyHMSM(0, 0, 0, 125, OS_OPT_TIME_HMSM_STRICT, &err);
	        TPUA.TSTR.BIT.CST0 = 1;	// On
	        OSTimeDlyHMSM(0, 0, 0, 125, OS_OPT_TIME_HMSM_STRICT, &err);
	        TPUA.TSTR.BIT.CST0 = 0;	// Off
	        OSTimeDlyHMSM(0, 0, 0, 125, OS_OPT_TIME_HMSM_STRICT, &err);
			
            // Play Tone 2.
	        TPU0.TGRA = p_waveform->tone2;
	        TPUA.TSTR.BIT.CST0 = 1;	// On
	        OSTimeDlyHMSM(0, 0, 0, 125, OS_OPT_TIME_HMSM_STRICT, &err);
	        TPUA.TSTR.BIT.CST0 = 0;	// Off
	        OSTimeDlyHMSM(0, 0, 0, 125, OS_OPT_TIME_HMSM_STRICT, &err);
	        TPUA.TSTR.BIT.CST0 = 1;	// On
	        OSTimeDlyHMSM(0, 0, 0, 125, OS_OPT_TIME_HMSM_STRICT, &err);
	        TPUA.TSTR.BIT.CST0 = 0;	// Off
	        OSTimeDlyHMSM(0, 0, 0, 125, OS_OPT_TIME_HMSM_STRICT, &err);	
	    }
    }
    else
    {
        // No chopping.
	    for (;;)
        {
            // Play Tone 1.
	        TPU0.TGRA = p_waveform->tone1;
	        TPUA.TSTR.BIT.CST0 = 1;	// On
	        OSTimeDlyHMSM(0, 0, p_waveform->interval, 0, OS_OPT_TIME_HMSM_STRICT, &err);
	        TPUA.TSTR.BIT.CST0 = 0;	// Off

            // Play Tone 2.
	        TPU0.TGRA = p_waveform->tone2;
	        TPUA.TSTR.BIT.CST0 = 1;	// On
	        OSTimeDlyHMSM(0, 0, p_waveform->interval, 0, OS_OPT_TIME_HMSM_STRICT, &err);
	        TPUA.TSTR.BIT.CST0 = 0;	// Off
	    }
    }
}


/*!
*
* @brief Alarm Task
*/
void alarm_task (void * p_arg)
{
    wave_t const *  p_waveform = (wave_t *) NULL;
    uint8_t         b_speaker_task_alive = 0;
    uint8_t         b_create_speaker_task;
    OS_ERR		    err;	
    CPU_TS ts;
    OS_FLAGS flagsToWait = 0x0f;
    int flags;


    (void)p_arg;    // NOTE: Silence compiler warning about unused param.
   
    // Configure the speaker hardware.
    speaker_config();
    
    

    for (;;)	
    {
        // TODO: Do nothing until there is a signal from another task.
        flags = OSFlagPend(&g_alarm_flags, 
                           (OS_FLAGS)flagsToWait, 
                           0, 
                           (OS_OPT)OS_OPT_PEND_BLOCKING | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_FLAG_SET_ANY, 
                           &ts, 
                           &err);
		
        // Assume for now there's no new task creation to do.
        b_create_speaker_task = 0;
		
        // Ensure the proper alarm is playing.
        if ((flags & ALARM_HIGH) != 0)
        {
            // High priority alarm should be playing.
            if (p_waveform != &alarm_high)
            {
                // Wrong alarm playing.
		if (b_speaker_task_alive)
                {
                    // Kill the speaker task.
		    OSTaskDel(&g_speaker_tcb, &err);	
                    assert(OS_ERR_NONE == err);
		    b_speaker_task_alive = 0;
		
                    // Ensure the speaker is off.
                    TPUA.TSTR.BIT.CST0 = 0;						
		}

                // Select the correct waveform.
		p_waveform = &alarm_high;
		b_create_speaker_task = 1;
	    }
	}
        else if ((flags & ALARM_MEDIUM) != 0)
        {
            // Medium priority alarm should be playing.
	    if (p_waveform != &alarm_medium)
            {
                // Wrong alarm playing.
	        if (b_speaker_task_alive)
                {
                    // Kill the speaker task.
		    OSTaskDel(&g_speaker_tcb, &err);
                    assert(OS_ERR_NONE == err);
		    b_speaker_task_alive = 0;
		
                    // Ensure the speaker is off.
                    TPUA.TSTR.BIT.CST0 = 0;						
		}

                // Select the correct waveform.
		p_waveform = &alarm_medium;
		b_create_speaker_task = 1;
	    }
	}
        else if ((flags & ALARM_LOW) != 0)
        {
            // Low priority alarm should be playing.
	    if (p_waveform != &alarm_low)
            {
                // Wrong alarm playing.
	        if (b_speaker_task_alive)
                {
                    // Kill the speaker task.
	            OSTaskDel(&g_speaker_tcb, &err);
                    assert(OS_ERR_NONE == err);
	            b_speaker_task_alive = 0;

                    // Ensure the speaker is off.
	            TPUA.TSTR.BIT.CST0 = 0;										
	        }
		
                // Select the correct waveform.
                p_waveform = &alarm_low;
	        b_create_speaker_task = 1;
	    }
	}
        else if ((flags & ALARM_NONE) != 0)
        {
            // No alarm should be playing.
	    if (b_speaker_task_alive)
            {
                // Kill the speaker task.
	        OSTaskDel(&g_speaker_tcb, &err);
                assert(OS_ERR_NONE == err);
	        b_speaker_task_alive = 0;

                // Ensure the speaker is off.
	        TPUA.TSTR.BIT.CST0 = 0;
	    }

            // Select the correct waveform
            p_waveform = NULL;
            b_create_speaker_task = 0;
	}
        else
        {
            // We should never get here.
            assert(0);
        }
		
        // If necessary, create a speaker task to play the new tone.
	if (b_create_speaker_task)
        {
            OSTaskCreate(
                (OS_TCB     *)&g_speaker_tcb,
                (CPU_CHAR   *)"Speaker Task",
                (OS_TASK_PTR ) speaker_task,
                (void       *) p_waveform,
                (OS_PRIO     ) 9,
                (CPU_STK    *)&g_speaker_stack[0],
                (CPU_STK_SIZE) TASK_STACK_SIZE / 10u,
                (CPU_STK_SIZE) TASK_STACK_SIZE,
                (OS_MSG_QTY  ) 0u,
                (OS_TICK     ) 0u,
                (void       *) 0,
                (OS_OPT      ) 0,
                (OS_ERR     *)&err);
	        assert(OS_ERR_NONE == err);

	        b_speaker_task_alive = 1;
	    }
    }
}
