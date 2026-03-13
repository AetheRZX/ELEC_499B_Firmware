/* ==============================================================================
System Name:  	BLDC_Int

File Name:		BLDC_Int.h

Description:	Primary system header file for the Real Implementation of Sensorless
          		Trapezoidal Control of Brushless DC Motors (BLDC) using BEMF Integration

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DRV8312-EVM. 
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 06-08-2011	Version 1.0
=================================================================================  */

/*-------------------------------------------------------------------------------
Next, Include project specific include files.
-------------------------------------------------------------------------------*/

#include "park.h"               // Include header for the PARK object
#include "clarke.h"             // Include header for the CLARKE object
#include "id_mean.h"
#include "pid_control.h"       	// Include header for the PID_GRANDO_CONTROLLER object
#include "pid_control_speed.h"  // Include header for the PID_GRANDO_CONTROLLER speed object
#include "rmp3cntl.h"       	// Include header for the RMP3 object
#include "rmp2cntl.h"           // Include header for the RMP2 object
#include "impulse.h"       		// Include header for the IMPULSE object
#include "mod6_cnt_dir.h"      	// Include header for the MOD6CNTDIR object
#include "Commutation_detection.h"
#include "Commutation_detection_2.h"
#include "CommutationDuty.h"
#include "CommutationDuty_Shi.h"
#include "pwm_gen.h"
#include "pwm_gen_Shi.h"
#include "start_current.h"
#include "Commutation_extended.h"
#include "Three_Phase_conduct.h"
#include "Current_reconstruct2.h"
#include "InstaSPIN_BLDC_Lib.h"	// Include header for the InstaSPIN library
#include "speed_pr.h"           // Include header for the SPEED_MEAS_REV object
#include "speed_pr_1.h"         // Include header for the SPEED_ES object
#include "speed_est_1.h"        // Include header for the SPEED_ES object
#include "phiv_filter.h"        // Include header for filtering of firing angle
#include "FiringAngle.h"        // Include header for adding of firing angle
#include "Torque_cal.h"        // Include header for adding of torque calculation
#include "Torque_compare.h"        // Include header for adding of torque compare
#include "DutyRatioCal.h"
#include "DutyRatioCal2.h"
#include "BackEMF_cal.h"        // Include header for adding of backemf calculation
#include "rmp_cntl.h"       	// Include header for the RMPCNTL object*/
#include "rampgen.h"            // Include header for the RAMPGEN object
#include "f2803xileg_vdc.h"	    // Include header for the ILEG2DCBUSMEAS object
#if defined(DRV8312)
#include "f2803xpwm_cntl.h"         // Include header for the PWMGEN object
#include "f2803xpwmdac_BLDC.h"    	// Include header for the PWMGEN object
#endif
#if defined(DRV8301) || defined(DRV8302)
#include "f2803xpwmdac_BLDC_8301.h"  // Include header for the PWMGEN object
#include "f2803xpwm_cntl_8301.h"     // Include header for the PWMGEN object
#endif
#include "DSP2803x_EPwm_defines.h"	// Include header for PWM defines
#include "f2803xhall_gpio_BLDC.h"   // Include header for the HALL object

#include "dlog4ch-BLDC_Int.h" // Include header for the DLOG_4CH object

//SPI only used for DRV8301
#ifdef DRV8301
#include "DRV8301_SPI.h"
#endif

//===========================================================================
// No more.
//===========================================================================
