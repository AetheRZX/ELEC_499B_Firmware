/* =================================================================================
File name:  BLDC_Int-Settings.h                     
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Incremental Build Level control file.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 06-08-2011	Version 1.0
=================================================================================  */
#ifndef PROJ_SETTINGS_H

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/
#define LEVEL1  1      		// Module check out (do not connect the motors) 1
#define LEVEL2  2           // Verify ADC, run the motor open loop
#define LEVEL3	3			// Auto-calibrate the current sensor and BEMF sense offsets
#define LEVEL4  4           // Verify BEMFINT module
#define LEVEL5  5           // Verify commutation based on the BEMF Integration
#define LEVEL6  6           // Verify the closed current loop and current PI controller
#define LEVEL7  7           // Verify the closed speed loop and speed PI controller
#define LEVEL8  8           // Verify the closed speed loop and current loop
#define LEVEL9  9           // Verify the closed speed loop and current loop
#define LEVEL10  10           // Verify the closed speed loop and current loop
#define LEVEL11  11          // Verify the closed speed loop and current loop

/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/
#define   BUILDLEVEL 11


#ifndef BUILDLEVEL    
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL
//------------------------------------------------------------------------------
// The following are the motor types used in this project

#define MOTOR_A 1 // Large misalignment in Hall Sensors
#define MOTOR_D 2
#define MOTOR_SELECT MOTOR_A

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PI 3.14159265358979

// Define the system frequency (MHz)
#if (DSP2803x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 60
#elif (DSP280x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 100
#elif (F2806x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 80
#endif

//Define system Math Type
// Select Floating Math Type for 2806x
// Select IQ Math Type for 2803x 
#if (DSP2803x_DEVICE_H==1)
#define MATH_TYPE 0 
#elif (F2806x_DEVICE_H==1)
#define MATH_TYPE 1
#endif

// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 20
#define PWM_FREQUENCY 20
//#define ISR_FREQUENCY 18
//#define PWM_FREQUENCY 18

//cutoff freq and time constant of the offset calibration LPF
#define WC_CAL	100
#define TC_CAL	1/WC_CAL

// Parameter Initializations 
#define ALIGN_DUTY   0x0500
#define LOOP_CNT_MAX 0

// This machine parameters are based on 24V PM motors inside Multi-Axis +PFC package
// Define the PMSM motor parameters
#define RS 		0.27/*0.15*/               		// Stator resistance (ohm)
#define RR   	0               		// Rotor resistance (ohm) 
#define LS   	0.0012/*0.00045*/   				// Stator inductance (H)
#define LR   	0						// Rotor inductance (H) 	
#define LM   	0						// Magnetizing inductance (H)
#define POLES   8						// Number of poles

// Define the base quantites 
#define BASE_VOLTAGE    66.32		    // Base peak phase voltage (volt), maximum measurable DC Bus 
#if defined(DRV8312)
#define BASE_CURRENT    60            	// Base peak phase current (amp) , maximum measurable peak current
#endif
#if defined(DRV8301) || defined(DRV8302)
//options for BASE_CURRENT based on DRV830x current-sense amplifier gain setting
//NOTE: DRV8302 can only be set to gain of 10 or 40
//#define DRV_GAIN	10
#define DRV_GAIN	20
//#define DRV_GAIN	40
//#define DRV_GAIN	80

#if DRV_GAIN == 10
#define BASE_CURRENT    82.5           	// Base peak phase current (amp) , maximum measurable peak current (with DRV830x gain set to 10)
#elif DRV_GAIN == 20
#define BASE_CURRENT    41.25          	// Base peak phase current (amp) , maximum measurable peak current (with DRV830x gain set to 20)
#elif DRV_GAIN == 40
#define BASE_CURRENT    20.625         	// Base peak phase current (amp) , maximum measurable peak current (with DRV830x gain set to 40)
#elif DRV_GAIN == 80
#define BASE_CURRENT    10.3125        	// Base peak phase current (amp) , maximum measurable peak current (with DRV830x gain set to 80)
#else
#error  Invalid GAIN selection !!
#endif
#endif

#define BASE_FREQ      	200           	// Base electrical frequency (Hz)

//define motor start up parameters
#define BEGIN_START_RPM 50.0
#define END_START_RPM 200.0
//#define END_START_RPM 600.0

//define starting and ending parameters for ramp generator
#define COMMUTATES_PER_E_REV 6.0
#define RAMP_START_RATE	(PWM_FREQUENCY*1000)*60.0/BEGIN_START_RPM/COMMUTATES_PER_E_REV/(POLES/2.0)
#define RAMP_END_RATE (PWM_FREQUENCY*1000)*60.0/END_START_RPM/COMMUTATES_PER_E_REV/(POLES/2.0)


#endif 
