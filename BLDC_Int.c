/* ==============================================================================
System Name:  	BLDC_Int

File Name:	  	BLDC_Int.c

Description:	Primary system file for the Real Implementation of Sensorless
          		Trapeziodal Control of Brushless DC Motors (BLDC) using BEMF Integration

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DRV8312-EVM. 
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 06-08-2011	Version 1.0:  F2806x or F2803x target
===================================================================================*/

// Include header files used in the main function

#include "PeripheralHeaderIncludes.h"
#include "BLDC_Int-Settings.h"
#include "IQmathLib.h"
#include "BLDC_Int.h"
#include <math.h>

#ifdef DRV8301
union DRV8301_STATUS_REG_1 DRV8301_stat_reg1;
union DRV8301_STATUS_REG_2 DRV8301_stat_reg2;
union DRV8301_CONTROL_REG_1 DRV8301_cntrl_reg1;
union DRV8301_CONTROL_REG_2 DRV8301_cntrl_reg2;
Uint16 read_drv_status = 0;
#endif

#ifdef FLASH
#pragma CODE_SECTION(MainISR,"ramfuncs");
void MemCopy();
void InitFlash();
#endif

// Prototype statements for functions found within this file.
interrupt void MainISR(void);
void DeviceInit();

// State Machine function prototypes
//------------------------------------
// Alpha states
void A0(void);	//state A0
void B0(void);	//state B0
//void C0(void);	//state C0 # unused

// A branch states
void A1(void);	//state A1
//void A2(void);	//state A2 # unused
//void A3(void);	//state A3 # unused

// B branch states
void B1(void);	//state B1
//void B2(void);	//state B2 # unused
//void B3(void);	//state B3 # unused

// C branch states # All unused
//void C1(void);	//state C1
//void C2(void);	//state C2
//void C3(void);	//state C3

// Variable declarations
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
//void (*C_Task_Ptr)(void);		// State pointer C branch


int16	VTimer0[4];			// Virtual Timers slaved off CPU Timer 0 (A events)
int16	VTimer1[4]; 		// Virtual Timers slaved off CPU Timer 1 (B events)
int16	VTimer2[4]; 		// Virtual Timers slaved off CPU Timer 2 (C events)
int16	SerialCommsTimer;
int16  DFuncDesired = (int16)_IQtoQ15(_IQ(1));

// Global variables used in this system

// Mark's stuff
#define DEG_0 _IQ(0)
//#define DEG_30 _IQ(0.083333333333333)
//#define DEG_90 _IQ(0.25)
#define DEG_120 _IQ(0.33333333333333)
//#define DEG_150 _IQ(0.416666666666667)
//#define DEG_210 _IQ(0.583333333333333)
//#define DEG_270 _IQ(0.75)
//#define DEG_330 _IQ(0.916666666666667)
#define DEG_360 _IQ(1)

// Motors Hall-Elec Angle Mapping
#if (MOTOR_SELECT == MOTOR_A)
#define SEC3_NEXTANGLE  _IQ(0) // DEG_0
#define SEC1_NEXTANGLE  _IQ(0.1666667) // DEG_60
#define SEC5_NEXTANGLE  _IQ(0.33333333333333) // DEG_120
#define SEC4_NEXTANGLE  _IQ(0.5) // DEG_180
#define SEC6_NEXTANGLE  _IQ(0.6666667) // DEG_240
#define SEC2_NEXTANGLE  _IQ(0.8333333) // DEG_300
#endif

#if (MOTOR_SELECT == MOTOR_C)
#define SEC6_NEXTANGLE  _IQ(0)         // DEG_0
#define SEC4_NEXTANGLE  _IQ(0.1666667) // DEG_60
#define SEC5_NEXTANGLE  _IQ(0.3333333) // DEG_120
#define SEC1_NEXTANGLE  _IQ(0.5)       // DEG_180
#define SEC3_NEXTANGLE  _IQ(0.6666667) // DEG_240
#define SEC2_NEXTANGLE  _IQ(0.8333333) // DEG_300
#endif

float ids_float = 0;
_iq iqIaIn = 0;
_iq iqIbIn = 0;
_iq iqIcIn = 0;
_iq ids_avg = 0;

//MTPA
Uint16 MTPAFlag = FALSE;
_iq delta_phiv = _IQ(0);
_iq delta_phiv_filtered = _IQ(0);
_iq delta_phiv_storage[6] = {_IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0)};
Uint16 ids_avg_cal_enable=FALSE;
//_iq speed3_angle_adjust = 0;
//

// LUT MTPA User Toggles
Uint16 timing_mode = 1; // Set to 0 to enable LUT mode, 1 to use old online filter
int16 LutMtpaDir = 1;   // Set to 1 for positive direction LUT, -1 for negative direction LUT

#define HALL_STATE_5 5
#define HALL_STATE_4 4
#define HALL_STATE_6 6
#define HALL_STATE_2 2
#define HALL_STATE_3 3
#define HALL_STATE_1 1
#define HALL_STATE_0 0
#define HALL_STATE_7 7
#define HALL_S16_60_PHASE_SHIFT 10923


// LUTs defined from Excel
// static const int16 LUTB_hall_state_elec_duration_digit[8] = {
//     [HALL_STATE_5] = 13158,
//     [HALL_STATE_4] = 10387,
//     [HALL_STATE_6] = 9714,
//     [HALL_STATE_2] = 12336,
//     [HALL_STATE_3] = 10743,
//     [HALL_STATE_1] = 9199,
//     [HALL_STATE_0] = HALL_S16_60_PHASE_SHIFT,
//     [HALL_STATE_7] = HALL_S16_60_PHASE_SHIFT, // safe defaults for invalid states
// };

// Ryan's LUT
// static const int16 LUTB_hall_state_elec_duration_digit[8] = {
//     [HALL_STATE_5] = 12518,
//     [HALL_STATE_4] = 10146,
//     [HALL_STATE_6] = 10424,
//     [HALL_STATE_2] = 11889,
//     [HALL_STATE_3] = 10667,
//     [HALL_STATE_1] = 9892,
//     [HALL_STATE_0] = HALL_S16_60_PHASE_SHIFT,
//     [HALL_STATE_7] = HALL_S16_60_PHASE_SHIFT, // safe defaults for invalid states
// };

// Measured LUT MOTOR C
static const int16 LUTB_hall_state_elec_duration_digit[8] = {
    [HALL_STATE_5] = 10427,
    [HALL_STATE_1] = 12842,
    [HALL_STATE_3] = 8398,
    [HALL_STATE_2] = 12678,
    [HALL_STATE_6] = 11069,
    [HALL_STATE_4] = 10121,
    [HALL_STATE_0] = HALL_S16_60_PHASE_SHIFT,
    [HALL_STATE_7] = HALL_S16_60_PHASE_SHIFT, // safe defaults for invalid states
};

//// Positive Direction
// static const int16 LUTB_corr_angle[8] = {
//     [HALL_STATE_5] = 11304,
//     [HALL_STATE_4] = 11484,
//     [HALL_STATE_6] = 10071,
//     [HALL_STATE_2] = 11280,
//     [HALL_STATE_3] = 11816,
//     [HALL_STATE_1] = 9581,
//     [HALL_STATE_0] = HALL_S16_60_PHASE_SHIFT,
//     [HALL_STATE_7] = HALL_S16_60_PHASE_SHIFT, // safe defaults for invalid states
// };

//// Negative Direction
// static const int16 LUTB_corr_angle[8] = {
//     [HALL_STATE_5] = 12265,
//     [HALL_STATE_4] = 10029,
//     [HALL_STATE_6] = 10566,
//     [HALL_STATE_2] = 11775,
//     [HALL_STATE_3] = 10361,
//     [HALL_STATE_1] = 10541,
//     [HALL_STATE_0] = HALL_S16_60_PHASE_SHIFT,
//     [HALL_STATE_7] = HALL_S16_60_PHASE_SHIFT, // safe defaults for invalid states
// };

// static const int16 LUTB_corr_angle[8] = {
//     [HALL_STATE_5] = 11764,
//     [HALL_STATE_4] = 10169,
//     [HALL_STATE_6] = 10946,
//     [HALL_STATE_2] = 11445,
//     [HALL_STATE_3] = 10478,
//     [HALL_STATE_1] = 10734,
//     [HALL_STATE_0] = HALL_S16_60_PHASE_SHIFT,
//     [HALL_STATE_7] = HALL_S16_60_PHASE_SHIFT, // safe defaults for invalid states
// };

// Measured phi correction
static const int16 LUTB_corr_angle[8] = {
    [HALL_STATE_5] = 11137,
    [HALL_STATE_1] = 11632,
    [HALL_STATE_3] = 9713,
    [HALL_STATE_2] = 12237,
    [HALL_STATE_6] = 10482,
    [HALL_STATE_4] = 10335,
    [HALL_STATE_0] = HALL_S16_60_PHASE_SHIFT,
    [HALL_STATE_7] = HALL_S16_60_PHASE_SHIFT, // safe defaults for invalid states
};

// static const int16 LUTB_corr_angle[8] = {
//     [HALL_STATE_5] = 10213,
//     [HALL_STATE_1] = 12133,
//     [HALL_STATE_3] = 9608,
//     [HALL_STATE_2] = 11364,
//     [HALL_STATE_6] = 11510,
//     [HALL_STATE_4] = 10708,
//     [HALL_STATE_0] = HALL_S16_60_PHASE_SHIFT,
//     [HALL_STATE_7] = HALL_S16_60_PHASE_SHIFT, // safe defaults for invalid states
// };

float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h 

Uint32 IsrTicker = 0;
Uint16 BackTicker = 0;
Uint16 ClosedFlag = FALSE;
Uint16 PWMmode = 1;

Uint32 VirtualTimer = 0;
Uint32 MyTimer = 0;
Uint32 TimeStamp1 = 0;
Uint32 TimeStamp2 = 0;
Uint32 Period = 0;
_iq  d_pwm = _IQ(0.5);
_iq  d_pwm1 = _IQ(0.0);
_iq  d_pwm2 = _IQ(0.0);
_iq  delta = _IQ(0.0);
_iq  delta2 = _IQ(0.0);
Uint16 commutation_2 = 0;
_iq theta=_IQ(0);
_iq theta_1 = _IQ(0);
_iq D_com=_IQ(0.3333333333);
_iq D_com1=_IQ(0.3333333333);
_iq theta_ref1 = _IQ(0);
_iq theta_ref2 = _IQ(0);
_iq theta_ref3 = _IQ(0);
_iq phid = _IQ(-0.16);
_iq phid1 = _IQ(-0.16);
_iq timer = _IQ(0);
_iq realspeed = _IQ(0);
_iq realia = _IQ(0);
_iq DCbus_current = 0;
_iq Te2 = 0;
_iq cal_offset_A = _IQ15(0.5059);      //0.4990: F28035
_iq cal_offset_B = _IQ15(0.5039);      //0.5034: F28035
_iq cal_offset_C = _IQ15(0.5069);
_iq ad = _IQ(0);
_iq Out = _IQ(0);
_iq nextspeed_LUTB;


int16 PwmDacCh1 = 0;
int16 PwmDacCh2 = 0;
int16 PwmDacCh3 = 0;
#if defined(DRV8301) || defined(DRV8302)
int16 PwmDacCh4 = 0;
#endif

int16 DlogCh1 = 0;
int16 DlogCh2 = 0;
int16 DlogCh3 = 0;
int16 DlogCh4 = 0;

int16 Observation_data_i = 0;
int16 Observation_data_a = 0;
int16 Observation_data_ia[600] = {0};
int16 Observation_data_te[600] = {0};
Uint16 k = FALSE;
Uint16 commutation = 0;

//#if (BUILDLEVEL==LEVEL1)
//Uint16 DRV_RESET = 1;
//#else
Uint16 DRV_RESET = 0;
//#endif

volatile Uint16 EnableFlag = FALSE;
Uint16 RunMotor = FALSE;

_iq BemfA_offset = _IQ15(0.0070);	//modify offset after calibration step
_iq BemfB_offset = _IQ15(0.0072);	//modify offset after calibration step
_iq BemfC_offset = _IQ15(0.0063);	//modify offset after calibration step
_iq IDC_offset = _IQ15(0.5041);	//modify offset after calibration step
_iq cal_filt_gain;
// Instance a backemf calculation
BACKEMF backemf1 = BACKEMF_DEFAULTS;

//Instance a Commutation Detection
COMMUTATIONDETEC commutation1 = COMMUTATIONDETEC_DEFAULTS;

//Instance a Commutation Detection2
COMMUTATIONDETEC2 commutation3 = COMMUTATIONDETEC_DEFAULTS2;

//Instance a Commutation Duty Calculation
COMMUTATIONDUTY commutation2 = COMMUTATIONDUTY_DEFAULTS;

//Instance a Commutation Duty Shi Calculation
COMMUTATIONDUTYSHI commutation4 = COMMUTATIONDUTYSHI_DEFAULTS;

//Instance a Commutation Extended Calculation
COMMUTATIONEXT commutationext = COMMUTATIONEXT_DEFAULTS;

// Instance a few transform objects
ID_MEAN idmean1 = {MEAN_TR_DEFAULTS,MEAN_DATA_DEFAULTS};

// Instance PID regulator to force id_avg = 0 for MTPA
PI_CONTROLLER pi1_id = {PI_TERM_DEFAULTS,PI_PARAM_DEFAULTS,PI_DATA_DEFAULTS};

// Instance PID regulator to regulate the DC-bus current and speed
PI_CONTROLLER1 pi_speed = {PI_TERM_SPEED_DEFAULTS,PI_PARAM_SPEED_DEFAULTS,PI_DATA_SPEED_DEFAULTS};

// Instance a PWM driver instance
PWM_CNTL pwmcntl1 = PWM_CNTL_DEFAULTS;

// Instance a PWM DAC driver instance
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;

// Instance a Hall effect driver
HALL3  hall1 = HALL3_DEFAULTS;

// Instance a RAMP2 Module
RMP2 rmp2 = RMP2_DEFAULTS;
// Instance a Torque_cal Module
TORQUECAL   torque = TORQUECAL_DEFAULTS;

// Instance a current reconstruct Module
CURRENTRECONSTRUCT currentrecons = CURRENTRECONSTRUCT_DEFAULTS;

// Instance a Three Phase Module
THREEPHASE ThreePhase = THREEPHASE_DEFAULTS;

// Instance a Duty Calculation Module
DutyCal  Dutycal = DutyCal_DEFAULTS;

// Instance a Duty Calculation 2 Module
DutyCal2  Dutycal2 = DutyCal2_DEFAULTS;

// Instance a COMTRIG Module
//CMTN cmtn1 = CMTN_DEFAULTS;

// Instance a PWM GEN Module
PWMgen PWM1 = PWMgen_DEFAULTS;

// Instance a PWM GEN Shi Module
//PWMgen_Shi PWM2 = PWMgenShi_DEFAULTS;
PWMgen PWM2 = PWMgen_DEFAULTS;


// Instance a Start Current Module
STARTCURRENT startcurrent =  STARTCURRENT_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
//  Instance a ramp generator to simulate an Angle
RAMPGEN rg1 = RAMPGEN_DEFAULTS;
//  Instance a ramp generator to simulate an Angle
RAMPGEN rg2 = RAMPGEN_DEFAULTS;
// For filtering of firing angle
PHIV_FILTER phivf=PHIV_FILTER_DEFAULTS;
// For filtering of firing angle
PHIV_FILTER phivf2=PHIV_FILTER_DEFAULTS;
// For adding of firing angle
FIREANGLE fa1=FIREANGLE_DEFAULTS;

// Instance a SPEED_PR Modul
// SPEED_MEAS_CAP speed1 = SPEED_MEAS_CAP_DEFAULTS;
// Instance a SPEED_PR Module
SPEED_MEAS_CAP1 speed2 = SPEED_MEAS_CAP_DEFAULTS1;
SPEED_ESTIMATION1 speed3 = SPEED_ESTIMATION_DEFAULTS1;


// Create an instance of DATALOG Module
DLOG_4CH dlog = DLOG_4CH_DEFAULTS; 



   void main(void)

   {

	DeviceInit();	// Device Life support & GPIO

// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler

#ifdef FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files. 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)


// Timing sync for background loops 
// Timer period definitions found in device specific PeripheralHeaderIncludes.h
	CpuTimer1Regs.PRD.all =  mSec1;		// A tasks
	CpuTimer2Regs.PRD.all =  mSec5;		// B tasks

// Tasks State-machine init
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;

   // Waiting for enable flag set
   while (EnableFlag==FALSE) 
    { 
      BackTicker++;
    }

// Initialize all the Device Peripherals:
// This function is found in DSP280x_CpuTimers.c
   InitCpuTimers();
// Configure CPU-Timer 0 to interrupt every ISR Period:
// 60MHz CPU Freq, ISR Period (in uSeconds)
// This function is found in DSP280x_CpuTimers.c
   ConfigCpuTimer(&CpuTimer0, SYSTEM_FREQUENCY, 1000/ISR_FREQUENCY);
   StartCpuTimer0();

#ifdef DRV8301
// Initialize SPI for communication to the DRV8301
	DRV8301_SPI_Init(&SpibRegs);
#endif

// Reassign ISRs. 
// Reassign the PIE vector for TINT0 to point to a different 
// ISR then the shell routine found in DSP280x_DefaultIsr.c.
// This is done if the user does not want to use the shell ISR routine
// but instead wants to use their own ISR.

	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.ADCINT1 = &MainISR;
	//PieVectTable.EPWM1_INT = &MainISR;
	EDIS;   // This is needed to disable write to EALLOW protected registers

	// Enable PIE group 3 interrupt 1 for EPWM1_INT
	    //PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

// Enable PIE group 1 interrupt 1 for ADCINT1
    PieCtrlRegs.PIEIER1.all = M_INT1;

	    // Enable CNT_zero interrupt using EPWM1 Time-base
	        //EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation
	        //EPwm1Regs.ETSEL.bit.INTSEL = 2;  // Enable interrupt CNT_zero event
	        //EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
	        //EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts

// Enable CPU INT1 for ADCINT1:
	 IER |= M_INT1;
	        //IER |= M_INT3;
	// Enable Global realtime interrupt DBGM

// Initialize the PWM control module
	pwmcntl1.PWMprd = SYSTEM_FREQUENCY*1000000*T/2;;				// Set the pwm period for this example
    PWM_CNTL_INIT_MACRO(pwmcntl1)
    PWM_CNTL_MACRO(pwmcntl1)

// Initialize PWMDAC module
	pwmdac1.PeriodMax = 500;   // 3000->10kHz, 1500->20kHz, 1000-> 30kHz, 500->60kHz
    pwmdac1.PwmDacInPointer0 = &PwmDacCh1;
    pwmdac1.PwmDacInPointer1 = &PwmDacCh2;
    pwmdac1.PwmDacInPointer2 = &PwmDacCh3;
#if defined(DRV8301) || defined(DRV8302)
    pwmdac1.PwmDacInPointer3 = &PwmDacCh4;
#endif
	PWMDAC_INIT_MACRO(pwmdac1)


// Initialize Hall module
    hall1.DebounceAmount = 1;
    hall1.Revolutions = -1;
    hall1.HallMap[0] = 5;
    hall1.HallMap[1] = 4;
    hall1.HallMap[2] = 6;
    hall1.HallMap[3] = 2;
    hall1.HallMap[4] = 3;
    hall1.HallMap[5] = 1;
    HALL3_INIT_MACRO(hall1)



// Initialize DATALOG module
    dlog.iptr1 = &DlogCh1;
    dlog.iptr2 = &DlogCh2;
    dlog.iptr3 = &DlogCh3;
    dlog.iptr4 = &DlogCh4;
    dlog.trig_value = 0x1;
    dlog.size = 0x0C8;//0x0C8;//0x190;//0x12C;
#if (BUILDLEVEL < 5)
    dlog.prescalar =25; //25;
#else
    dlog.prescalar = 1;
#endif
    dlog.init(&dlog);

// Initialize ADC module (F2803XIDC_VEMF.H)
	 ADC_MACRO()

// Initialize the SPEED_PR module
    // speed1.InputSelect = 0;
    speed3.BaseRpm = 120*(BASE_FREQ/POLES);         // Base RPM = 4999
    // speed1.SpeedScaler = (Uint32)((ISR_FREQUENCY)/(BASE_FREQ*0.001));   // divider of 10
    speed2.InputSelect = 0;
	speed3.speedscaler = (Uint32)((ISR_FREQUENCY)/(BASE_FREQ*0.001));


// Initialize the RAMPGEN module
 	rg1.StepAngleMax = _IQ(BASE_FREQ*T);
 	rg2.StepAngleMax = _IQ(BASE_FREQ*T);

// Initialize RMPCNTL module
    rc1.RampDelayMax = 1;
    rc1.RampLowLimit = _IQ(-1.0);
    rc1.RampHighLimit = _IQ(1.0);


// Initialize the PID controller for MTPA
//  pi1_id.param.Kp = _IQ(0.0022*BASE_CURRENT);
//	pi1_id.param.Ki = _IQ(0.2368*BASE_CURRENT*T);

    pi1_id.param.Kp = _IQ(0.4);
    pi1_id.param.Ki = _IQ(0.18);

// Initialize the PID_GRANDO_CONTROLLER module for speed
	pi_speed.param.Kp = _IQ(0.00009);
	pi_speed.param.Ki = _IQ(0.0000);
	pi_speed.param.Ba = _IQ(0.009);


// Initialize CMTN module
	//cmtn1.NWDelayThres = 20;
//	cmtn1.NWDelta = 2;
	//cmtn1.NoiseWindowMax = cmtn1.NWDelayThres - cmtn1.NWDelta;

    
// Initialize the current offset calibration filter
    cal_filt_gain = _IQ(T/(T+TC_CAL));

// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM


// IDLE loop. Just sit and loop forever:
	for(;;)  //infinite loop
	{
		BackTicker++;

		// State machine entry & exit point
		//===========================================================
		(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
		//===========================================================

#ifdef DRV8301
		//read the status registers from the DRV8301
		{
			DRV8301_stat_reg1.all = DRV8301_SPI_Read(&SpibRegs,STAT_REG_1_ADDR);
			DRV8301_stat_reg2.all = DRV8301_SPI_Read(&SpibRegs,STAT_REG_2_ADDR);
			read_drv_status = 0;
		}
#endif
	}
} //END MAIN CODE



//=================================================================================
//	STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
void A0(void)
{
	// loop rate synchronizer for A-tasks
	if(CpuTimer0Regs.TCR.bit.TIF == 1)
	{
		CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag

		//-----------------------------------------------------------
		(*A_Task_Ptr)();		// jump to an A Task (A1,A2,A3,...)
		//-----------------------------------------------------------

		VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
		SerialCommsTimer++;
	}

	Alpha_State_Ptr = &B0;		// Comment out to allow only A tasks
}

void B0(void)
{
	// loop rate synchronizer for B-tasks
	if(CpuTimer1Regs.TCR.bit.TIF == 1)
	{
		CpuTimer1Regs.TCR.bit.TIF = 1;				// clear flag

		//-----------------------------------------------------------
		(*B_Task_Ptr)();		// jump to a B Task (B1,B2,B3,...)
		//-----------------------------------------------------------
		VTimer1[0]++;			// virtual timer 1, instance 0 (spare)
	}

	Alpha_State_Ptr = &A0;		// Allow C state tasks
}

//=================================================================================
//	A - TASKS (executed in every 1 msec)
//=================================================================================
//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
	if (EnableFlag == FALSE)
	{

#if defined(DRV8301) || defined(DRV8302)
		//de-assert the DRV830x EN_GATE pin
		GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
#endif

		RunMotor = FALSE;

		EALLOW;
	 	EPwm1Regs.TZFRC.bit.OST=1;
		EPwm2Regs.TZFRC.bit.OST=1;
		EPwm3Regs.TZFRC.bit.OST=1;
	 	EDIS;
	}
	else if((EnableFlag == TRUE) && (RunMotor == FALSE))
	{
#ifdef DRV8302
#if DRV_GAIN == 10
		GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;	// GAIN = 10
#elif DRV_GAIN == 40
		GpioDataRegs.GPASET.bit.GPIO25 = 1;		// GAIN = 40
#else
#error  Invalid GAIN setting for DRV8302!!
#endif
		//GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;	// M_OC - cycle by cycle current limit
		GpioDataRegs.GPASET.bit.GPIO24 = 1;		// M_OC - fault on OC
#endif
		//if we want the power stage active we need to enable the DRV830x
		//and configure it.
		if(DRV_RESET == 0)
		{
#if defined(DRV8301) || defined(DRV8302)
			//assert the DRV830x EN_GATE pin
			GpioDataRegs.GPBSET.bit.GPIO39 = 1;

			DELAY_US(50000);		//delay to allow DRV830x supplies to ramp up

#ifdef DRV8301
			DRV8301_cntrl_reg1.bit.GATE_CURRENT = 0;		// full current 1.7A
//			DRV8301_cntrl_reg1.bit.GATE_CURRENT = 1;		// med current 0.7A
//			DRV8301_cntrl_reg1.bit.GATE_CURRENT = 2;		// min current 0.25A
			DRV8301_cntrl_reg1.bit.GATE_RESET = 0;			// Normal Mode
			DRV8301_cntrl_reg1.bit.PWM_MODE = 2;			// six independant PWMs
//			DRV8301_cntrl_reg1.bit.OC_MODE = 0;				// current limiting when OC detected
			DRV8301_cntrl_reg1.bit.OC_MODE = 1;				// latched OC shutdown
//			DRV8301_cntrl_reg1.bit.OC_MODE = 2;				// Report on OCTWn pin and SPI reg only, no shut-down
//			DRV8301_cntrl_reg1.bit.OC_MODE = 3;				// OC protection disabled
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 0;			// OC @ Vds=0.060V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 4;			// OC @ Vds=0.097V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 6;			// OC @ Vds=0.123V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 9;			// OC @ Vds=0.175V
			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 15;			// OC @ Vds=0.358V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 16;			// OC @ Vds=0.403V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 17;			// OC @ Vds=0.454V
//			DRV8301_cntrl_reg1.bit.OC_ADJ_SET = 18;			// OC @ Vds=0.511V
			DRV8301_cntrl_reg1.bit.Reserved = 0;

//			DRV8301_cntrl_reg2.bit.OCTW_SET = 0;			// report OT and OC
			DRV8301_cntrl_reg2.bit.OCTW_SET = 1;			// report OT only
#if DRV_GAIN == 10
			DRV8301_cntrl_reg2.bit.GAIN = 0;				// CS amplifier gain = 10
#elif DRV_GAIN == 20
			DRV8301_cntrl_reg2.bit.GAIN = 1;				// CS amplifier gain = 20
#elif DRV_GAIN == 40
			DRV8301_cntrl_reg2.bit.GAIN = 2;				// CS amplifier gain = 40
#elif DRV_GAIN == 80
			DRV8301_cntrl_reg2.bit.GAIN = 3;				// CS amplifier gain = 80
#endif
			DRV8301_cntrl_reg2.bit.DC_CAL_CH1 = 0;			// not in CS calibrate mode
			DRV8301_cntrl_reg2.bit.DC_CAL_CH2 = 0;			// not in CS calibrate mode
			DRV8301_cntrl_reg2.bit.OC_TOFF = 0;				// normal mode
			DRV8301_cntrl_reg2.bit.Reserved = 0;

			//write to DRV8301 control register 1, returns status register 1
			DRV8301_stat_reg1.all = DRV8301_SPI_Write(&SpibRegs,CNTRL_REG_1_ADDR,DRV8301_cntrl_reg1.all);
			//write to DRV8301 control register 2, returns status register 1
			DRV8301_stat_reg1.all = DRV8301_SPI_Write(&SpibRegs,CNTRL_REG_2_ADDR,DRV8301_cntrl_reg2.all);
#endif
#endif
		}

	    hall1.DebounceAmount = 1;
	    hall1.Revolutions = -1;
	    hall1.HallMap[0] = 5;
	    hall1.HallMap[1] = 4;
	    hall1.HallMap[2] = 6;
	    hall1.HallMap[3] = 2;
	    hall1.HallMap[4] = 3;
	    hall1.HallMap[5] = 1;
	    HALL3_INIT_MACRO(hall1)

		/*speed1.InputSelect = 0;
		speed1.NewTimeStamp = 0;
		speed1.OldTimeStamp = 0;
		speed1.EventPeriod = 0;
		speed1.Speed = 0;*/
		VirtualTimer = 0;

		speed2.InputSelect = 0;
		speed2.NewTimeStamp = 0;
		speed2.OldTimeStamp = 0;
		speed2.TimeStamp = 0;
		speed2.EventPeriod_n = 0;
		speed2.EventPeriod_n_1 = 0;
		speed2.EventPeriod_n_2 = 0;
		speed2.EventPeriod_n_3 = 0;
		speed2.EventPeriod_n_4 = 0;
		speed2.EventPeriod_n_5 = 0;
		speed2.EventPeriod_n_6 = 0;

        speed3.nexttheta = _IQ(0);
        speed3.currenttheta = _IQ(0);
        speed3.dtheta = _IQ(0);
        speed3.nextspeed = _IQ(0);
        speed3.HallGpio = 0;
        speed3.EventPeriod = 0;
        speed3.SUflag=TRUE;

		rc1.EqualFlag = 0;
		rc1.RampDelayCount = 0;
		rc1.TargetValue = 0;

		RESET_PI_CTRL(pi1_id)
//		pi1_id.data.e1 = 0;
//		pi1_id.data.ui = 0;
//		pi1_id.data.up = 0;
//		pi1_id.data.v1 = 0;
//		pi1_id.data.e  = 0;
//		pi1_id.term.Out = 0;
//		pi1_id.param.Umax=_IQ(0.05);
//		pi1_id.param.Umin=_IQ(-0.05);

        pi_speed.data.e1 = 0;
        pi_speed.data.ui = 0;
        pi_speed.data.i1 = 0;
        pi_speed.data.up = 0;
        pi_speed.data.v1 = 0;
        pi_speed.data.e  = 0;
        pi_speed.term.Out = 0;
        pi_speed.param.Umax=_IQ(0.8);
        pi_speed.param.Umin=_IQ(-0.8);


//		idmean1.trans.Angle = _IQ(0);
//		idmean1.trans.As = _IQ(0);
//		idmean1.trans.Bs = _IQ(0);
//		idmean1.trans.Cs = _IQ(0);
//		idmean1.trans.Sine = _IQ(0);
//		idmean1.trans.Sine2 = _IQ(0);
//		idmean1.trans.Sine3 = _IQ(0);
//		idmean1.trans.Ds = _IQ(0);
//
//		idmean1.data.countertemp = 1;
//		idmean1.data.ids = _IQ(0);
//		idmean1.data.ids_storage = [_IQ(0), _IQ(0), _IQ(0), _IQ(0), _IQ(0), _IQ(0)];
//		idmean1.data.idtemp = _IQ(0);
//		idmean1.data.sector = _IQ(0);
//		idmean1.data.prev_sector = _IQ(0);


		// phivf.x=phid;
		// phivf.x_1=phid;
		// phivf.y_1=phid;
		// phivf.y=phid;
		// phivf.K=_IQ(0.9333);

		// phivf2.x=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
		// phivf2.x_1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
		// phivf2.y_1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
		// phivf2.y=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
		// phivf2.K=_IQ(0.9868);

		// fa1.phiv=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
		// fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
		// fa1.phiv2=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
		// fa1.phiv3=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;

		timer=_IQ(0);




		RunMotor = TRUE;

		EALLOW;
			EPwm1Regs.TZCLR.bit.OST=1;
			EPwm2Regs.TZCLR.bit.OST=1;
			EPwm3Regs.TZCLR.bit.OST=1;
		EDIS;
	}

	//-------------------
	//the next time CpuTimer0 'counter' reaches Period value go to A2
	A_Task_Ptr = &A1;
	//-------------------
}

////-----------------------------------------------------------------
//void A2(void) // SPARE (not used)
////-----------------------------------------------------------------
//{
//
//	//-------------------
//	//the next time CpuTimer0 'counter' reaches Period value go to A3
//	A_Task_Ptr = &A3;
//	//-------------------
//}
//
////-----------------------------------------
//void A3(void) // SPARE (not used)
////-----------------------------------------
//{
//
//	//-----------------
//	//the next time CpuTimer0 'counter' reaches Period value go to A1
//	A_Task_Ptr = &A1;
//	//-----------------
//}



//=================================================================================
//	B - TASKS (executed in every 5 msec)
//=================================================================================

//----------------------------------- USER ----------------------------------------

//----------------------------------------
void B1(void) // Toggle GPIO-00
//----------------------------------------
{
	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
	//-----------------
	//the next time CpuTimer1 'counter' reaches Period value go to B2
	B_Task_Ptr = &B1;
	//-----------------
}

////----------------------------------------
//void B2(void) //  SPARE
////----------------------------------------
//{
//
//	//-----------------
//	//the next time CpuTimer1 'counter' reaches Period value go to B3
//	B_Task_Ptr = &B3;
//	//-----------------
//}
//
////----------------------------------------
//void B3(void) //  SPARE
////----------------------------------------
//{
//
//	//-----------------
//	//the next time CpuTimer1 'counter' reaches Period value go to B1
//	B_Task_Ptr = &B1;
//	//-----------------
//}

// ==============================================================================
// =============================== MAIN ISR =====================================
// ==============================================================================


interrupt void MainISR(void)
{
// Verifying the ISR
    IsrTicker++;

	if(RunMotor)
	{
// =============================== LEVEL 1 ======================================
// PWM Generated, and Sensed Voltage and DC Current
// ============================================================================== 

#if (BUILDLEVEL==LEVEL1)

	_iq iqVaIn;
	_iq iqVbIn;
	_iq IDCfdbk;

// ------------------------------------------------------------------------------
//    ADC conversion and offset adjustment 
// ------------------------------------------------------------------------------
	iqVaIn =  _IQ15toIQ((AdcResult.ADCRESULT4<<3)-BemfA_offset);
	iqVbIn =  _IQ15toIQ((AdcResult.ADCRESULT5<<3)-BemfA_offset);
	IDCfdbk= (_IQ15toIQ(AdcResult.ADCRESULT7<<3)-IDC_offset)<<1;


// ------------------------------------------------------------------------------
//  Connect inputs of the RMP module and call the ramp control macro
// ------------------------------------------------------------------------------
	rc1.TargetValue = _IQ(0.1);
	RC_MACRO(rc1)

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
	rg1.Freq = _IQ(0.1);//rc1.SetpointValue;
	RG_MACRO(rg1)
    theta= (rg1.Out);

// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//	  update macro.
// ------------------------------------------------------------------------------
	theta_ref1= _IQmpy(D_com,_IQ(0.5));
	theta_ref2= D_com-_IQ(0.333333333333333);

	/*theta_temp1=_IQ(0.25)-theta_ref1;
	theta_temp2=_IQ(0.916666666666667)-theta_ref2;*/
	pwmcntl1.Duty1 = _IQ((theta>=_IQ(0.25)-theta_ref1) && (theta<_IQ(0.25)))+_IQmpy(  d_pwm,_IQ(((theta>=0) && ((theta<_IQ(0.25)-theta_ref1))) || ((theta>=_IQ(0.916666666666667)-theta_ref2) && (theta<_IQ(1)))) );
	/*theta_temp1=_IQ(0.75)-theta_ref1;
	theta_temp2=_IQ(0.416666666666667)-theta_ref2;*/
	pwmcntl1.Duty2 = _IQ(1)-(_IQ((theta>=_IQ(0.75)-theta_ref1) && (theta<_IQ(0.75)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.416666666666667)-theta_ref2) && ((theta<_IQ(0.75)-theta_ref1))))  ));
	/*if (pwmcntl1.Duty2 >= _IQ(1.0)){
	    pwmcntl1.Duty2=pwmcntl1.Duty2-_IQ(0.0000001);
	}*/
	/*pwmcntl1.Duty1 = _IQ(0);
	pwmcntl1.Duty2 = _IQ(0.99999997);*/

    /*theta_temp1=_IQ(0.583333333333333)-theta_ref1;
    theta_temp2=_IQ(0.25)-theta_ref2;*/
	pwmcntl1.Duty3 = _IQ((theta>=_IQ(0.583333333333333)-theta_ref1) && (theta<_IQ(0.583333333333333)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.25)-theta_ref2) && ((theta<_IQ(0.583333333333333)-theta_ref1))))  );
	/*theta_temp1=_IQ(0.083333333333333)+_IQ(1)-theta_ref1;
	theta_temp2=_IQ(0.75)-theta_ref2;*/
	pwmcntl1.Duty4 = _IQ(1)-(_IQ( (theta>=_IQ(0)) && (theta < _IQ(0.083333333333333)) )+_IQ(((theta>=_IQ(0.083333333333333)+_IQ(1)-theta_ref1) && (theta<_IQ(1.0))))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.75)-theta_ref2) && ((theta<_IQ(0.083333333333333)+_IQ(1)-theta_ref1))))  ));

    /*theta_temp1=_IQ(0.916666666666667)-theta_ref1;
    theta_temp2=_IQ(0.583333333333333)-theta_ref2;*/
	pwmcntl1.Duty5 = _IQ((theta>=_IQ(0.916666666666667)-theta_ref1) && (theta<_IQ(0.916666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.583333333333333)-theta_ref2) && ((theta<_IQ(0.916666666666667)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.416666666666667)-theta_ref1;
    theta_temp2=_IQ(0.083333333333333)-theta_ref2;*/
	pwmcntl1.Duty6 = _IQ(1)-(_IQ((theta>=_IQ(0.416666666666667)-theta_ref1) && (theta<_IQ(0.416666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.083333333333333)-theta_ref2) && ((theta<_IQ(0.416666666666667)-theta_ref1))) || ((theta>=_IQ(0.083333333333333)-theta_ref2+_IQ(1)) && (theta<_IQ(1)))) ));

	PWM_CNTL_MACRO(pwmcntl1)

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------
	PwmDacCh1 = (int16)(theta);
	PwmDacCh2 = _IQtoQ15(iqVaIn);
	PwmDacCh3 = _IQtoQ15(iqVbIn);
#if defined(DRV8301) || defined(DRV8302)
	PwmDacCh4 = _IQtoQ15(IDCfdbk);
#endif

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
	DlogCh1 = (int16)(theta);
	DlogCh2 = _IQtoQ15(iqVaIn);
	DlogCh3 = _IQtoQ15(iqVbIn);
	DlogCh4 = _IQtoQ15(IDCfdbk);

#endif // (BUILDLEVEL==LEVEL1)

// =============================== LEVEL 2 ======================================
	// PWM Generated, and Hall-sensored control
// ==============================================================================

#if (BUILDLEVEL==LEVEL2) 

    _iq iqIaIn;
    _iq iqIbIn;
    _iq iqIcIn;
    _iq Te;
    _iq delta;
    _iq d_pwm2;

    D_com = _IQ(0.3333333333333333);
    d_pwm=_IQ(0.3);
    if (hall1.Revolutions >= 350)
    //D_com = _IQ(0.416666666667);
    //if (hall1.Revolutions >= 750)
    D_com = _IQ(0.333333333333333);


// ------------------------------------------------------------------------------
//    ADC conversion and offset adjustment 
// ------------------------------------------------------------------------------
    iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    iqIcIn = (_IQ15toIQ((AdcResult.ADCRESULT3<<3)-cal_offset_C)<<1);
    //IDCfdbk= (_IQ15toIQ(AdcResult.ADCRESULT7<<3)-IDC_offset)<<1;

 if (hall1.Revolutions>=300)
      ClosedFlag=TRUE;

// ------------------------------------------------------------------------------
//    Connect inputs of the HALL module and call the Hall sensor read macro.
// ------------------------------------------------------------------------------
	HALL3_READ_MACRO(hall1)
	if (hall1.CmtnTrigHall==0x7FFF)
	{
	    speed2.TimeStamp = VirtualTimer;
        SPEED_PR_MACRO1(speed2)
	    speed3.Timestamp = VirtualTimer;
        if (hall1.Revolutions<=5){
            speed3.EventPeriod=speed2.EventPeriod_n_1;
            speed3.SUflag=TRUE;
        }else{
            speed3.currenttheta=rg1.Angle;
            speed3.HallGpio=hall1.HallGpioAccepted;
            speed3.EventPeriod=speed2.EventPeriod_n;
            speed3.SUflag=FALSE;
        }

        SE_MACRO1(speed3)
    }

    rg1.Freq = speed3.nextspeed;
    RG_MACRO(rg1)
    fa1.Angle= rg1.Out;   /* motor A 0 */
    fa1.phiv=_IQ(0.0);  /* motor D 48V ~0.9/0.7Nm -0.508 */
    FA_MACRO(fa1)         /* motor C 48V ~0.7Nm -0.05 */
    theta= (fa1.Out);

    backemf1.theta = theta;
    backemf1.speed = _IQdiv(speed3.speed_rads,_IQ(100));//Now,the magnitude of calculated back-emf is 100 times smaller than the actual back-emf!!!
    backemf1.lambda = _IQ(0.037);
    BACKEMF_CAL(backemf1);

//    commutation1.speed_rads = speed3.speed_rads;
//    commutation1.vdc = _IQ(20);
//    commutation1.i_start = _IQ(4);
//    COMMUTATION_DETEC(commutation1);
//    delta = commutation1.delta;
    delta = _IQ(0);

    commutation2.speed_rads = speed3.speed_rads;
    commutation2.vdc = _IQ(20);
    commutation2.i_start = _IQ(4);
    commutation2.lambda = _IQ(0.037);
    COMMUTATION_DUTY(commutation2);
    d_pwm2 = 0;


    torque.ia = _IQmpy(iqIaIn,_IQ(BASE_CURRENT/100)); //Now the calculated current is 100 times smaller than the actual current!!!
    torque.ib = _IQmpy(iqIbIn,_IQ(BASE_CURRENT/100));
    torque.ic = _IQmpy(iqIcIn,_IQ(BASE_CURRENT/100));
    torque.ea = backemf1.ea; //The back-emf is also 100 times smaller than actual value
    torque.eb = backemf1.eb;
    torque.ec = backemf1.ec;

    torque.wr = _IQdiv(backemf1.speed,_IQ(POLES/2)); //Mechanical angular speed is used for torque calculation
    TORQUE_CAL(torque);
    Te = torque.Out; //So the calculated torque should also be 100 times smaller!

   // if (hall1.Revolutions>800)
      //   {pi_speed.term.Ref = _IQ(40);
      //   pi_speed.term.Fbk = speed3.speed_rads;
      //   PI_SPEED_MACRO(pi_speed)}

    Dutycal.theta = theta;
    Dutycal.speed = torque.wr;
    Dutycal.Te = Te;
    //Dutycal.Te_ref = torquecom.torque_ref;
    Dutycal.ea = 0;
    Dutycal.eb = 0;
    Dutycal.ec = 0;
    Dutycal.vdc = _IQdiv(_IQ(20),_IQ(100));
    DUTY_CAL(Dutycal)

    Dutycal2.Te = Te;
    //Dutycal2.Te_ref = torquecom.torque_ref;
    Dutycal2.Te_ref = _IQmpy(_IQ(0.005),_IQ(1.03));
    if (hall1.Revolutions >= 800)
        Dutycal2.Te_ref = _IQmpy(_IQ(0.005),_IQ(1.03));
    Dutycal2.fup = Dutycal.fup;
    Dutycal2.fdown = Dutycal.fdown;
    DUTY_CAL2(Dutycal2)

    if (hall1.Revolutions>500)
    {
        //if (torquecom.Out == 1)
         //   d_pwm = _IQ(0.5);
        //else
            //d_pwm = _IQ(0.3);
       d_pwm = Dutycal2.Out;
        //d_pwm = _IQ(0.5);
    }


// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
	theta_ref1= _IQmpy(D_com,_IQ(0.5));
	theta_ref2= D_com-_IQ(0.333333333333333);
    theta_ref3=  _IQdiv(theta_ref2,_IQ(2));
    if (ClosedFlag == FALSE)
    {

       /*180*/
        //pwmcntl1.Duty1 = _IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==6)),_IQ(1))+_IQmpy(_IQ(hall1.HallGpioAccepted==4),d_pwm);

        //pwmcntl1.Duty2 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==2)),_IQ(1)))+_IQmpy(_IQ(hall1.HallGpioAccepted==3),d_pwm);

        //pwmcntl1.Duty3 = _IQmpy(_IQ((hall1.HallGpioAccepted==3)||(hall1.HallGpioAccepted==6)),_IQ(1))+_IQmpy(_IQ(hall1.HallGpioAccepted==2),d_pwm);

        //pwmcntl1.Duty4 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==1)),_IQ(1)))+_IQmpy(_IQ(hall1.HallGpioAccepted==5),d_pwm);

        //pwmcntl1.Duty5 = _IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==3)),_IQ(1))+_IQmpy(_IQ(hall1.HallGpioAccepted==1),d_pwm);

        //pwmcntl1.Duty6 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==4)),_IQ(1)))+_IQmpy(_IQ(hall1.HallGpioAccepted==6),d_pwm);
       /*180 jinhe code*/
       //pwmcntl1.Duty1 = _IQmpy(_IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==2)),_IQ(1))+_IQmpy(_IQ(hall1.HallGpioAccepted==3),d_pwm);

       //pwmcntl1.Duty2 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==6)),_IQ(1)))+_IQmpy(_IQ(hall1.HallGpioAccepted==4),d_pwm);

      // pwmcntl1.Duty3 = _IQmpy(_IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==4)),_IQ(1))+_IQmpy(_IQ(hall1.HallGpioAccepted==5),d_pwm);

       //pwmcntl1.Duty4 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==3)),_IQ(1)))+_IQmpy(_IQ(hall1.HallGpioAccepted==2),d_pwm);

       //pwmcntl1.Duty5 = _IQmpy(_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==2)),_IQ(1))+_IQmpy(_IQ(hall1.HallGpioAccepted==6),d_pwm);

       //pwmcntl1.Duty6 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==3)||(hall1.HallGpioAccepted==5)),_IQ(1)))+_IQmpy(_IQ(hall1.HallGpioAccepted==1),d_pwm);
       /*Inverter Box*/
             pwmcntl1.Duty2 =  _IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==3));

             pwmcntl1.Duty1 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==4)),d_pwm);

             pwmcntl1.Duty4 = _IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==5));

             pwmcntl1.Duty3 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==3)),d_pwm);

             pwmcntl1.Duty6 = _IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==6));

             pwmcntl1.Duty5 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==1)),d_pwm);
       /*JINHE Paper*/
       // pwmcntl1.Duty1 = _IQmpy(_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==6)),d_pwm);

        //pwmcntl1.Duty2 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==3)||(hall1.HallGpioAccepted==1)),_IQ(1)));

       // pwmcntl1.Duty3 = _IQmpy(_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==3)),d_pwm);

        //pwmcntl1.Duty4 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==4)),_IQ(1)));

        //pwmcntl1.Duty5 = _IQmpy(_IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==5)),d_pwm);

       // pwmcntl1.Duty6 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==2)),_IQ(1)));


        //pwmcntl1.Duty1 = _IQmpy(_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==3)),d_pwm);

        //pwmcntl1.Duty2 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==4)),_IQ(1)));

        //pwmcntl1.Duty3 = _IQmpy(_IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==5)),d_pwm);

        //pwmcntl1.Duty4 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==2)),_IQ(1)));

        //pwmcntl1.Duty5 = _IQmpy(_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==6)),d_pwm);

        //pwmcntl1.Duty6 = /*_IQ(1)-*/(_IQmpy(_IQ((hall1.HallGpioAccepted==3)||(hall1.HallGpioAccepted==1)),_IQ(1)));
    }

    else{
        //JK60_BLDC
        //pwmcntl1.Duty1 = _IQmpy(d_pwm,_IQ((theta>=_IQ(0.083333333333333)-theta_ref3)&&(theta<_IQ(0.416666666666667)+theta_ref3)));
        //pwmcntl1.Duty2 = _IQ(((theta>=_IQ(0.583333333333333)-theta_ref3)&&(theta<_IQ(0.916666666666667)+theta_ref3)));
        //pwmcntl1.Duty3 = _IQmpy(d_pwm,_IQ((((theta>=_IQ(0.75)-theta_ref3)&&(theta<_IQ(1))))||(theta>=_IQ(0))&&(theta<_IQ(0.083333333333333)+theta_ref3)));
        //pwmcntl1.Duty4 = _IQ((theta>=_IQ(0.25)-theta_ref3)&&(theta<_IQ(0.583333333333333)+theta_ref3));
        //pwmcntl1.Duty5 = _IQmpy(d_pwm,_IQ((theta>=_IQ(0.416666666666667)-theta_ref3)&&(theta<_IQ(0.75)+theta_ref3)));
        //pwmcntl1.Duty6 = _IQ((((theta>=_IQ(0.916666666666667)-theta_ref3)&&(theta<_IQ(1))))||(theta>=_IQ(0))&&(theta<_IQ(0.25)+theta_ref3));
        //JK80_BLDC
        pwmcntl1.Duty4 = _IQ((theta>=_IQ(0.083333333333333)-theta_ref3)&&(theta<_IQ(0.416666666666667)+theta_ref3));
        pwmcntl1.Duty3 = _IQmpy(d_pwm,_IQ(((theta>=_IQ(0.583333333333333)-theta_ref3)&&(theta<_IQ(0.916666666666667)+theta_ref3))));
        pwmcntl1.Duty6 = _IQ((((theta>=_IQ(0.75)-theta_ref3)&&(theta<_IQ(1))))||(theta>=_IQ(0))&&(theta<_IQ(0.083333333333333)+theta_ref3));
        pwmcntl1.Duty5 = _IQmpy(d_pwm,_IQ((theta>=_IQ(0.25)-theta_ref3)&&(theta<_IQ(0.583333333333333)+theta_ref3)));
        pwmcntl1.Duty2 = _IQ((theta>=_IQ(0.416666666666667)-theta_ref3)&&(theta<_IQ(0.75)+theta_ref3));
        pwmcntl1.Duty1 = _IQmpy(d_pwm,_IQ((((theta>=_IQ(0.916666666666667)-theta_ref3)&&(theta<_IQ(1))))||(theta>=_IQ(0))&&(theta<_IQ(0.25)+theta_ref3)));

        //pwmcntl1.Duty1 = _IQ(theta>=_IQ(0.916666666666667)&&(theta<(_IQ(0.916666666666667)+delta))) + _IQmpy(d_pwm,_IQ(  (theta>=(_IQ(0.916666666666667)+delta))||(theta<_IQ(0.0833333333333333))||((theta>=(_IQ(0.0833333333333333)+delta))&&(theta<_IQ(0.25)))  )) + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.0833333333333333))&&(theta<(_IQ(0.0833333333333333)+delta))) );
        //pwmcntl1.Duty2 = _IQ( ((theta>=_IQ(0.416666666666667))&&(theta<_IQ(0.583333333333333))) || ((theta>=(_IQ(0.583333333333333)+delta))&&(theta<_IQ(0.75)))  )  + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.583333333333333))&&(theta<(_IQ(0.583333333333333)+delta))) ) ;
        //pwmcntl1.Duty3 = _IQ(theta>=_IQ(0.583333333333333)&&(theta<_IQ(0.583333333333333)+delta)) + _IQmpy(d_pwm,_IQ( ((theta>=_IQ(0.583333333333333)+delta)&&(theta<_IQ(0.75))) || ((theta>=_IQ(0.75)+delta)&&(theta<_IQ(0.916666666666667))) )) + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.75))&&(theta<(_IQ(0.75)+delta))) );
        //pwmcntl1.Duty4 = _IQ( ((theta>=_IQ(0.083333333333333))&&(theta<_IQ(0.25))) || ((theta>=_IQ(0.25)+delta)&&(theta<_IQ(0.416666666666667)))  )  + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.25))&&(theta<(_IQ(0.25)+delta))) ) ;
        //pwmcntl1.Duty5 = _IQ(theta>=_IQ(0.25)&&(theta<_IQ(0.25)+delta)) + _IQmpy(d_pwm,_IQ( ((theta>=_IQ(0.25)+delta)&&(theta<_IQ(0.416666666666667))) || ((theta>=_IQ(0.416666666666667)+delta)&&(theta<_IQ(0.583333333333333))) )) + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.416666666666667))&&(theta<(_IQ(0.416666666666667)+delta))) );
        //pwmcntl1.Duty6 = _IQ( ((theta>=_IQ(0.75))&&(theta<_IQ(0.916666666666667))) || (theta>=_IQ(0.916666666666667)+delta) || (theta<_IQ(0.083333333333333))  )  +  _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.916666666666667))&&(theta<(_IQ(0.916666666666667)+delta))) );
        //H-PWM L-ON mode maxonB BLDC work for both 180 and 120 commutation)
        //pwmcntl1.Duty1 = _IQmpy(d_pwm, (_IQ(((theta>=_IQ(0.083333333333333)-theta_ref3)&&(theta<_IQ(0.16666666666667)))||(theta>=_IQ(0.3333333333333)&&(theta<_IQ(0.416666666666667)+theta_ref3)))+_IQmpy(_IQ(1), _IQ((theta>=_IQ(0.16666666666667))&&(theta<_IQ(0.3333333333333))))));
        //pwmcntl1.Duty2 = _IQ(((theta>=_IQ(0.583333333333333)-theta_ref3)&&(theta<_IQ(0.66666666666667)))||(theta>=_IQ(0.8333333333333)&&(theta<_IQ(0.916666666666667)+theta_ref3)))+_IQmpy(_IQ(1), _IQ((theta>=_IQ(0.66666666666667))&&(theta<_IQ(0.8333333333333))));
        //pwmcntl1.Duty3 = _IQmpy(d_pwm, (_IQ(((theta>=_IQ(0.416666666666667)-theta_ref3)&&(theta<_IQ(0.5)))||(theta>=_IQ(0.66666666666667)&&(theta<_IQ(0.75)+theta_ref3)))+_IQmpy(_IQ(1), _IQ((theta>=_IQ(0.5))&&(theta<_IQ(0.66666666666667))))));
        //pwmcntl1.Duty4 = _IQ(((theta>=_IQ(0.916666666666667)-theta_ref3)&&(theta<_IQ(1)))||(theta>=_IQ(0.166666666666667)&&(theta<_IQ(0.25)+theta_ref3)))+_IQmpy(_IQ(1), _IQ((theta>=_IQ(0.0))&&(theta<_IQ(0.166666666666667))));
        //pwmcntl1.Duty5 =  _IQmpy(d_pwm, (_IQ(((theta>=_IQ(0.75)-theta_ref3)&&(theta<_IQ(0.833333333333333)))||(theta>=_IQ(0.0)&&(theta<_IQ(0.0833333333333333)+theta_ref3)))+_IQmpy(_IQ(1), _IQ((theta>=_IQ(0.833333333333333))&&(theta<_IQ(1))))));
        //pwmcntl1.Duty6 = _IQ(((theta>=_IQ(0.25)-theta_ref3)&&(theta<_IQ(0.333333333333333)))||(theta>=_IQ(0.5)&&(theta<_IQ(0.5833333333333333)+theta_ref3)))+_IQmpy(_IQ(1), _IQ((theta>=_IQ(0.333333333333333))&&(theta<_IQ(0.5))));
        //ON-PWM-ON mode (can work for both 180 and 120 commutation)
        //pwmcntl1.Duty1 = _IQ(((theta>=_IQ(0.083333333333333)-theta_ref3)&&(theta<_IQ(0.16666666666667)))||(theta>=_IQ(0.3333333333333)&&(theta<_IQ(0.416666666666667)+theta_ref3)))+_IQmpy(d_pwm, _IQ((theta>=_IQ(0.16666666666667))&&(theta<_IQ(0.3333333333333))));
        //pwmcntl1.Duty2 = _IQ(((theta>=_IQ(0.583333333333333)-theta_ref3)&&(theta<_IQ(0.66666666666667)))||(theta>=_IQ(0.8333333333333)&&(theta<_IQ(0.916666666666667)+theta_ref3)))+_IQmpy(d_pwm, _IQ((theta>=_IQ(0.66666666666667))&&(theta<_IQ(0.8333333333333))));
        //pwmcntl1.Duty3 = _IQ(((theta>=_IQ(0.416666666666667)-theta_ref3)&&(theta<_IQ(0.5)))||(theta>=_IQ(0.66666666666667)&&(theta<_IQ(0.75)+theta_ref3)))+_IQmpy(d_pwm, _IQ((theta>=_IQ(0.5))&&(theta<_IQ(0.66666666666667))));
        //pwmcntl1.Duty4 = _IQ(((theta>=_IQ(0.916666666666667)-theta_ref3)&&(theta<_IQ(1)))||(theta>=_IQ(0.166666666666667)&&(theta<_IQ(0.25)+theta_ref3)))+_IQmpy(d_pwm, _IQ((theta>=_IQ(0.0))&&(theta<_IQ(0.166666666666667))));
        //pwmcntl1.Duty5 = _IQ(((theta>=_IQ(0.75)-theta_ref3)&&(theta<_IQ(0.833333333333333)))||(theta>=_IQ(0.0)&&(theta<_IQ(0.0833333333333333)+theta_ref3)))+_IQmpy(d_pwm, _IQ((theta>=_IQ(0.833333333333333))&&(theta<_IQ(1))));
        //pwmcntl1.Duty6 = _IQ(((theta>=_IQ(0.25)-theta_ref3)&&(theta<_IQ(0.333333333333333)))||(theta>=_IQ(0.5)&&(theta<_IQ(0.5833333333333333)+theta_ref3)))+_IQmpy(d_pwm, _IQ((theta>=_IQ(0.333333333333333))&&(theta<_IQ(0.5))));
        //PWM-ON mode (this mode can work for 120 commutation, it cannot work for 180 degree commutation!)
        //pwmcntl1.Duty1 = (_IQ((theta>=_IQ(0.25))&&(theta<_IQ(0.416666666666667)+theta_ref3))+_IQmpy( d_pwm, _IQ((theta>=_IQ(0.0833333333333333)-theta_ref3)&&(theta<_IQ(0.25)))));
        //pwmcntl1.Duty2 = (_IQ((theta>=_IQ(0.75))&&(theta<_IQ(0.916666666666667)+theta_ref3))+_IQmpy( d_pwm, _IQ((theta>=_IQ(0.5833333333333333)-theta_ref3)&&(theta<_IQ(0.75)))));
        //pwmcntl1.Duty3 = (_IQ((theta>=_IQ(0.5833333333333333))&&(theta<_IQ(0.75)+theta_ref3))+_IQmpy( d_pwm, _IQ((theta>=_IQ(0.416666666666667)-theta_ref3)&&(theta<_IQ(0.5833333333333333)))));
        //pwmcntl1.Duty4 = (_IQ((theta>=_IQ(0.083333333333333))&&(theta<_IQ(0.25)+theta_ref3))+_IQmpy( d_pwm, _IQ((theta>=_IQ(0.916666666666667)-theta_ref3)||(theta<_IQ(0.083333333333333)))));
        //pwmcntl1.Duty5 = (_IQ((theta>=_IQ(0.916666666666667))||(theta<_IQ(0.083333333333333)+theta_ref3))+_IQmpy( d_pwm, _IQ((theta>=_IQ(0.75)-theta_ref3)&&(theta<_IQ(0.916666666666667)))));
        //pwmcntl1.Duty6 = (_IQ((theta>=_IQ(0.416666666666667))&&(theta<_IQ(0.583333333333333)+theta_ref3))+_IQmpy( d_pwm, _IQ((theta>=_IQ(0.25)-theta_ref3)&&(theta<_IQ(0.416666666666667)))));


        //pwmcntl1.Duty1 = _IQ((theta>=_IQ(0.416666666666667)-theta_ref1) && (theta<_IQ(0.416666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.083333333333333)-theta_ref2) && ((theta<_IQ(0.416666666666667)-theta_ref1))) || ((theta>=_IQ(0.083333333333333)-theta_ref2+_IQ(1)) && (theta<_IQ(1)))) );
        //pwmcntl1.Duty2 = _IQ((theta>=_IQ(0.916666666666667)-theta_ref1) && (theta<_IQ(0.916666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.583333333333333)-theta_ref2) && ((theta<_IQ(0.916666666666667)-theta_ref1))))  );
        //pwmcntl1.Duty3 = _IQ((theta>=_IQ(0.75)-theta_ref1) && (theta<_IQ(0.75)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.416666666666667)-theta_ref2) && ((theta<_IQ(0.75)-theta_ref1))))  );
        //pwmcntl1.Duty4 = _IQ((theta1>=_IQ(0.25)-theta_ref1) && (theta1<_IQ(0.25)))+_IQmpy(  d_pwm,_IQ(((theta1>=0) && ((theta1<_IQ(0.25)-theta_ref1))) || ((theta1>=_IQ(0.916666666666667)-theta_ref2) && (theta1<_IQ(1)))) );
        //pwmcntl1.Duty5 = _IQ( (theta1>=_IQ(0)) && (theta1 < _IQ(0.083333333333333)) )+_IQ(((theta1>=_IQ(0.083333333333333)+_IQ(1)-theta_ref1) && (theta1<_IQ(1.0))))+_IQmpy(  d_pwm,_IQ(((theta1>=_IQ(0.75)-theta_ref2) && ((theta1<_IQ(0.083333333333333)+_IQ(1)-theta_ref1))))  );
        //pwmcntl1.Duty6 = _IQ((theta1>=_IQ(0.583333333333333)-theta_ref1) && (theta1<_IQ(0.583333333333333)))+_IQmpy(  d_pwm,_IQ(((theta1>=_IQ(0.25)-theta_ref2) && ((theta1<_IQ(0.583333333333333)-theta_ref1))))  );


    //pwmcntl1.Duty1 = _IQmpy(d_pwm,_IQ((theta>=_IQ(0.25)-theta_ref1) && (theta<_IQ(0.25))))+_IQ(((theta>=0) && ((theta<_IQ(0.25)-theta_ref1))) || ((theta>=_IQ(0.916666666666667)-theta_ref2) && (theta<_IQ(1)))) ;

    //pwmcntl1.Duty2 = _IQ(1)-(_IQmpy(d_pwm,_IQ((theta>=_IQ(0.75)-theta_ref1) && (theta<_IQ(0.75))))+_IQ(((theta>=_IQ(0.416666666666667)-theta_ref2) && ((theta<_IQ(0.75)-theta_ref1))))  );

    //pwmcntl1.Duty3 = _IQmpy(d_pwm,_IQ((theta>=_IQ(0.583333333333333)-theta_ref1) && (theta<_IQ(0.583333333333333))))+_IQ(((theta>=_IQ(0.25)-theta_ref2) && ((theta<_IQ(0.583333333333333)-theta_ref1)))  );

    //pwmcntl1.Duty4 = _IQ(1)-(_IQmpy(d_pwm,_IQ( (theta>=_IQ(0)) && (theta < _IQ(0.083333333333333)) )+_IQ(((theta>=_IQ(0.083333333333333)+_IQ(1)-theta_ref1) && (theta<_IQ(1.0)))))+_IQ(((theta>=_IQ(0.75)-theta_ref2) && ((theta<_IQ(0.083333333333333)+_IQ(1)-theta_ref1)))));

    //pwmcntl1.Duty5 = _IQmpy(d_pwm,_IQ((theta>=_IQ(0.916666666666667)-theta_ref1) && (theta<_IQ(0.916666666666667))))+_IQ(((theta>=_IQ(0.583333333333333)-theta_ref2) && ((theta<_IQ(0.916666666666667)-theta_ref1)))  );

    //pwmcntl1.Duty6 = _IQ(1)-(_IQmpy(d_pwm,_IQ((theta>=_IQ(0.416666666666667)-theta_ref1) && (theta<_IQ(0.416666666666667))))+_IQ(((theta>=_IQ(0.083333333333333)-theta_ref2) && ((theta<_IQ(0.416666666666667)-theta_ref1))) || ((theta>=_IQ(0.083333333333333)-theta_ref2+_IQ(1)) && (theta<_IQ(1)))) );
    }
	PWM_CNTL_MACRO(pwmcntl1)


if (hall1.Revolutions >= 600)
{
    if (Observation_data_i >= 300)
       k=0;
   else
        k=1;
}

if (k == 1)
    {
    if (Observation_data_a%10 == 1)
    {
    Observation_data_ia[Observation_data_i] = _IQtoQ15(d_pwm);
    Observation_data_te[Observation_data_i] = _IQtoQ15(torque.Out);
    Observation_data_i++;
    }
    Observation_data_a++;
    if (Observation_data_i >= 300)
    k = 0;
    }

// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module 
// ------------------------------------------------------------------------------
	//PwmDacCh1 =  _IQtoQ15(torque.ea);
	//PwmDacCh1 = _IQtoQ15(Dutycal.Te);
	PwmDacCh1 = (MyTimer * 4096.0L);
	PwmDacCh2 = _IQtoQ15(torque.test1);
	//PwmDacCh4 = (hall1.HallGpioAccepted * 4096.0L);
	//PwmDacCh3 = _IQtoQ15(pi_speed.term.Out);
	PwmDacCh3 = _IQtoQ15(theta);
	//PwmDacCh2 = _IQtoQ15(theta);
	//PwmDacCh3 = (hall1.HallGpioAccepted * 4096.0L);
	#if defined(DRV8301) || defined(DRV8302)
	PwmDacCh4 = (hall1.HallGpioAccepted * 4096.0L);
	#endif

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module 
// ------------------------------------------------------------------------------
	DlogCh1 =_IQtoQ15(d_pwm);
	DlogCh2 = _IQtoQ15(Dutycal.fup);
	//DlogCh2 = _IQtoQ15(Dutycal2.Out);
	//DlogCh3 = _IQtoQ15(iqVbIn);
	DlogCh4 = _IQtoQ15(Dutycal2.Out);
	//DlogCh4 = _IQtoQ15(iqVaIn);

#endif // (BUILDLEVEL==LEVEL2)

// =============================== LEVEL 3 ======================================
// Single point operation with MTPA
// ============================================================================== 

#if (BUILDLEVEL==LEVEL3) 

    _iq iqIaIn;
    _iq iqIbIn;


    d_pwm=_IQ(0.5);
    D_com=_IQ(0.333333333);
    if (hall1.Revolutions>=550)
      D_com = _IQ(0.444444444);
      d_pwm = _IQ(0.2);
    if (hall1.Revolutions>=1050)
      D_com = _IQ(0.5);
      d_pwm = _IQ(0.2);

    //0.333333333333333
    //0.388888888888889
    //0.444444444444444
    //0.5


/*
    if (D_com==_IQ(0.5)){
        phid=_IQ(-0.16);
        //phid=_IQ(-0.152);
        //phid=_IQ(-0.148);
        //phid=_IQ(-0.14);
        phid=_IQ(-0.138);
        //phid=_IQ(-0.134);
        //phid=_IQ(-0.13);
    }else if (D_com==_IQ(0.444444444444444)){
        phid=_IQ(-0.164);
        //phid=_IQ(-0.158);
        //phid=_IQ(-0.158);
        //phid=_IQ(-0.158);
        phid=_IQ(-0.158);
        //phid=_IQ(-0.15);
        //phid=_IQ(-0.134);
    }else if (D_com==_IQ(0.388888888888889)){
        //phid=_IQ(-0.184);
        //phid=_IQ(-0.184);
        //phid=_IQ(-0.174);
        //phid=_IQ(-0.162);
        phid=_IQ(-0.152);
        //phid=_IQ(-0.144);
        //phid=_IQ(-0.138);
    } else{
        phid=_IQ(-0.174);
    }
*/

   //----with two switches off
   //phid=_IQ(-0.134);//D_com 120
   //phid=_IQ(-0.138);//D_com 125:0.347222222222222
   //phid=_IQ(-0.142);//D_com 130:0.361111111111111
   //phid=_IQ(-0.142);//D_com 135:0.375000000000000
   //phid=_IQ(-0.142);//D_com 140:0.388888888888889
   //phid=_IQ(-0.142);//D_com 145:0.402777777777778
   //phid=_IQ(-0.146);//D_com 150:0.416666666666667
   //phid=_IQ(-0.146);//D_com 155:0.430555555555556
   //phid=_IQ(-0.144);//D_com 160:0.444444444444444
   //phid=_IQ(-0.142);//D_com 165:0.458333333333333
   //phid=_IQ(-0.144);//D_com 170:0.472222222222222
   //phid=_IQ(-0.144);//D_com 175:0.486111111111111
   //phid=_IQ(-0.144);//D_com 180:0.5
// ------------------------------------------------------------------------------
//    ADC conversion and offset adjustment 
// ------------------------------------------------------------------------------
    iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);

// ------------------------------------------------------------------------------
//    Connect inputs of the HALL module and call the Hall sensor read macro.
// ------------------------------------------------------------------------------
    HALL3_READ_MACRO(hall1)
    if (hall1.CmtnTrigHall==0x7FFF)
    {
        speed2.TimeStamp = VirtualTimer;
        SPEED_PR_MACRO1(speed2)

        if (hall1.Revolutions<=5){
            speed3.EventPeriod=speed2.EventPeriod_n_1;
            speed3.SUflag=TRUE;
        }else{
            speed3.currenttheta=rg1.Angle;
            speed3.HallGpio=hall1.HallGpioAccepted;
            speed3.EventPeriod=speed2.EventPeriod_n;
            speed3.SUflag=FALSE;
        }

        SE_MACRO1(speed3)

    }


    rg1.Freq = speed3.nextspeed;
    RG_MACRO(rg1)
    fa1.Angle= rg1.Out;
    if (hall1.Revolutions==-1){
        fa1.phiv=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }
    //fa1.phiv=_IQ(-0.055);
    FA_MACRO(fa1)
    theta=fa1.Out;


// ------------------------------------------------------------------------------
// mean of ids
// ------------------------------------------------------------------------------
    if (theta>=_IQ(0.583333333333333))
    {
        if (theta>=_IQ(0.75)){
            idmean1.data.idtemp=_IQ(0);
            idmean1.data.countertemp=_IQ(0);
        }else{
            idmean1.trans.As=iqIaIn;
            idmean1.trans.Bs=iqIbIn;
            idmean1.trans.Angle = rg1.Out+phid;
            MEAN_MACRO(idmean1)

            if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq))>=_IQ(0.75)){
                idmean1.data.ids=_IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                MEAN_FILTER(idmean1);
            }

        }
    }


// ------------------------------------------------------------------------------
//    Connect inputs of the PID_REG3 module and call the PID current controller
//    macro.
// ------------------------------------------------------------------------------

    if (hall1.Revolutions<=30){
        fa1.phiv=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }else{
        pi1_id.term.Ref = _IQ(0);
        pi1_id.term.Fbk = idmean1.filter.Out;
        PI_MACRO(pi1_id)
        fa1.phiv=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }

//fa1.phiv=_IQ(-0.1155);
// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
    theta_ref1= _IQmpy(D_com,_IQ(0.5));
    theta_ref2= D_com-_IQ(0.333333333333333);

    /*theta_temp1=_IQ(0.25)-theta_ref1;
    theta_temp2=_IQ(0.916666666666667)-theta_ref2;*/
    pwmcntl1.Duty1 = _IQ((theta>=_IQ(0.25)-theta_ref1) && (theta<_IQ(0.25)))+_IQmpy(  d_pwm,_IQ(((theta>=0) && ((theta<_IQ(0.25)-theta_ref1))) || ((theta>=_IQ(0.916666666666667)-theta_ref2) && (theta<_IQ(1)))) );
    /*theta_temp1=_IQ(0.75)-theta_ref1;
    theta_temp2=_IQ(0.416666666666667)-theta_ref2;*/
    pwmcntl1.Duty2 = _IQ(1)-_IQ((theta>=_IQ(0.75)-theta_ref1) && (theta<_IQ(0.75)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.416666666666667)-theta_ref2) && ((theta<_IQ(0.75)-theta_ref1))))  );
    /*if (pwmcntl1.Duty2 >= _IQ(1.0)){
    pwmcntl1.Duty2=pwmcntl1.Duty2-_IQ(0.0000001);
    }*/
    /*pwmcntl1.Duty1 = _IQ(0);
    pwmcntl1.Duty2 = _IQ(0.99999997);*/

    /*theta_temp1=_IQ(0.583333333333333)-theta_ref1;
    theta_temp2=_IQ(0.25)-theta_ref2;*/
    pwmcntl1.Duty3 = _IQ((theta>=_IQ(0.583333333333333)-theta_ref1) && (theta<_IQ(0.583333333333333)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.25)-theta_ref2) && ((theta<_IQ(0.583333333333333)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.083333333333333)+_IQ(1)-theta_ref1;
    theta_temp2=_IQ(0.75)-theta_ref2;*/
    pwmcntl1.Duty4 = _IQ(1)-_IQ( (theta>=_IQ(0)) && (theta < _IQ(0.083333333333333)) )+_IQ(((theta>=_IQ(0.083333333333333)+_IQ(1)-theta_ref1) && (theta<_IQ(1.0))))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.75)-theta_ref2) && ((theta<_IQ(0.083333333333333)+_IQ(1)-theta_ref1))))  );

    /*theta_temp1=_IQ(0.916666666666667)-theta_ref1;
    theta_temp2=_IQ(0.583333333333333)-theta_ref2;*/
    pwmcntl1.Duty5 = _IQ((theta>=_IQ(0.916666666666667)-theta_ref1) && (theta<_IQ(0.916666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.583333333333333)-theta_ref2) && ((theta<_IQ(0.916666666666667)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.416666666666667)-theta_ref1;
    theta_temp2=_IQ(0.083333333333333)-theta_ref2;*/
    pwmcntl1.Duty6 = _IQ(1)-_IQ((theta>=_IQ(0.416666666666667)-theta_ref1) && (theta<_IQ(0.416666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.083333333333333)-theta_ref2) && ((theta<_IQ(0.416666666666667)-theta_ref1))) || ((theta>=_IQ(0.083333333333333)-theta_ref2+_IQ(1)) && (theta<_IQ(1)))) );

    PWM_CNTL_MACRO(pwmcntl1)


// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(rg1.Out);
    //PwmDacCh2 = _IQtoQ15(idmean1.filter.Out);
    //PwmDacCh3 = _IQtoQ15(iqIaIn);
    PwmDacCh2 = _IQtoQ15(theta);
    //PwmDacCh3 = (hall1.HallGpioAccepted * 4096.0L);
//#if defined(DRV8301) || defined(DRV8302)
 //   PwmDacCh4 = _IQtoQ15(fa1.phiv);
//#endif

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    // DlogCh1 = _IQtoQ15(theta);
   // DlogCh2 = _IQtoQ15(idmean1.filter.Out);
   // DlogCh3 = _IQtoQ15(iqIaIn);
   // DlogCh4 = _IQtoQ15(fa1.phiv);

#endif // (BUILDLEVEL==LEVEL3)

// =============================== LEVEL 4 ======================================
// Calibration
// ==============================================================================

#if (BUILDLEVEL==LEVEL4)

    _iq iqIaIn;
    _iq iqIbIn;
    _iq iqIcIn;
    _iq iqVaIn;
    _iq iqVbIn;
    _iq iqVcIn;
    _iq temp_offset_A;
    _iq temp_offset_B;
    _iq temp_offset_C;

// ------------------------------------------------------------------------------
//    ADC conversion and offset adjustment
// ------------------------------------------------------------------------------
    iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    iqIcIn = (_IQ15toIQ((AdcResult.ADCRESULT3<<3)-cal_offset_C)<<1);

    iqVaIn =  _IQ15toIQ((AdcResult.ADCRESULT4<<3)-BemfA_offset);
    iqVbIn =  _IQ15toIQ((AdcResult.ADCRESULT5<<3)-BemfB_offset);
    iqVcIn =  _IQ15toIQ((AdcResult.ADCRESULT6<<3)-BemfC_offset);

    cal_offset_A = _IQtoIQ15(_IQmpy(cal_filt_gain,iqIaIn)) + cal_offset_A;
    cal_offset_B = _IQtoIQ15(_IQmpy(cal_filt_gain,iqIbIn)) + cal_offset_B;
    cal_offset_C = _IQtoIQ15(_IQmpy(cal_filt_gain,iqIcIn)) + cal_offset_C;

    BemfA_offset = _IQmpy(cal_filt_gain,iqVaIn) + BemfA_offset;
    BemfB_offset = _IQmpy(cal_filt_gain,iqVbIn) + BemfB_offset;
    BemfC_offset = _IQmpy(cal_filt_gain,iqVcIn) + BemfC_offset;


    temp_offset_A=_IQ15toIQ(cal_offset_A);
    temp_offset_B=_IQ15toIQ(cal_offset_B);
    temp_offset_C=_IQ15toIQ(cal_offset_C);

    pwmcntl1.Duty1=_IQ(0);
    pwmcntl1.Duty2=_IQ(0);
    pwmcntl1.Duty3=_IQ(0);
    pwmcntl1.Duty4=_IQ(0);
    pwmcntl1.Duty5=_IQ(0);
    pwmcntl1.Duty6=_IQ(0);

    PWM_CNTL_MACRO(pwmcntl1)


// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(iqVaIn);
    PwmDacCh2 = _IQtoQ15(iqIcIn);
    PwmDacCh3 = _IQtoQ15(temp_offset_A);
#if defined(DRV8301) || defined(DRV8302)
    PwmDacCh4 = _IQtoQ15(temp_offset_C);
#endif

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(iqIaIn);
    DlogCh2 = _IQtoQ15(iqIcIn);
    DlogCh3 = _IQtoQ15(temp_offset_A);
    DlogCh4 = _IQtoQ15(temp_offset_C);

#endif // (BUILDLEVEL==LEVEL4)

// =============================== LEVEL 5 ======================================
// ==============================================================================

#if (BUILDLEVEL==LEVEL5)

    _iq iqIaIn;
    _iq iqIbIn;

// ------------------------------------------------------------------------------
//    ADC conversion and offset adjustment
// ------------------------------------------------------------------------------
    iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);


// ------------------------------------------------------------------------------
//   Duty cylce adjustment
// ------------------------------------------------------------------------------
    d_pwm1 += _IQ(T*0.04);
    d_pwm = _IQsat(d_pwm1,_IQ(0.5),_IQ(0.25));
    if (d_pwm==_IQ(0.5)){
        D_com1 += _IQ(T*0.01);
        D_com = _IQsat(D_com1,_IQ(0.5),_IQ(0.3333333333));
    }else{
        D_com1 = _IQ(0.3333333333);
        D_com = _IQ(0.3333333333);
    }

    if (D_com1==_IQ(0.52)){
        EnableFlag=FALSE;
    }

// ------------------------------------------------------------------------------
//    Connect inputs of the HALL module and call the Hall sensor read macro.
// ------------------------------------------------------------------------------
    HALL3_READ_MACRO(hall1)
    if (hall1.CmtnTrigHall==0x7FFF)
    {
        speed2.TimeStamp = VirtualTimer;
        SPEED_PR_MACRO1(speed2)

        if (hall1.Revolutions<=5){
            speed3.EventPeriod=speed2.EventPeriod_n_1;
            speed3.SUflag=TRUE;
        }else{
            speed3.currenttheta=rg1.Angle;
            speed3.phic = _IQ(0.00);
            speed3.HallGpio=hall1.HallGpioAccepted;
            speed3.EventPeriod=speed2.EventPeriod_n;
            speed3.SUflag=FALSE;
        }

        SE_MACRO1(speed3)

    }


    rg1.Freq = speed3.nextspeed;
    RG_MACRO(rg1)
    fa1.Angle= rg1.Out;
    if (hall1.Revolutions==-1){
        fa1.phiv=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }
    //fa1.phiv=phid;
    FA_MACRO(fa1)
    theta=fa1.Out;

// ------------------------------------------------------------------------------
// mean of ids
// ------------------------------------------------------------------------------
    if (theta>=_IQ(0.583333333333333)){
        if (theta>=_IQ(0.75)){
            idmean1.data.idtemp=_IQ(0);
            idmean1.data.countertemp=_IQ(0);
        }else{
            idmean1.trans.As=iqIaIn;
            idmean1.trans.Bs=iqIbIn;
            idmean1.trans.Angle = rg1.Out+phid;
            MEAN_MACRO(idmean1)

            if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq))>=_IQ(0.75)){
                idmean1.data.ids=_IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                MEAN_FILTER(idmean1)
            }

        }
    }


// ------------------------------------------------------------------------------
//    Connect inputs of the PID_REG3 module and call the PID current controller
//    macro.
// ------------------------------------------------------------------------------

    if (hall1.Revolutions<=30){
        fa1.phiv=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }else{
        pi1_id.term.Ref = _IQ(0);
        pi1_id.term.Fbk = idmean1.filter.Out;
        PI_MACRO(pi1_id)
        fa1.phiv=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }


// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
    theta_ref1= _IQmpy(D_com,_IQ(0.5));
    theta_ref2= D_com-_IQ(0.333333333333333);

    /*theta_temp1=_IQ(0.25)-theta_ref1;
    theta_temp2=_IQ(0.916666666666667)-theta_ref2;*/
    pwmcntl1.Duty1 = _IQ((theta>=_IQ(0.25)-theta_ref1) && (theta<_IQ(0.25)))+_IQmpy(  d_pwm,_IQ(((theta>=0) && ((theta<_IQ(0.25)-theta_ref1))) || ((theta>=_IQ(0.916666666666667)-theta_ref2) && (theta<_IQ(1)))) );
    /*theta_temp1=_IQ(0.75)-theta_ref1;
    theta_temp2=_IQ(0.416666666666667)-theta_ref2;*/
    pwmcntl1.Duty2 = _IQ((theta>=_IQ(0.75)-theta_ref1) && (theta<_IQ(0.75)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.416666666666667)-theta_ref2) && ((theta<_IQ(0.75)-theta_ref1))))  );
    /*if (pwmcntl1.Duty2 >= _IQ(1.0)){
    pwmcntl1.Duty2=pwmcntl1.Duty2-_IQ(0.0000001);
    }*/
    /*pwmcntl1.Duty1 = _IQ(0);
    pwmcntl1.Duty2 = _IQ(0.99999997);*/

    /*theta_temp1=_IQ(0.583333333333333)-theta_ref1;
    theta_temp2=_IQ(0.25)-theta_ref2;*/
    pwmcntl1.Duty3 = _IQ((theta>=_IQ(0.583333333333333)-theta_ref1) && (theta<_IQ(0.583333333333333)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.25)-theta_ref2) && ((theta<_IQ(0.583333333333333)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.083333333333333)+_IQ(1)-theta_ref1;
    theta_temp2=_IQ(0.75)-theta_ref2;*/
    pwmcntl1.Duty4 = _IQ( (theta>=_IQ(0)) && (theta < _IQ(0.083333333333333)) )+_IQ(((theta>=_IQ(0.083333333333333)+_IQ(1)-theta_ref1) && (theta<_IQ(1.0))))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.75)-theta_ref2) && ((theta<_IQ(0.083333333333333)+_IQ(1)-theta_ref1))))  );

    /*theta_temp1=_IQ(0.916666666666667)-theta_ref1;
    theta_temp2=_IQ(0.583333333333333)-theta_ref2;*/
    pwmcntl1.Duty5 = _IQ((theta>=_IQ(0.916666666666667)-theta_ref1) && (theta<_IQ(0.916666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.583333333333333)-theta_ref2) && ((theta<_IQ(0.916666666666667)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.416666666666667)-theta_ref1;
    theta_temp2=_IQ(0.083333333333333)-theta_ref2;*/
    pwmcntl1.Duty6 = _IQ((theta>=_IQ(0.416666666666667)-theta_ref1) && (theta<_IQ(0.416666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.083333333333333)-theta_ref2) && ((theta<_IQ(0.416666666666667)-theta_ref1))) || ((theta>=_IQ(0.083333333333333)-theta_ref2+_IQ(1)) && (theta<_IQ(1)))) );

    PWM_CNTL_MACRO(pwmcntl1)


// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(pwmcntl1.Duty1);
    PwmDacCh3 = (hall1.HallGpioAccepted* 4096.0L);
    //PwmDacCh1 = _IQtoQ15(theta);
    //PwmDacCh2 = _IQtoQ15(idmean1.filter.Out);
    //PwmDacCh3 = _IQtoQ15(d_pwm1);
#if defined(DRV8301) || defined(DRV8302)
    PwmDacCh4 = _IQtoQ15(fa1.phiv);
#endif

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(theta);
    DlogCh2 = _IQtoQ15(idmean1.filter.Out);
    DlogCh3 = _IQtoQ15(d_pwm1);
    DlogCh4 = _IQtoQ15(fa1.phiv);

#endif // (BUILDLEVEL==LEVEL5)
// =============================== LEVEL 6 ======================================
// ==============================================================================
#if (BUILDLEVEL==LEVEL6)

    _iq iqIaIn;
    _iq iqIbIn;
    _iq iqIcIn;
    _iq marker;

    timer += _IQ(T*0.05);
    timer = _IQsat(timer,_IQ(1),_IQ(0));

    if (timer<_IQ(0.9)){
        /*d_pwm=_IQ(0.9);
        D_com=_IQ(0.333333333333333);*/
        d_pwm=_IQ(1);
        D_com=_IQ(0.388888888888889);
    }else{
        /*d_pwm=_IQ(0.9);
        D_com=_IQ(0.333333333333333);*/
        d_pwm=_IQ(1);
        D_com=_IQ(0.388888888888889);
    }



    //0.333333333333333
    //0.388888888888889
    //0.444444444444444
    //0.499999999999999
    phid=_IQ(-0.17);
// ------------------------------------------------------------------------------
//    ADC conversion and offset adjustment
// ------------------------------------------------------------------------------
    iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    iqIcIn = (_IQ15toIQ((AdcResult.ADCRESULT3<<3)-cal_offset_C)<<1);

// ------------------------------------------------------------------------------
//    Connect inputs of the HALL module and call the Hall sensor read macro.
// ------------------------------------------------------------------------------
    HALL3_READ_MACRO(hall1)
    if (hall1.CmtnTrigHall==0x7FFF)
    {
        speed2.TimeStamp = VirtualTimer;
        SPEED_PR_MACRO1(speed2)

        if (hall1.Revolutions<=5){
            speed3.EventPeriod=speed2.EventPeriod_n_1;
            speed3.SUflag=TRUE;
        }else{
            speed3.currenttheta=rg1.Angle;
            speed3.HallGpio=hall1.HallGpioAccepted;
            speed3.EventPeriod=speed2.EventPeriod_n;
            speed3.SUflag=FALSE;
        }

        SE_MACRO1(speed3)
    }

   if (hall1.Revolutions>1000){
            if (timer>_IQ(0.9)){
                /*phid1=_IQ(-0.3242)+_IQdiv(_IQ(0.134),speed3.nextspeed);*/
                phid1=_IQ(-0.3242)+_IQdiv(_IQ(0.158),speed3.nextspeed);
            }else{
                phid1=_IQ(-0.3242)+_IQdiv(_IQ(0.158),speed3.nextspeed);
            }


            phivf.x=_IQsat(phid1, _IQ(-0.05), _IQ(-0.17));
            PHIV_FILTER_MACRO(phivf)
            phid=phivf.y;
            phid = _IQsat(phivf.y, _IQ(-0.05), _IQ(-0.17));
    }
   /*phid=_IQ(-0.156);*/

    rg1.Freq = speed3.nextspeed;
    RG_MACRO(rg1)
    fa1.Angle= rg1.Out;
    if (hall1.Revolutions==-1){
        fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }

    FA_FILTER(fa1)
    FA_MACRO(fa1)
    theta=fa1.Out;


// ------------------------------------------------------------------------------
// mean of ids && PI controller
// ------------------------------------------------------------------------------
    if ((theta>=_IQ(0.583333333333333))&&(theta<=_IQ(0.833333333333333))){
        if ((theta>=_IQ(0.75)) && (theta<=_IQ(0.833333333333333))){
            idmean1.data.idtemp=_IQ(0);
            idmean1.data.countertemp=_IQ(0);
        }else{
            idmean1.data.sector=_IQ(5);
            idmean1.trans.As=iqIaIn;
            idmean1.trans.Bs=iqIbIn;
            idmean1.trans.Cs=iqIcIn;
            idmean1.trans.Angle = rg1.Out+phid;
            MEAN_MACRO(idmean1)

            if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq)>=_IQ(0.75)) && (theta<=_IQ(0.833333333333333))){
                pi1_id.term.Ref = _IQ(0);
                pi1_id.term.Fbk = _IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                pi1_id.param.Ki = _IQmpy(_IQ(0.2368*BASE_CURRENT*T*2),idmean1.data.countertemp);
                PI_MACRO(pi1_id)
                fa1.phiv1=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
            }

        }
    }

    if ((theta>=_IQ(0.25))&&(theta<=_IQ(0.5))){
            if ((theta>=_IQ(0.416666666666667)) && (theta<=_IQ(0.5))){
                idmean1.data.idtemp=_IQ(0);
                idmean1.data.countertemp=_IQ(0);
            }else{
                idmean1.data.sector=_IQ(3);
                idmean1.trans.As=iqIaIn;
                idmean1.trans.Bs=iqIbIn;
                idmean1.trans.Cs=iqIcIn;
                idmean1.trans.Angle = rg1.Out+phid;
                MEAN_MACRO(idmean1)

                if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq)>=_IQ(0.416666666666667)) && (theta<=_IQ(0.5))){
                    pi1_id.term.Ref = _IQ(0);
                    pi1_id.term.Fbk = _IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                    pi1_id.param.Ki = _IQmpy(_IQ(0.2368*BASE_CURRENT*T*2),idmean1.data.countertemp);
                    PI_MACRO(pi1_id)
                    fa1.phiv1=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                }

            }
        }

    if ((theta>=_IQ(0.916666666666667))||(theta<=_IQ(0.166666666666667))){
                if ((theta>=_IQ(0.083333333333333)) && (theta<=_IQ(0.166666666666667))){
                    idmean1.data.idtemp=_IQ(0);
                    idmean1.data.countertemp=_IQ(0);
                }else{
                    idmean1.data.sector=_IQ(1);
                    idmean1.trans.As=iqIaIn;
                    idmean1.trans.Bs=iqIbIn;
                    idmean1.trans.Cs=iqIcIn;
                    idmean1.trans.Angle = rg1.Out+phid;
                    MEAN_MACRO(idmean1)

                    if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq)>=_IQ(0.083333333333333)) && (theta<=_IQ(0.166666666666667))){
                        pi1_id.term.Ref = _IQ(0);
                        pi1_id.term.Fbk = _IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                        pi1_id.param.Ki = _IQmpy(_IQ(0.2368*BASE_CURRENT*T*2),idmean1.data.countertemp);
                        PI_MACRO(pi1_id)
                        fa1.phiv1=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                    }

                }
            }

/*
    if (hall1.Revolutions<5){
        fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }
*/

// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
    theta_ref1= _IQmpy(D_com,_IQ(0.5));
    theta_ref2= D_com-_IQ(0.333333333333333);

    /*theta_temp1=_IQ(0.25)-theta_ref1;
    theta_temp2=_IQ(0.916666666666667)-theta_ref2;*/
    pwmcntl1.Duty1 = _IQ((theta>=_IQ(0.25)-theta_ref1) && (theta<_IQ(0.25)))+_IQmpy(  d_pwm,_IQ(((theta>=0) && ((theta<_IQ(0.25)-theta_ref1))) || ((theta>=_IQ(0.916666666666667)-theta_ref2) && (theta<_IQ(1)))) );
    /*theta_temp1=_IQ(0.75)-theta_ref1;
    theta_temp2=_IQ(0.416666666666667)-theta_ref2;*/
    pwmcntl1.Duty2 = _IQ((theta>=_IQ(0.75)-theta_ref1) && (theta<_IQ(0.75)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.416666666666667)-theta_ref2) && ((theta<_IQ(0.75)-theta_ref1))))  );
    /*if (pwmcntl1.Duty2 >= _IQ(1.0)){
    pwmcntl1.Duty2=pwmcntl1.Duty2-_IQ(0.0000001);
    }*/
    /*pwmcntl1.Duty1 = _IQ(0);
    pwmcntl1.Duty2 = _IQ(0.99999997);*/

    /*theta_temp1=_IQ(0.583333333333333)-theta_ref1;
    theta_temp2=_IQ(0.25)-theta_ref2;*/
    pwmcntl1.Duty3 = _IQ((theta>=_IQ(0.583333333333333)-theta_ref1) && (theta<_IQ(0.583333333333333)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.25)-theta_ref2) && ((theta<_IQ(0.583333333333333)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.083333333333333)+_IQ(1)-theta_ref1;
    theta_temp2=_IQ(0.75)-theta_ref2;*/
    pwmcntl1.Duty4 = _IQ( (theta>=_IQ(0)) && (theta < _IQ(0.083333333333333)) )+_IQ(((theta>=_IQ(0.083333333333333)+_IQ(1)-theta_ref1) && (theta<_IQ(1.0))))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.75)-theta_ref2) && ((theta<_IQ(0.083333333333333)+_IQ(1)-theta_ref1))))  );

    /*theta_temp1=_IQ(0.916666666666667)-theta_ref1;
    theta_temp2=_IQ(0.583333333333333)-theta_ref2;*/
    pwmcntl1.Duty5 = _IQ((theta>=_IQ(0.916666666666667)-theta_ref1) && (theta<_IQ(0.916666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.583333333333333)-theta_ref2) && ((theta<_IQ(0.916666666666667)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.416666666666667)-theta_ref1;
    theta_temp2=_IQ(0.083333333333333)-theta_ref2;*/
    pwmcntl1.Duty6 = _IQ((theta>=_IQ(0.416666666666667)-theta_ref1) && (theta<_IQ(0.416666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.083333333333333)-theta_ref2) && ((theta<_IQ(0.416666666666667)-theta_ref1))) || ((theta>=_IQ(0.083333333333333)-theta_ref2+_IQ(1)) && (theta<_IQ(1)))) );

    PWM_CNTL_MACRO(pwmcntl1)


// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(theta);
    PwmDacCh2 = _IQtoQ15(fa1.phiv);
    PwmDacCh3 = _IQtoQ15(fa1.phiv1);
#if defined(DRV8301) || defined(DRV8302)
    PwmDacCh4 = _IQtoQ15(iqIbIn);
#endif

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(theta);
    DlogCh2 = _IQtoQ15(fa1.phiv);
    DlogCh3 = _IQtoQ15(fa1.phiv1);
    DlogCh4 = _IQtoQ15(iqIbIn);

#endif // (BUILDLEVEL==LEVEL6)

#if (BUILDLEVEL==61)

//    _iq iqIaIn;
//    _iq iqIbIn;
//    _iq iqIcIn;
    _iq marker;



    d_pwm=_IQ(0.7);
    D_com=_IQ(0.333333333333333);

    //0.333333333333333
    //0.388888888888889
    //0.444444444444444
    //0.499999999999999
    phid=_IQ(-0.17);
// ------------------------------------------------------------------------------
//    ADC conversion and offset adjustment
// ------------------------------------------------------------------------------
    iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    iqIcIn = (_IQ15toIQ((AdcResult.ADCRESULT3<<3)-cal_offset_C)<<1);

// ------------------------------------------------------------------------------
//    Connect inputs of the HALL module and call the Hall sensor read macro.
// ------------------------------------------------------------------------------
    HALL3_READ_MACRO(hall1)
    if (hall1.CmtnTrigHall==0x7FFF)
    {
        speed2.TimeStamp = VirtualTimer;
        SPEED_PR_MACRO1(speed2)

        if (hall1.Revolutions<=5){
            speed3.EventPeriod=speed2.EventPeriod_n_1;
            speed3.SUflag=TRUE;
        }else{
            speed3.currenttheta=rg1.Angle;
            speed3.HallGpio=hall1.HallGpioAccepted;
            speed3.EventPeriod=speed2.EventPeriod_n;
            speed3.SUflag=FALSE;
        }

        SE_MACRO1(speed3)
    }

   if (hall1.Revolutions>1000){
            //phid1=_IQ(-0.3242)+_IQdiv(_IQ(0.162),speed3.nextspeed);
            phid1=_IQ(-0.3242)+_IQdiv(_IQ(0.164),speed3.nextspeed);
            phivf.x=_IQsat(phid1, _IQ(-0.05), _IQ(-0.17));
            PHIV_FILTER_MACRO(phivf)
            phid=phivf.y;
            phid = _IQsat(phivf.y, _IQ(-0.05), _IQ(-0.17));
    }
   /*phid=_IQ(-0.156);*/

    rg1.Freq = speed3.nextspeed;
    RG_MACRO(rg1)
    fa1.Angle= rg1.Out;
    if (hall1.Revolutions==-1){
        fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }


    fa1.phiv=fa1.phiv1;
    FA_MACRO(fa1)
    theta=fa1.Out;


// ------------------------------------------------------------------------------
// mean of ids && PI controller
// ------------------------------------------------------------------------------
    if ((theta>=_IQ(0.583333333333333))&&(theta<=_IQ(0.833333333333333))){
        if ((theta>=_IQ(0.75)) && (theta<=_IQ(0.833333333333333))){
            idmean1.data.idtemp=_IQ(0);
            idmean1.data.countertemp=_IQ(0);
        }else{
            idmean1.data.sector=_IQ(5);
            idmean1.trans.As=iqIaIn;
            idmean1.trans.Bs=iqIbIn;
            idmean1.trans.Cs=iqIcIn;
            idmean1.trans.Angle = rg1.Out+phid;
            MEAN_MACRO(idmean1)

            if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq)>=_IQ(0.75)) && (theta<=_IQ(0.833333333333333))){
                pi1_id.term.Ref = _IQ(0);
                pi1_id.term.Fbk = _IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                pi1_id.param.Ki = _IQmpy(_IQ(0.2368*BASE_CURRENT*T*2),idmean1.data.countertemp);
                PI_MACRO(pi1_id)
                fa1.phiv1=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
            }

        }
    }

    if ((theta>=_IQ(0.25))&&(theta<=_IQ(0.5))){
            if ((theta>=_IQ(0.416666666666667)) && (theta<=_IQ(0.5))){
                idmean1.data.idtemp=_IQ(0);
                idmean1.data.countertemp=_IQ(0);
            }else{
                idmean1.data.sector=_IQ(3);
                idmean1.trans.As=iqIaIn;
                idmean1.trans.Bs=iqIbIn;
                idmean1.trans.Cs=iqIcIn;
                idmean1.trans.Angle = rg1.Out+phid;
                MEAN_MACRO(idmean1)

                if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq)>=_IQ(0.416666666666667)) && (theta<=_IQ(0.5))){
                    pi1_id.term.Ref = _IQ(0);
                    pi1_id.term.Fbk = _IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                    pi1_id.param.Ki = _IQmpy(_IQ(0.2368*BASE_CURRENT*T*2),idmean1.data.countertemp);
                    PI_MACRO(pi1_id)
                    fa1.phiv1=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                }

            }
        }

    if ((theta>=_IQ(0.916666666666667))||(theta<=_IQ(0.166666666666667))){
                if ((theta>=_IQ(0.083333333333333)) && (theta<=_IQ(0.166666666666667))){
                    idmean1.data.idtemp=_IQ(0);
                    idmean1.data.countertemp=_IQ(0);
                }else{
                    idmean1.data.sector=_IQ(1);
                    idmean1.trans.As=iqIaIn;
                    idmean1.trans.Bs=iqIbIn;
                    idmean1.trans.Cs=iqIcIn;
                    idmean1.trans.Angle = rg1.Out+phid;
                    MEAN_MACRO(idmean1)

                    if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq)>=_IQ(0.083333333333333)) && (theta<=_IQ(0.166666666666667))){
                        pi1_id.term.Ref = _IQ(0);
                        pi1_id.term.Fbk = _IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                        pi1_id.param.Ki = _IQmpy(_IQ(0.2368*BASE_CURRENT*T*2),idmean1.data.countertemp);
                        PI_MACRO(pi1_id)
                        fa1.phiv1=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                    }

                }
            }

/*
    if (hall1.Revolutions<5){
        fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }
*/

// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
    theta_ref1= _IQmpy(D_com,_IQ(0.5));
    theta_ref2= D_com-_IQ(0.333333333333333);

    /*theta_temp1=_IQ(0.25)-theta_ref1;
    theta_temp2=_IQ(0.916666666666667)-theta_ref2;*/
    pwmcntl1.Duty1 = _IQ((theta>=_IQ(0.25)-theta_ref1) && (theta<_IQ(0.25)))+_IQmpy(  d_pwm,_IQ(((theta>=0) && ((theta<_IQ(0.25)-theta_ref1))) || ((theta>=_IQ(0.916666666666667)-theta_ref2) && (theta<_IQ(1)))) );
    /*theta_temp1=_IQ(0.75)-theta_ref1;
    theta_temp2=_IQ(0.416666666666667)-theta_ref2;*/
    pwmcntl1.Duty2 = _IQ((theta>=_IQ(0.75)-theta_ref1) && (theta<_IQ(0.75)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.416666666666667)-theta_ref2) && ((theta<_IQ(0.75)-theta_ref1))))  );
    /*if (pwmcntl1.Duty2 >= _IQ(1.0)){
    pwmcntl1.Duty2=pwmcntl1.Duty2-_IQ(0.0000001);
    }*/
    /*pwmcntl1.Duty1 = _IQ(0);
    pwmcntl1.Duty2 = _IQ(0.99999997);*/

    /*theta_temp1=_IQ(0.583333333333333)-theta_ref1;
    theta_temp2=_IQ(0.25)-theta_ref2;*/
    pwmcntl1.Duty3 = _IQ((theta>=_IQ(0.583333333333333)-theta_ref1) && (theta<_IQ(0.583333333333333)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.25)-theta_ref2) && ((theta<_IQ(0.583333333333333)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.083333333333333)+_IQ(1)-theta_ref1;
    theta_temp2=_IQ(0.75)-theta_ref2;*/
    pwmcntl1.Duty4 = _IQ( (theta>=_IQ(0)) && (theta < _IQ(0.083333333333333)) )+_IQ(((theta>=_IQ(0.083333333333333)+_IQ(1)-theta_ref1) && (theta<_IQ(1.0))))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.75)-theta_ref2) && ((theta<_IQ(0.083333333333333)+_IQ(1)-theta_ref1))))  );

    /*theta_temp1=_IQ(0.916666666666667)-theta_ref1;
    theta_temp2=_IQ(0.583333333333333)-theta_ref2;*/
    pwmcntl1.Duty5 = _IQ((theta>=_IQ(0.916666666666667)-theta_ref1) && (theta<_IQ(0.916666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.583333333333333)-theta_ref2) && ((theta<_IQ(0.916666666666667)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.416666666666667)-theta_ref1;
    theta_temp2=_IQ(0.083333333333333)-theta_ref2;*/
    pwmcntl1.Duty6 = _IQ((theta>=_IQ(0.416666666666667)-theta_ref1) && (theta<_IQ(0.416666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.083333333333333)-theta_ref2) && ((theta<_IQ(0.416666666666667)-theta_ref1))) || ((theta>=_IQ(0.083333333333333)-theta_ref2+_IQ(1)) && (theta<_IQ(1)))) );

    PWM_CNTL_MACRO(pwmcntl1)


// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(theta);
    PwmDacCh2 = _IQtoQ15(iqIbIn);
    PwmDacCh3 = _IQtoQ15(fa1.phiv1);
#if defined(DRV8301) || defined(DRV8302)
    PwmDacCh4 = _IQtoQ15(fa1.phiv);
#endif

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(theta);
    DlogCh2 = _IQtoQ15(iqIbIn);
    DlogCh3 = _IQtoQ15(fa1.phiv1);
    DlogCh4 = _IQtoQ15(fa1.phiv);

#endif // (BUILDLEVEL==LEVEL61)

//// ------------------------------------------------------------------------------
////    Call the PWMDAC update macro.
//// ------------------------------------------------------------------------------
//    PWMDAC_MACRO(pwmdac1)
//
//// ------------------------------------------------------------------------------
////    Call the DATALOG update function.
//// ------------------------------------------------------------------------------
//    dlog.update(&dlog);
//
//// ------------------------------------------------------------------------------
////    Increase virtual timer and force 15 bit wrap around
//// ------------------------------------------------------------------------------
//    VirtualTimer++;
//    VirtualTimer &= 0x00007FFF;
//
//    }//end if(RunMotor)
//
//
//
//#if (DSP2803x_DEVICE_H==1)
///* Enable more interrupts from this timer
//    EPwm1Regs.ETCLR.bit.INT = 1;
//
//// Acknowledge interrupt to recieve more interrupts from PIE group 3
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
//*/
//
//    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;       // Clear ADCINT1 flag reinitialize for next SOC
//
//// Acknowledge interrupt to recieve more interrupts from PIE group 1
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//
//#endif

#if (DSP280x_DEVICE_H==1)
// Enable more interrupts from this timer
    EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
#endif

// =============================== LEVEL 7 ======================================
// ==============================================================================
#if (BUILDLEVEL==LEVEL7)

    _iq iqIaIn;
    _iq iqIbIn;
    _iq iqIcIn;
    _iq marker;

    timer += _IQ(T*0.05);
    timer = _IQsat(timer,_IQ(1),_IQ(0));

    if (timer<_IQ(0.9)){
        /*d_pwm=_IQ(0.9);
        D_com=_IQ(0.333333333333333);*/
        d_pwm=_IQ(0.5);
        D_com=_IQ(0.388888888888889);
    }else{
        /*d_pwm=_IQ(0.9);
        D_com=_IQ(0.333333333333333);*/
        d_pwm=_IQ(1);
        D_com=_IQ(0.388888888888889);
    }

    D_com=_IQ(0.333333333333333);

    //0.333333333333333
    //0.388888888888889
    //0.444444444444444
    //0.499999999999999
    phid=_IQ(-0.15);
// ------------------------------------------------------------------------------
//    ADC conversion and offset adjustment
// ------------------------------------------------------------------------------
    iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
    iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
    iqIcIn = (_IQ15toIQ((AdcResult.ADCRESULT3<<3)-cal_offset_C)<<1);

// ------------------------------------------------------------------------------
//    Connect inputs of the HALL module and call the Hall sensor read macro.
// ------------------------------------------------------------------------------
    HALL3_READ_MACRO(hall1)
    if (hall1.CmtnTrigHall==0x7FFF)
    {
        speed2.TimeStamp = VirtualTimer;
        SPEED_PR_MACRO1(speed2)

        if (hall1.Revolutions<=5){
            speed3.EventPeriod=speed2.EventPeriod_n_1;
            speed3.SUflag=TRUE;
        }else{
            speed3.currenttheta=rg1.Angle;
            speed3.HallGpio=hall1.HallGpioAccepted;
            speed3.EventPeriod=speed2.EventPeriod_n;
            speed3.SUflag=FALSE;
        }

        SE_MACRO1(speed3)
    }

    if (hall1.Revolutions>500){
        if (timer>_IQ(0.9)){
            /*phid1=_IQ(-0.3242)+_IQdiv(_IQ(0.134),speed3.nextspeed);*/
            phid1=_IQ(-0.3242)+_IQdiv(_IQ(0.134),speed3.nextspeed);
        }else{
            phid1=_IQ(-0.3242)+_IQdiv(_IQ(0.134),speed3.nextspeed);
        }


        phivf.x=_IQsat(phid1, _IQ(-0.05), _IQ(-0.25));
        PHIV_FILTER_MACRO(phivf)
        /*phid=phivf.y;*/
        phid = _IQsat(phivf.y, _IQ(-0.05), _IQ(-0.25));
        //phid = _IQ(-0.14);

        if (timing_mode == 0)
        {
            Uint16 prev_hw_state = speed3.HallGpio; // previous state we were in
            Uint16 curr_hw_state = hall1.HallGpioAccepted; // current state we are in
            
            // use PREVIOUS sector to get current speed proportionality
            int16 elec_duration_d = LUTB_hall_state_elec_duration_digit[prev_hw_state];
            
            // Look up correction angle for the CURRENT state (boundary we just crossed into)
            int16 corr_angle_d;
            if (LutMtpaDir == 1) {
                corr_angle_d = LUTB_corr_angle_positive_direction_digit[curr_hw_state];
            } else {
                corr_angle_d = LUTB_corr_angle_negative_direction_digit[curr_hw_state];
            }
            
            // time elapsed traversing previous sector
            int32 delta_t_hw_now = (int32)speed2.EventPeriod_n;

                 // Calculate the time to wait until next software commutation
                 Uint32 tau_corr = (Uint32)( ( (long long)corr_angle_d * (long long)delta_t_hw_now ) / (long long)elec_duration_d );
                 
                 _iq phase_correction_iq = _IQdiv( _IQ(tau_corr), _IQ(delta_t_hw_now) );

                 
                 // 1 sector is _IQ(0.1666667) (which is 60 electrical degrees ratio over 360 deg)
                 phase_correction_iq = _IQmpy(phase_correction_iq, _IQ(0.1666667));
                 
                 // Apply negative polarity assuming phid relies on it being negative offset 
                 // (based on previous filter phid = [ -0.05, -0.25 ] and phid1 approx logic).
                 phid = -phase_correction_iq; 

                 // Saturate it to keep the system safe just in case
                 phid = _IQsat(phid, _IQ(0.0), _IQ(-0.5));
            }
        }
    }
    /*
    if (hall1.Revolutions>500){
           phid = _IQ(-0.144);
    }*/

    /*phid=_IQ(-0.14);*/

    rg1.Freq = speed3.nextspeed;
    RG_MACRO(rg1)
    fa1.Angle= rg1.Out;
    if (hall1.Revolutions==-1){
        fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }

    if (timing_mode == 0) {
        // LUT Mode: We want instantaneous, cycle-by-cycle phase correction.
        // We bypass FA_FILTER so the sharp step changes in phid aren't washed out!
        fa1.phiv1 = _IQdiv((_IQ(0.5)-D_com),_IQ(2)) + phid;
        fa1.phiv = fa1.phiv1; 
    } else {
        // Online Filter Mode: Keep the legacy low-pass filter active.
        FA_FILTER(fa1)
    }

    FA_MACRO(fa1)
    theta=fa1.Out;


// ------------------------------------------------------------------------------
// mean of ids && PI controller
// ------------------------------------------------------------------------------
     if (hall1.Revolutions>100){
    if ((theta>=_IQ(0.583333333333333))&&(theta<=_IQ(0.833333333333333))){
        if ((theta>=_IQ(0.75)) && (theta<=_IQ(0.833333333333333))){
            idmean1.data.idtemp=_IQ(0);
            idmean1.data.countertemp=_IQ(0);
        }else{
            idmean1.data.sector=_IQ(5);
            idmean1.trans.As=iqIaIn;
            idmean1.trans.Bs=iqIbIn;
            idmean1.trans.Cs=iqIcIn;
            idmean1.trans.Angle = rg1.Out+phid;
            MEAN_MACRO(idmean1)

            if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq)>=_IQ(0.75)) && (theta<=_IQ(0.833333333333333))){
                pi1_id.term.Ref = _IQ(0);
                pi1_id.term.Fbk = _IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                pi1_id.param.Ki = _IQmpy(_IQ(0.2368*BASE_CURRENT*T*2),idmean1.data.countertemp);
                PI_MACRO(pi1_id)
                if (timing_mode != 0) {
                    fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                    //fa1.phiv1=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                }
            }
        }
    }

    if ((theta>=_IQ(0.25))&&(theta<=_IQ(0.5))){
        if ((theta>=_IQ(0.416666666666667)) && (theta<=_IQ(0.5))){
            idmean1.data.idtemp=_IQ(0);
            idmean1.data.countertemp=_IQ(0);
        }else{
            idmean1.data.sector=_IQ(3);
            idmean1.trans.As=iqIaIn;
            idmean1.trans.Bs=iqIbIn;
            idmean1.trans.Cs=iqIcIn;
            idmean1.trans.Angle = rg1.Out+phid;
            MEAN_MACRO(idmean1)

            if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq)>=_IQ(0.416666666666667)) && (theta<=_IQ(0.5))){
                pi1_id.term.Ref = _IQ(0);
                pi1_id.term.Fbk = _IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                pi1_id.param.Ki = _IQmpy(_IQ(0.2368*BASE_CURRENT*T*2),idmean1.data.countertemp);
                PI_MACRO(pi1_id)
                if (timing_mode != 0) {
                    fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                    //fa1.phiv1=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                }
            }
        }
    }

    if ((theta>=_IQ(0.916666666666667))||(theta<=_IQ(0.166666666666667))){
        if ((theta>=_IQ(0.083333333333333)) && (theta<=_IQ(0.166666666666667))){
            idmean1.data.idtemp=_IQ(0);
            idmean1.data.countertemp=_IQ(0);
        }else{
            idmean1.data.sector=_IQ(1);
            idmean1.trans.As=iqIaIn;
            idmean1.trans.Bs=iqIbIn;
            idmean1.trans.Cs=iqIcIn;
            idmean1.trans.Angle = rg1.Out+phid;
            MEAN_MACRO(idmean1)

            if ((theta+_IQmpy(rg1.StepAngleMax,rg1.Freq)>=_IQ(0.083333333333333)) && (theta<=_IQ(0.166666666666667))){
                pi1_id.term.Ref = _IQ(0);
                pi1_id.term.Fbk = _IQdiv(idmean1.data.idtemp,idmean1.data.countertemp);
                pi1_id.param.Ki = _IQmpy(_IQ(0.2368*BASE_CURRENT*T*2),idmean1.data.countertemp);
                PI_MACRO(pi1_id)
                if (timing_mode != 0) {
                    fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                    //fa1.phiv1=-pi1_id.term.Out+_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
                }
            }

        }
    }
    }
    /*
    if (hall1.Revolutions<5){
        Fa1.phiv1=_IQdiv((_IQ(0.5)-D_com),_IQ(2))+phid;
    }
    */

// ------------------------------------------------------------------------------
//    Connect inputs of the PWM_DRV module and call the PWM signal generation
//    update macro.
// ------------------------------------------------------------------------------
    theta_ref1= _IQmpy(D_com,_IQ(0.5));
    theta_ref2= D_com-_IQ(0.333333333333333);

    /*theta_temp1=_IQ(0.25)-theta_ref1;
    theta_temp2=_IQ(0.916666666666667)-theta_ref2;*/
    pwmcntl1.Duty1 = _IQ((theta>=_IQ(0.25)-theta_ref1) && (theta<_IQ(0.25)))+_IQmpy(  d_pwm,_IQ(((theta>=0) && ((theta<_IQ(0.25)-theta_ref1))) || ((theta>=_IQ(0.916666666666667)-theta_ref2) && (theta<_IQ(1)))) );
    /*theta_temp1=_IQ(0.75)-theta_ref1;
    theta_temp2=_IQ(0.416666666666667)-theta_ref2;*/
    pwmcntl1.Duty2 = _IQ((theta>=_IQ(0.75)-theta_ref1) && (theta<_IQ(0.75)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.416666666666667)-theta_ref2) && ((theta<_IQ(0.75)-theta_ref1))))  );
    /*if (pwmcntl1.Duty2 >= _IQ(1.0)){
    pwmcntl1.Duty2=pwmcntl1.Duty2-_IQ(0.0000001);
    }*/
    /*pwmcntl1.Duty1 = _IQ(0);
    pwmcntl1.Duty2 = _IQ(0.99999997);*/

    /*theta_temp1=_IQ(0.583333333333333)-theta_ref1;
    theta_temp2=_IQ(0.25)-theta_ref2;*/
    pwmcntl1.Duty3 = _IQ((theta>=_IQ(0.583333333333333)-theta_ref1) && (theta<_IQ(0.583333333333333)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.25)-theta_ref2) && ((theta<_IQ(0.583333333333333)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.083333333333333)+_IQ(1)-theta_ref1;
    theta_temp2=_IQ(0.75)-theta_ref2;*/
    pwmcntl1.Duty4 = _IQ( (theta>=_IQ(0)) && (theta < _IQ(0.083333333333333)) )+_IQ(((theta>=_IQ(0.083333333333333)+_IQ(1)-theta_ref1) && (theta<_IQ(1.0))))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.75)-theta_ref2) && ((theta<_IQ(0.083333333333333)+_IQ(1)-theta_ref1))))  );

    /*theta_temp1=_IQ(0.916666666666667)-theta_ref1;
    theta_temp2=_IQ(0.583333333333333)-theta_ref2;*/
    pwmcntl1.Duty5 = _IQ((theta>=_IQ(0.916666666666667)-theta_ref1) && (theta<_IQ(0.916666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.583333333333333)-theta_ref2) && ((theta<_IQ(0.916666666666667)-theta_ref1))))  );
    /*theta_temp1=_IQ(0.416666666666667)-theta_ref1;
    theta_temp2=_IQ(0.083333333333333)-theta_ref2;*/
    pwmcntl1.Duty6 = _IQ((theta>=_IQ(0.416666666666667)-theta_ref1) && (theta<_IQ(0.416666666666667)))+_IQmpy(  d_pwm,_IQ(((theta>=_IQ(0.083333333333333)-theta_ref2) && ((theta<_IQ(0.416666666666667)-theta_ref1))) || ((theta>=_IQ(0.083333333333333)-theta_ref2+_IQ(1)) && (theta<_IQ(1)))) );

    PWM_CNTL_MACRO(pwmcntl1)


// ------------------------------------------------------------------------------
//    Connect inputs of the PWMDAC module
// ------------------------------------------------------------------------------
    PwmDacCh1 = _IQtoQ15(theta);
    PwmDacCh2 = _IQtoQ15(pi1_id.term.Out);
    PwmDacCh3 = _IQtoQ15(fa1.phiv1);
#if defined(DRV8301) || defined(DRV8302)
    PwmDacCh4 = _IQtoQ15(iqIbIn);
#endif

// ------------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// ------------------------------------------------------------------------------
    DlogCh1 = _IQtoQ15(theta);
    DlogCh2 = _IQtoQ15(pi1_id.term.Out);
    DlogCh3 = _IQtoQ15(fa1.phiv1);
    DlogCh4 = _IQtoQ15(iqIbIn);

#endif // (BUILDLEVEL==LEVEL7)

    // =============================== LEVEL 8 ======================================
        // PWM Generated, and Hall-sensored control
    // ==============================================================================

    #if (BUILDLEVEL==LEVEL8)

        _iq iqIaIn;
        _iq iqIbIn;
        _iq iqIcIn;
        _iq Te;

        D_com = _IQ(0.3333333333333333);
        d_pwm=_IQ(0.16);
        if (hall1.Revolutions >= 350)
        //D_com = _IQ(0.416666666667);
        //if (hall1.Revolutions >= 750)
        D_com = _IQ(0.333333333333333);


    // ------------------------------------------------------------------------------
    //    ADC conversion and offset adjustment
    // ------------------------------------------------------------------------------
        iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
        iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
        iqIcIn = (_IQ15toIQ((AdcResult.ADCRESULT3<<3)-cal_offset_C)<<1);
        //IDCfdbk= (_IQ15toIQ(AdcResult.ADCRESULT7<<3)-IDC_offset)<<1;

     if (hall1.Revolutions>=300)
          ClosedFlag=TRUE;

    // ------------------------------------------------------------------------------
    //    Connect inputs of the HALL module and call the Hall sensor read macro.
    // ------------------------------------------------------------------------------
        HALL3_READ_MACRO(hall1)
        if (hall1.CmtnTrigHall==0x7FFF)
        {
            speed2.TimeStamp = VirtualTimer;
            SPEED_PR_MACRO1(speed2)
            speed3.Timestamp = VirtualTimer;
            if (hall1.Revolutions<=5){
                speed3.EventPeriod=speed2.EventPeriod_n_1;
                speed3.SUflag=TRUE;
            }else{
                speed3.currenttheta=rg1.Angle;
                speed3.HallGpio=hall1.HallGpioAccepted;
                speed3.EventPeriod=speed2.EventPeriod_n;
                speed3.SUflag=FALSE;
            }

            SE_MACRO1(speed3)
        }

        rg1.Freq = speed3.nextspeed;
        RG_MACRO(rg1)
        fa1.Angle= rg1.Out;   /* motor A 0 */
        fa1.phiv=_IQ(0.0);  /* motor D 48V ~0.9/0.7Nm -0.508 */
        FA_MACRO(fa1)         /* motor C 48V ~0.7Nm -0.05 */
        theta= (fa1.Out);
        //theta_1 = _IQmpy(_IQ(3.6),theta);

        startcurrent.ia = iqIaIn;
        startcurrent.theta = theta;
        START_CURRENT(startcurrent)


        backemf1.theta = theta;
        backemf1.speed = speed3.speed_rads;//_IQdiv(speed3.speed_rads,_IQ(100));//Now,the magnitude of calculated back-emf is 100 times smaller than the actual back-emf!!!
        backemf1.lambda = _IQ(0.037);
        BACKEMF_CAL(backemf1);

        commutation1.speed_rads = speed3.speed_rads;
        commutation1.vdc = _IQ(34);
        commutation1.i_start =  startcurrent.i_start;//_IQ(5.16);
        COMMUTATION_DETEC(commutation1);
        delta = commutation1.delta;
        //delta = _IQmpy(_IQ(3.6),delta);
        delta2 = _IQmpy(_IQ(1.5),delta);

        commutation2.speed_rads =  backemf1.speed; //speed3.speed_rads;
        commutation2.vdc = _IQdiv(_IQ(34),_IQ(100));
//        commutation2.i_start = _IQ(0.05);
        commutation2.lambda = _IQ(0.037);
        commutation2.T = Te;
        COMMUTATION_DUTY(commutation2);
        //d_pwm2 = commutation2.duty;

        commutation =  (theta>=_IQ(0.916666666666667)&&(theta<_IQ(0.916666666666667)+_IQdiv(delta,_IQ(1.5))))*400 + (theta>=_IQ(0.083333333333333)&&(theta<_IQ(0.083333333333333)+delta))*500 + (theta>=_IQ(0.25)&&(theta<_IQ(0.25)+delta))*600 + (theta>=_IQ(0.416666666666667)&&(theta<_IQ(0.416666666666667)+_IQdiv(delta,_IQ(1.0))))*700 + (theta>=_IQ(0.583333333333333)&&(theta<_IQ(0.583333333333333)+delta))*800 + (theta>=_IQ(0.75)&&(theta<_IQ(0.75)+delta))*900;
        commutation_2 =  (theta>=_IQ(0.916666666666667)&&(theta<_IQ(0.916666666666667)+_IQdiv(delta2,_IQ(1.0))))*400 + (theta>=_IQ(0.083333333333333)&&(theta<_IQ(0.083333333333333)+delta2))*500 + (theta>=_IQ(0.25)&&(theta<_IQ(0.25)+delta2))*600 + (theta>=_IQ(0.416666666666667)&&(theta<_IQ(0.416666666666667)+_IQdiv(delta2,_IQ(1.0))))*700 + (theta>=_IQ(0.583333333333333)&&(theta<_IQ(0.583333333333333)+delta2))*800 + (theta>=_IQ(0.75)&&(theta<_IQ(0.75)+delta2))*900;
        //theta_1 = _IQmpy(_IQ(theta>=_IQ(0.916666666666667)&&(theta<_IQ(0.916666666666667)+delta)),_IQ(0.3))+_IQmpy(_IQ(theta>=_IQ(0.083333333333333)&&(theta<_IQ(0.083333333333333)+delta)),_IQ(0.3))+_IQ(0.2);
        currentrecons.ia = iqIaIn;
        currentrecons.ib = iqIbIn;
        currentrecons.ic = iqIcIn;
        currentrecons.commutation = commutation_2;
        Current_recons(currentrecons)


//        torque.ia = _IQmpy(iqIaIn,_IQ(BASE_CURRENT/100)); //Now the calculated current is 100 times smaller than the actual current!!!
//        torque.ib = _IQmpy(iqIbIn,_IQ(BASE_CURRENT/100));
//        torque.ic = _IQmpy(iqIcIn,_IQ(BASE_CURRENT/100));
        torque.ia = _IQmpy(currentrecons.ia_new,_IQ(BASE_CURRENT/100));
        torque.ib = _IQmpy(currentrecons.ib_new,_IQ(BASE_CURRENT/100));
        torque.ic = _IQmpy(currentrecons.ic_new,_IQ(BASE_CURRENT/100));
        torque.ea = backemf1.ea; //The back-emf is also 100 times smaller than actual value
        torque.eb = backemf1.eb;
        torque.ec = backemf1.ec;

        torque.wr = _IQdiv(backemf1.speed,_IQ(POLES/2)); //Mechanical angular speed is used for torque calculation
        TORQUE_CAL(torque);
        Te = torque.Out; //So the calculated torque should also be 100 times smaller!

        Dutycal.theta = theta;
        Dutycal.speed = torque.wr;
        Dutycal.Te = Te;

        Dutycal.ea = backemf1.ea;
        Dutycal.eb = backemf1.eb;
        Dutycal.ec = backemf1.ec;
        Dutycal.vdc = _IQdiv(_IQ(34),_IQ(100));
        DUTY_CAL(Dutycal)

        Dutycal2.Te = Te;

        Dutycal2.Te_ref = _IQmpy(_IQ(0.0065),_IQ(1.00));
        Dutycal2.fup = Dutycal.fup;
        Dutycal2.fdown = Dutycal.fdown;
        Dutycal2.fup2 = commutation2.fup2;
        Dutycal2.fdown2 = commutation2.fdown2;
        DUTY_CAL2(Dutycal2)

        if (hall1.Revolutions>500)
        {
            d_pwm = Dutycal2.Out;
            //d_pwm = _IQ(0.5);
        }

    // ------------------------------------------------------------------------------
    //    Connect inputs of the PWM_DRV module and call the PWM signal generation
    //    update macro.
    // ------------------------------------------------------------------------------

        if (ClosedFlag == FALSE)
        {
                 pwmcntl1.Duty2 =  _IQ(1)-_IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==3));

                 pwmcntl1.Duty1 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==4)),d_pwm);

                 pwmcntl1.Duty4 = _IQ(1)-_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==5));

                 pwmcntl1.Duty3 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==3)),d_pwm);

                 pwmcntl1.Duty6 = _IQ(1)-_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==6));

                 pwmcntl1.Duty5 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==1)),d_pwm);
        }

        else{
            //JK80_BLDC
//            pwmcntl1.Duty4 = _IQ((theta>=_IQ(0.083333333333333))&&(theta<_IQ(0.416666666666667)));
//            pwmcntl1.Duty3 = _IQmpy(d_pwm,_IQ(((theta>=_IQ(0.583333333333333))&&(theta<_IQ(0.916666666666667)))));
//            pwmcntl1.Duty6 = _IQ((((theta>=_IQ(0.75))&&(theta<_IQ(1))))||(theta>=_IQ(0))&&(theta<_IQ(0.083333333333333)));
//            pwmcntl1.Duty5 = _IQmpy(d_pwm,_IQ((theta>=_IQ(0.25))&&(theta<_IQ(0.583333333333333))));
//            pwmcntl1.Duty2 = _IQ((theta>=_IQ(0.416666666666667))&&(theta<_IQ(0.75)));
//            pwmcntl1.Duty1 = _IQmpy(d_pwm,_IQ((((theta>=_IQ(0.916666666666667))&&(theta<_IQ(1))))||(theta>=_IQ(0))&&(theta<_IQ(0.25))));
//
//            pwmcntl1.Duty1 = _IQ(theta>=_IQ(0.916666666666667)&&(theta<(_IQ(0.916666666666667)+delta))) + _IQmpy(d_pwm,_IQ(  (theta>=(_IQ(0.916666666666667)+delta))||(theta<_IQ(0.0833333333333333))||((theta>=(_IQ(0.0833333333333333)+delta))&&(theta<_IQ(0.25)))  )) + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.0833333333333333))&&(theta<(_IQ(0.0833333333333333)+delta))) );
//            pwmcntl1.Duty2 = _IQ( ((theta>=_IQ(0.416666666666667))&&(theta<_IQ(0.583333333333333))) || ((theta>=(_IQ(0.583333333333333)+delta))&&(theta<_IQ(0.75)))  )  + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.583333333333333))&&(theta<(_IQ(0.583333333333333)+delta))) ) ;
//            pwmcntl1.Duty3 = _IQ(theta>=_IQ(0.583333333333333)&&(theta<_IQ(0.583333333333333)+delta)) + _IQmpy(d_pwm,_IQ( ((theta>=_IQ(0.583333333333333)+delta)&&(theta<_IQ(0.75))) || ((theta>=_IQ(0.75)+delta)&&(theta<_IQ(0.916666666666667))) )) + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.75))&&(theta<(_IQ(0.75)+delta))) );
//            pwmcntl1.Duty4 = _IQ( ((theta>=_IQ(0.083333333333333))&&(theta<_IQ(0.25))) || ((theta>=_IQ(0.25)+delta)&&(theta<_IQ(0.416666666666667)))  )  + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.25))&&(theta<(_IQ(0.25)+delta))) ) ;
//            pwmcntl1.Duty5 = _IQ(theta>=_IQ(0.25)&&(theta<_IQ(0.25)+delta)) + _IQmpy(d_pwm,_IQ( ((theta>=_IQ(0.25)+delta)&&(theta<_IQ(0.416666666666667))) || ((theta>=_IQ(0.416666666666667)+delta)&&(theta<_IQ(0.583333333333333))) )) + _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.416666666666667))&&(theta<(_IQ(0.416666666666667)+delta))) );
//            pwmcntl1.Duty6 = _IQ( ((theta>=_IQ(0.75))&&(theta<_IQ(0.916666666666667))) || (theta>=_IQ(0.916666666666667)+delta) || (theta<_IQ(0.083333333333333))  )  +  _IQmpy(d_pwm2,_IQ((theta>=_IQ(0.916666666666667))&&(theta<(_IQ(0.916666666666667)+delta))) );

            PWM1.theta = theta;
            PWM1.d_pwm = d_pwm;
            PWM1.d_pwm2 = d_pwm2;
            PWM1.delta = delta;
            PWM1.x = commutation;
            PWMgen(PWM1)
            pwmcntl1.Duty1 = PWM1.g1;
            pwmcntl1.Duty2 = _IQ(1)-PWM1.g2;
            pwmcntl1.Duty3 = PWM1.g3;
            pwmcntl1.Duty4 = _IQ(1)-PWM1.g4;
            pwmcntl1.Duty5 = PWM1.g5;
            pwmcntl1.Duty6 = _IQ(1)-PWM1.g6;
        }
        PWM_CNTL_MACRO(pwmcntl1)


    if (hall1.Revolutions >= 600)
    {
        if (Observation_data_i >= 600)
           k=0;
       else
           k=1;
    }

    if (k == 1)
        {
        if (Observation_data_a%2 == 1)
        {
        Observation_data_ia[Observation_data_i] = _IQtoQ15(currentrecons.ia_new);
        Observation_data_te[Observation_data_i] = _IQtoQ15(d_pwm);
        Observation_data_i++;
        }
        Observation_data_a++;
        if (Observation_data_i >= 600)
        k = 0;
        }

    // ------------------------------------------------------------------------------
    //    Connect inputs of the PWMDAC module
    // ------------------------------------------------------------------------------
        //PwmDacCh1 =  _IQtoQ15(torque.ea);
        //PwmDacCh1 = _IQtoQ15(Dutycal.Te);
        PwmDacCh1 = (commutation* 30L);
        PwmDacCh2 = _IQtoQ15(theta_1);
        //PwmDacCh4 = (hall1.HallGpioAccepted * 4096.0L);
        //PwmDacCh3 = _IQtoQ15(pi_speed.term.Out);
        PwmDacCh3 = _IQtoQ15(theta);
        //PwmDacCh2 = _IQtoQ15(theta);
        //PwmDacCh3 = (hall1.HallGpioAccepted * 4096.0L);
        #if defined(DRV8301) || defined(DRV8302)
        PwmDacCh4 = (hall1.HallGpioAccepted * 4096.0L);
        #endif

    #endif // (BUILDLEVEL==LEVEL8)


        // =============================== LEVEL 9 ======================================
            // PWM Generated, and Hall-sensored control
        // ==============================================================================

        #if (BUILDLEVEL==LEVEL9)

            _iq iqIaIn;
            _iq iqIbIn;
            _iq iqIcIn;
            _iq Te;

            D_com = _IQ(0.3333333333333333);
            d_pwm=_IQ(0.2);
            if (hall1.Revolutions >= 350)
            D_com = _IQ(0.333333333333333);


        // ------------------------------------------------------------------------------
        //    ADC conversion and offset adjustment
        // ------------------------------------------------------------------------------
            iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
            iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
            iqIcIn = (_IQ15toIQ((AdcResult.ADCRESULT3<<3)-cal_offset_C)<<1);

         if (hall1.Revolutions>=300)
              ClosedFlag=TRUE;

        // ------------------------------------------------------------------------------
        //    Connect inputs of the HALL module and call the Hall sensor read macro.
        // ------------------------------------------------------------------------------
            HALL3_READ_MACRO(hall1)
            if (hall1.CmtnTrigHall==0x7FFF)
            {
                speed2.TimeStamp = VirtualTimer;
                SPEED_PR_MACRO1(speed2)
                speed3.Timestamp = VirtualTimer;
                if (hall1.Revolutions<=5){
                    speed3.EventPeriod=speed2.EventPeriod_n_1;
                    speed3.SUflag=TRUE;
                }else{
                    speed3.currenttheta=rg1.Angle;
                    speed3.HallGpio=hall1.HallGpioAccepted;
                    speed3.EventPeriod=speed2.EventPeriod_n;
                    speed3.SUflag=FALSE;
                }

                SE_MACRO1(speed3)
            }

            rg1.Freq = speed3.nextspeed;
            RG_MACRO(rg1)
            fa1.Angle= rg1.Out;   /* motor A 0 */
            fa1.phiv=_IQ(0.0);  /* motor D 48V ~0.9/0.7Nm -0.508 */
            FA_MACRO(fa1)         /* motor C 48V ~0.7Nm -0.05 */
            theta= (fa1.Out);

            startcurrent.ia = iqIcIn;
            startcurrent.theta = theta;
            START_CURRENT(startcurrent)


            backemf1.theta = theta;
            backemf1.speed = speed3.speed_rads;//_IQdiv(speed3.speed_rads,_IQ(100));//Now,the magnitude of calculated back-emf is 100 times smaller than the actual back-emf!!!
            backemf1.lambda = _IQ(0.037);
            BACKEMF_CAL(backemf1);

            commutation1.speed_rads = speed3.speed_rads;
            commutation1.vdc = _IQ(43);
            commutation1.i_start =  startcurrent.i_start;//_IQ(5.16);
            commutation1.theta = theta;
            COMMUTATION_DETEC(commutation1);
            //delta = _IQmpy(commutation1.delta,_IQ(1.0));
            //delta2 = _IQmpy(_IQ(1.5),delta);

            commutation = commutation1.commutation;

            //commutation =  (theta>=_IQ(0.916666666666667)&&(theta<_IQ(0.916666666666667)+_IQdiv(delta,_IQ(1.65))))*400 + (theta>=_IQ(0.083333333333333)&&(theta<_IQ(0.083333333333333)+delta))*500 + (theta>=_IQ(0.25)&&(theta<_IQ(0.25)+delta))*600 + (theta>=_IQ(0.416666666666667)&&(theta<_IQ(0.416666666666667)+_IQdiv(delta,_IQ(1.0))))*700 + (theta>=_IQ(0.583333333333333)&&(theta<_IQ(0.583333333333333)+delta))*800 + (theta>=_IQ(0.75)&&(theta<_IQ(0.75)+delta))*900;
            //commutation_2 =  (theta>=_IQ(0.916666666666667)&&(theta<_IQ(0.916666666666667)+_IQdiv(delta2,_IQ(1.0))))*400 + (theta>=_IQ(0.083333333333333)&&(theta<_IQ(0.083333333333333)+delta2))*500 + (theta>=_IQ(0.25)&&(theta<_IQ(0.25)+delta2))*600 + (theta>=_IQ(0.416666666666667)&&(theta<_IQ(0.416666666666667)+_IQdiv(delta2,_IQ(1.0))))*700 + (theta>=_IQ(0.583333333333333)&&(theta<_IQ(0.583333333333333)+delta2))*800 + (theta>=_IQ(0.75)&&(theta<_IQ(0.75)+delta2))*900;

            currentrecons.ia = iqIaIn;
            currentrecons.ib = iqIbIn;
            currentrecons.ic = iqIcIn;
            currentrecons.commutation = commutation1.commutation_2;
            Current_recons(currentrecons)

            torque.ia = _IQmpy(currentrecons.ia_new,_IQ(BASE_CURRENT/100));
            torque.ib = _IQmpy(currentrecons.ib_new,_IQ(BASE_CURRENT/100));
            torque.ic = _IQmpy(currentrecons.ic_new,_IQ(BASE_CURRENT/100));
            torque.ea = backemf1.ea; //The back-emf is also 100 times smaller than actual value
            torque.eb = backemf1.eb;
            torque.ec = backemf1.ec;

            torque.wr = _IQdiv(backemf1.speed,_IQ(POLES/2)); //Mechanical angular speed is used for torque calculation
            TORQUE_CAL(torque);
            Te = torque.Out; //So the calculated torque should also be 100 times smaller!

            Dutycal.theta = theta;
            Dutycal.speed = torque.wr;
            Dutycal.Te = Te;
            Dutycal.ea = backemf1.ea;
            Dutycal.eb = backemf1.eb;
            Dutycal.ec = backemf1.ec;
            Dutycal.vdc = _IQdiv(_IQ(43),_IQ(100));
            DUTY_CAL(Dutycal)

            commutation2.speed_rads =  backemf1.speed; //speed3.speed_rads;
            commutation2.vdc = _IQdiv(_IQ(43),_IQ(100));
            commutation2.lambda = _IQ(0.037);
            commutation2.commutation = commutation;
            commutation2.ea = backemf1.ea;
            commutation2.eb = backemf1.eb;
            commutation2.ec = backemf1.ec;
            commutation2.test1 = Dutycal.test1;
            COMMUTATION_DUTY(commutation2)
            //d_pwm2 = commutation2.duty;

            Dutycal2.Te = Te;
            Dutycal2.Te_ref = _IQ(0.007);
            Dutycal2.commutation = commutation;
            Dutycal2.fup = Dutycal.fup;
            Dutycal2.fdown = Dutycal.fdown;
            Dutycal2.fup2 = commutation2.fup2;
            Dutycal2.fdown2 = commutation2.fdown2;
            DUTY_CAL2(Dutycal2)

            if (hall1.Revolutions>500)
            {
                d_pwm = Dutycal2.Out;
            }

        // ------------------------------------------------------------------------------
        //    Connect inputs of the PWM_DRV module and call the PWM signal generation
        //    update macro.
        // ------------------------------------------------------------------------------

            if (ClosedFlag == FALSE)
            {
                     pwmcntl1.Duty2 =  _IQ(1)-_IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==3));

                     pwmcntl1.Duty1 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==4)),d_pwm);

                     pwmcntl1.Duty4 = _IQ(1)-_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==5));

                     pwmcntl1.Duty3 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==3)),d_pwm);

                     pwmcntl1.Duty6 = _IQ(1)-_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==6));

                     pwmcntl1.Duty5 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==1)),d_pwm);
            }

            else{
                PWM1.theta = theta;
                PWM1.d_pwm = d_pwm;
                PWM1.delta = delta;
                PWM1.x = commutation;
                PWMgen(PWM1)
                pwmcntl1.Duty1 = PWM1.g1;
                pwmcntl1.Duty2 = _IQ(1)-PWM1.g2;
                pwmcntl1.Duty3 = PWM1.g3;
                pwmcntl1.Duty4 = _IQ(1)-PWM1.g4;
                pwmcntl1.Duty5 = PWM1.g5;
                pwmcntl1.Duty6 = _IQ(1)-PWM1.g6;
            }
            PWM_CNTL_MACRO(pwmcntl1)


        if (hall1.Revolutions >= 800)
        {
            if (Observation_data_i >= 600)
               k=0;
           else
               k=1;
        }

        if (k == 1)
            {
            if (Observation_data_a%6 == 1)
            {
            Observation_data_ia[Observation_data_i] = _IQtoQ15(currentrecons.ic_new);
            Observation_data_te[Observation_data_i] = _IQtoQ15(d_pwm);
            Observation_data_i++;
            }
            Observation_data_a++;
            if (Observation_data_i >= 600)
            k = 0;
            }

            //PwmDacCh1 = _IQtoQ15(commutation2.fdown2);

            PwmDacCh2 = _IQtoQ15(torque.test);
            PwmDacCh3 = _IQtoQ15(d_pwm);
            #if defined(DRV8301) || defined(DRV8302)
            PwmDacCh4 = (hall1.HallGpioAccepted * 4096.0L);
            #endif

        #endif // (BUILDLEVEL==LEVEL9)


            // =============================== LEVEL 10 ======================================
                // PWM Generated, and Hall-sensored control
            // ==============================================================================

              #if (BUILDLEVEL==LEVEL10)

                _iq iqIaIn;
                _iq iqIbIn;
                _iq iqIcIn;
                _iq Te;

                ad = speed3.ad1;
                //ad = _IQ(0.0);
                D_com = _IQ(0.3333333333333333);
                //d_pwm=_IQ(0.2);
                d_pwm=_IQ(0.6);
                if (hall1.Revolutions >= 350)
                D_com = _IQ(0.333333333333333);

            // ------------------------------------------------------------------------------
            //    ADC conversion and offset adjustment
            // ------------------------------------------------------------------------------
                iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
                iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
                iqIcIn = (_IQ15toIQ((AdcResult.ADCRESULT3<<3)-cal_offset_C)<<1);

             if (hall1.Revolutions>=300)
                  ClosedFlag=FALSE;

            // ------------------------------------------------------------------------------
            //    Connect inputs of the HALL module and call the Hall sensor read macro.
            // ------------------------------------------------------------------------------
                HALL3_READ_MACRO(hall1)
                if (hall1.CmtnTrigHall==0x7FFF)
                {
                    speed2.TimeStamp = VirtualTimer;
                    SPEED_PR_MACRO1(speed2)
                    speed3.Timestamp = VirtualTimer;
                    if (hall1.Revolutions<=5){
                        speed3.EventPeriod=speed2.EventPeriod_n_1;
                        speed3.SUflag=TRUE;
                    }else{
                        speed3.currenttheta=rg1.Angle;
                        speed3.HallGpio=hall1.HallGpioAccepted;
                        speed3.EventPeriod=speed2.EventPeriod_n;
                        speed3.SUflag=FALSE;
                    }

                      SE_MACRO1(speed3)
                }

                rg1.Freq = speed3.nextspeed;
                RG_MACRO(rg1)
//                fa1.Angle= rg1.Out;   /* motor A 0 */
//                fa1.phiv=_IQ(0.0);  /* motor D 48V ~0.9/0.7Nm -0.508 */
//                FA_MACRO(fa1)         /* motor C 48V ~0.7Nm -0.05 */
                theta= rg1.Out;
                theta_1 = theta-ad;

                startcurrent.ia = iqIaIn;
                startcurrent.theta = theta_1;
                START_CURRENT(startcurrent)


                backemf1.theta = theta;
                backemf1.speed = speed3.speed_rads;//_IQdiv(speed3.speed_rads,_IQ(100));//Now,the magnitude of calculated back-emf is 100 times smaller than the actual back-emf!!!
                backemf1.lambda = _IQ(0.037);
                BACKEMF_CAL(backemf1);

                ThreePhase.theta = theta;
                THREEPHASE(ThreePhase)

                commutation1.speed_rads = speed3.speed_rads;
                commutation1.vdc = _IQ(58);
                commutation1.i_start =  startcurrent.i_start;//_IQ(5.16);
                commutation1.theta = theta_1;
                COMMUTATION_DETEC(commutation1);
                commutation =  commutation1.commutation;
                //commutation = 0;


//                commutation3.theta = theta_1;
//                commutation3.delta2 = commutation1.delta2;
//                COMMUTATION_DETEC2(commutation3)

                currentrecons.ia = iqIaIn;
                currentrecons.ib = iqIbIn;
                currentrecons.ic = iqIcIn;
                currentrecons.commutation = commutation1.commutation_2;
                Current_recons(currentrecons)

                torque.ia = _IQmpy(currentrecons.ia_new,_IQ(BASE_CURRENT/100));
                torque.ib = _IQmpy(currentrecons.ib_new,_IQ(BASE_CURRENT/100));
                torque.ic = _IQmpy(currentrecons.ic_new,_IQ(BASE_CURRENT/100));
                torque.ea = backemf1.ea; //The back-emf is also 100 times smaller than actual value
                torque.eb = backemf1.eb;
                torque.ec = backemf1.ec;
                torque.wr = _IQdiv(backemf1.speed,_IQ(2));//_IQdiv(backemf1.speed,_IQ(POLES/2)); //Mechanical angular speed is used for torque calculation
                TORQUE_CAL(torque);
                Te = torque.Out; //So the calculated torque should also be 100 times smaller!
             if (hall1.Revolutions>2600)
             {
                  //commutation = commutation1.commutation;
                  //torque.test = _IQsat(torque.test,_IQ(0.8),_IQ(0.65));
              }

                //Dutycal.theta = theta;
                Dutycal.speed = torque.wr;
                Dutycal.Te = Te;
                //Dutycal.ea = backemf1.ea;
                //Dutycal.eb = backemf1.eb;
                //Dutycal.ec = backemf1.ec;
                Dutycal.TP = ThreePhase.TP;
                Dutycal.ex = backemf1.ex;
                Dutycal.vdc = _IQ(0.58); //_IQdiv(_IQ(43),_IQ(100));
                DUTY_CAL(Dutycal)

                commutation2.speed_rads =  speed3.speed_rads;
                commutation2.vdc =  _IQ(0.58); //_IQdiv(_IQ(43),_IQ(100));
                commutation2.lambda = _IQ(0.037);
                commutation2.commutation = commutation;
                commutation2.ea = backemf1.ea;
                commutation2.eb = backemf1.eb;
                commutation2.ec = backemf1.ec;
                commutation2.test1 = Dutycal.test1;
                COMMUTATION_DUTY(commutation2)
                //d_pwm2 = commutation2.duty;

                Dutycal2.Te = Te;
                Dutycal2.Te_ref = _IQ(0.003);
                if (hall1.Revolutions>2900)
                { Dutycal2.Te_ref = _IQ(0.007);
                /*torque.test = _IQsat(torque.test,_IQ(0.8),_IQ(0.01))*/;}
                if (hall1.Revolutions>3600)
                { Dutycal2.Te_ref = _IQ(0.007);
                /*torque.test = _IQsat(torque.test,_IQ(0.8),_IQ(0.01))*/; }
                if (hall1.Revolutions>5100)
                { Dutycal2.Te_ref = _IQ(0.007);
                torque.test = _IQsat(torque.test,_IQ(0.8),_IQ(0.64));}

                Dutycal2.commutation = commutation;
                Dutycal2.fup = Dutycal.fup;
                Dutycal2.fdown = Dutycal.fdown;
                Dutycal2.fup2 = commutation2.fup2;
                Dutycal2.fdown2 = commutation2.fdown2;
                DUTY_CAL2(Dutycal2)



                if (hall1.Revolutions>2600)
                {
                   // d_pwm = Dutycal2.Out;
                    d_pwm=_IQ(0.6);
                }


            // ------------------------------------------------------------------------------
            //    Connect inputs of the PWM_DRV module and call the PWM signal generation
            //    update macro.
            // ------------------------------------------------------------------------------

                if (ClosedFlag == FALSE)
                {
#if (MOTOR_SELECT==MOTOR_A || MOTOR_SELECT == MOTOR_C)
        // PHASE A:
        // High side:
        pwmcntl1.Duty1 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==3)||(hall1.HallGpioAccepted==1)),d_pwm);
        // Low side:
        pwmcntl1.Duty2 = _IQ(1)- _IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==6));

        // PHASE B:
        // High side:
        pwmcntl1.Duty3 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==5)),d_pwm);
        // Low side:
        pwmcntl1.Duty4 = _IQ(1)-_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==3));

        // PHASE C:
        // High side:
        pwmcntl1.Duty5 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==2)),d_pwm);
        // Low side:
        pwmcntl1.Duty6 = _IQ(1)-_IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==5));
        #endif

        #if (MOTOR_SELECT==MOTOR_D)
        // PHASE A:
        // Low side:
        pwmcntl1.Duty2 = _IQ(1)- _IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==3));
        // High side:
        pwmcntl1.Duty1 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==4)),d_pwm);

        // PHASE B:
        // Low side:
        pwmcntl1.Duty4 = _IQ(1)-_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==5));
        // High side:
        pwmcntl1.Duty3 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==3)),d_pwm);

        // PHASE C:
        // Low side:
        pwmcntl1.Duty6 = _IQ(1)-_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==6));
        // High side:
        pwmcntl1.Duty5 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==1)),d_pwm);
        #endif
                 }

                else{
                    PWM1.theta = theta;
                    PWM1.d_pwm = d_pwm;
                    PWM1.delta = delta;
                    PWM1.x = commutation;
                    PWMgen(PWM1)
                    pwmcntl1.Duty1 = PWM1.g1;
                    pwmcntl1.Duty2 = _IQ(1)-PWM1.g2;
                    pwmcntl1.Duty3 = PWM1.g3;
                    pwmcntl1.Duty4 = _IQ(1)-PWM1.g4;
                    pwmcntl1.Duty5 = PWM1.g5;
                    pwmcntl1.Duty6 = _IQ(1)-PWM1.g6;
                }
                 PWM_CNTL_MACRO(pwmcntl1)


//             if (hall1.Revolutions >= 1900)
//            {
//                if (Observation_data_i >= 600)
//                   k=0;
//                else
//                   k=1;
//            }
//
//            if (k == 1)
//                {
//                if (Observation_data_a%6 == 1)
//                {
//                Observation_data_ia[Observation_data_i] = _IQtoQ15(torque.test);
//                Observation_data_te[Observation_data_i] = _IQtoQ15(d_pwm);
//                Observation_data_i++;
//                }
//                Observation_data_a++;
//                if (Observation_data_i >= 600)
//                k = 0;
//                }

                //PwmDacCh1 = _IQtoQ15(backemf1.ea);

                PwmDacCh2 = _IQtoQ15(torque.test);
                //PwmDacCh2 = commutation*10L;//_IQtoQ15(d_pwm);
                 PwmDacCh1 = _IQtoQ15(iqIaIn);
                //#if defined(DRV8301) || defined(DRV8302)
                //PwmDacCh4 = (ThreePhase.TP*4000L);
                //#endif

            #endif // (BUILDLEVEL==LEVEL10)


                 // =============================== LEVEL 11 ======================================
                    // PWM Generated, and Hall-sensored control  (Shi_qinna)
                // ==============================================================================

                #if (BUILDLEVEL==LEVEL11)

//                    _iq iqIaIn;
//                    _iq iqIbIn;
//                    _iq iqIcIn;
                    _iq Te;

                    //d_pwm=_IQ(0.2);
//                    d_pwm=_IQ(0);
//                    d_pwm=_IQ(0.0);
//                    if (hall1.Revolutions >= 350)
                    D_com = _IQ(0.333333333333333);



                // ------------------------------------------------------------------------------
                //    ADC conversion and offset adjustment
                // ------------------------------------------------------------------------------
                    iqIaIn = (_IQ15toIQ((AdcResult.ADCRESULT1<<3)-cal_offset_A)<<1);
                    iqIbIn = (_IQ15toIQ((AdcResult.ADCRESULT2<<3)-cal_offset_B)<<1);
                    iqIcIn = (_IQ15toIQ((AdcResult.ADCRESULT3<<3)-cal_offset_C)<<1);
//                    iqIaIn = (_IQ12toIQ((AdcResult.ADCRESULT1))<<1);
//                    iqIbIn = (_IQ12toIQ((AdcResult.ADCRESULT2))<<1);
//                    iqIcIn = (_IQ12toIQ((AdcResult.ADCRESULT3))<<1);

//                 if (hall1.Revolutions>=500) {
//                      ClosedFlag=TRUE;
//                 }

                // ------------------------------------------------------------------------------
                //    Connect inputs of the HALL module and call the Hall sensor read macro.
                // ------------------------------------------------------------------------------
                    
                    HALL3_READ_MACRO(hall1)
                    if (hall1.CmtnTrigHall==0x7FFF)
                    {
                        speed2.TimeStamp = VirtualTimer;
                        SPEED_PR_MACRO1(speed2)
                        speed3.Timestamp = VirtualTimer;
                        if (hall1.Revolutions<=5){
                            speed3.EventPeriod=speed2.EventPeriod_n_1;
                            speed3.SUflag=TRUE;
                        }else{
                            speed3.currenttheta=rg1.Angle;
                            speed3.HallGpio=hall1.HallGpioAccepted;
                            speed3.EventPeriod=speed2.EventPeriod_n;
                            speed3.SUflag=FALSE;
                        }
                        SE_MACRO1(speed3)

                        
                        Uint16 curr_hall = hall1.HallGpioAccepted;

                        const Uint16 PREV_HALL_MAP[7] = {0, 5, 3, 1, 6, 4, 2};

                        // const Uint16 my_hall_to_mark_hall_conversion[7] = {0, 1, 4, 5, 2, 3, 6};
                        // Uint16 mark_equiv_curr_hall = my_hall_to_mark_hall_conversion[curr_hall];
                        // curr_hall = mark_equiv_curr_hall;

                        // const Uint16 MARK_PREV_HALL_MAP[7] = {0, 5, 3, 1, 6, 4, 2};
                        // // Grab the previous hall directly from the map
                        // Uint16 prev_hall = MARK_PREV_HALL_MAP[curr_hall];
                         
                        // // Grab the previous hall directly from the map
                        Uint16 prev_hall = PREV_HALL_MAP[curr_hall];
                        
                        
                        
                        // int32 EventPeriod_LUTB = (int32)( (LUTB_corr_angle[curr_hall] * speed2.EventPeriod_n_1) / LUTB_hall_state_elec_duration_digit[prev_hall] ); // I need long long (gives me better result)
                        int32 EventPeriod_LUTB = (int32)( ((long long)LUTB_corr_angle[curr_hall] * (long long)speed2.EventPeriod_n_1) / (long long)LUTB_hall_state_elec_duration_digit[prev_hall] );
                        // _iq dtheta_LUTB = speed3.nexttheta + speed3.phic - speed3.currenttheta;  //This produces negative number sometimes so i just use speed3.dtheta           
                        
                        nextspeed_LUTB = _IQdiv(speed3.speedscaler, EventPeriod_LUTB); 
                        // nextspeed_LUTB = _IQmpy(nextspeed_LUTB, dtheta_LUTB);                                       
                        nextspeed_LUTB = _IQmpy(nextspeed_LUTB, speed3.dtheta);
                    }

                    // if using online filter
                    // {

                    if (timing_mode == 1)
                    rg1.Freq = speed3.nextspeed;
                    //}

                    else if (timing_mode == 0)
                    rg1.Freq = nextspeed_LUTB;
                    
                    
                    RG_MACRO(rg1)
    //                fa1.Angle= rg1.Out;   /* motor A 0 */
    //                fa1.phiv=_IQ(0.0);  /* motor D 48V ~0.9/0.7Nm -0.508 */
    //                FA_MACRO(fa1)         /* motor C 48V ~0.7Nm -0.05 */
                    theta= rg1.Out;


                    startcurrent.ia = iqIaIn;
                    startcurrent.theta = theta;
                    START_CURRENT(startcurrent)


                    backemf1.theta = theta;
                    backemf1.speed = speed3.speed_rads;//_IQdiv(speed3.speed_rads,_IQ(100));//Now,the magnitude of calculated back-emf is 100 times smaller than the actual back-emf!!!
                    backemf1.lambda = _IQ(0.0215);
                    BACKEMF_CAL(backemf1);

                    ThreePhase.theta = theta;
                    THREEPHASE(ThreePhase)

                    commutation3.speed_rads = speed3.speed_rads;
                    commutation3.vdc = _IQ(48);
                    commutation3.i_start =  startcurrent.i_start;//_IQ(5.16);
                    commutation3.theta = theta;
                    COMMUTATION_DETEC2(commutation3);
                    commutation = commutation3.commutation;

                    currentrecons.ia = iqIaIn;
                    currentrecons.ib = iqIbIn;
                    currentrecons.ic = iqIcIn;
                    currentrecons.commutation = commutation3.commutation_2;
                    Current_recons(currentrecons)

                    torque.ia = _IQmpy(currentrecons.ia_new,_IQ(BASE_CURRENT/100));
                    torque.ib = _IQmpy(currentrecons.ib_new,_IQ(BASE_CURRENT/100));
                    torque.ic = _IQmpy(currentrecons.ic_new,_IQ(BASE_CURRENT/100));
                    torque.ea = backemf1.ea; //The back-emf is also 100 times smaller than actual value
                    torque.eb = backemf1.eb;
                    torque.ec = backemf1.ec;
                    torque.wr = _IQdiv(backemf1.speed,_IQ(POLES/2)); //Mechanical angular speed is used for torque calculation
                    TORQUE_CAL(torque);
                    Te = torque.Out; // So the calculated torque should also be 100 times smaller!

                    Dutycal.speed = torque.wr;
                    Dutycal.Te = Te;
                    Dutycal.TP = ThreePhase.TP;
                    Dutycal.ex = backemf1.ex;
                    Dutycal.vdc = _IQdiv(_IQ(48),_IQ(100));
                    DUTY_CAL(Dutycal)

                    commutation4.speed_rads =  speed3.speed_rads;
                    commutation4.vdc = _IQdiv(_IQ(48),_IQ(100));
                    commutation4.lambda = _IQ(0.037);
                    commutation4.commutation = commutation;
                    commutation4.ea = backemf1.ea;
                    commutation4.eb = backemf1.eb;
                    commutation4.ec = backemf1.ec;
                    commutation4.test1 = Dutycal.test1;
                    COMMUTATION_DUTYShi(commutation4)
                    //d_pwm2 = commutation2.duty;

                    Dutycal2.Te = Te;
                    Dutycal2.Te_ref = _IQ(0.007);
                    Dutycal2.commutation = commutation;
                    Dutycal2.fup = Dutycal.fup;
                    Dutycal2.fdown = Dutycal.fdown;
                    Dutycal2.fup2 = commutation4.fup2;
                    Dutycal2.fdown2 = commutation4.fdown2;
                    DUTY_CAL2(Dutycal2)

                    // ------------------------------------------------------------------------------
                    // mean of ids && PI controller
                    // ------------------------------------------------------------------------------
//                    if (hall1.Revolutions>100){
                        idmean1.data.sector=_IQ(hall1.HallGpioAccepted);
                        idmean1.trans.As=iqIaIn;
                        idmean1.trans.Bs=iqIbIn;
                        idmean1.trans.Cs=iqIcIn;
                        idmean1.trans.Angle = theta;
                        MEAN_MACRO(idmean1)

                        /*When reaching a (software) state transition, calculate and update the average id and restart the counter*/
//                        if (idmean1.data.sector != idmean1.data.prev_sector){

//                        if (idmean1.data.sector != idmean1.data.prev_sector){
                            // Calculate and update the mean value
//                            float one_over_N = 1.0f/idmean1.data.countertemp;
//                            count+=1;
//                            if (ids_avg_cal_enable && hall1.HallGpioAccepted==1){
                            if (idmean1.data.sector != idmean1.data.prev_sector){
                                // Calculate average i_ds
                                idmean1.data.ids = _IQ(_IQtoF(idmean1.data.idtemp)/idmean1.data.countertemp);
                                MEAN_FILTER(idmean1)

                                // Reset accumulator state
                                idmean1.data.prev_sector = idmean1.data.sector;
                                idmean1.data.idtemp = _IQ(0);
                                idmean1.data.countertemp = 0;
                                ids_avg_cal_enable = FALSE;

                                
                                if (MTPAFlag == TRUE){
                                    pi1_id.term.Ref = _IQ(0);
                                    pi1_id.term.Fbk = idmean1.data.ids;
                                    PI_MACRO(pi1_id)
                                    delta_phiv = pi1_id.term.Out;
                                    delta_phiv_storage[0] = delta_phiv_storage[1];
                                    delta_phiv_storage[1] = delta_phiv_storage[2];
                                    delta_phiv_storage[2] = delta_phiv_storage[3];
                                    delta_phiv_storage[3] = delta_phiv_storage[4];
                                    delta_phiv_storage[4] = delta_phiv_storage[5];
                                    delta_phiv_storage[5] = delta_phiv;
                                    delta_phiv_filtered = _IQdiv(delta_phiv_storage[0]+delta_phiv_storage[1]+delta_phiv_storage[2]+delta_phiv_storage[3]+delta_phiv_storage[4]+delta_phiv_storage[5], _IQ(6));
                                } else {
                                    delta_phiv = _IQ(0);
                                    RESET_PI_CTRL(pi1_id);
                                }

                            }
//                            if (hall1.HallGpioAccepted==4){
//                                ids_avg_cal_enable = TRUE;
//                            }
//                        }


                // ------------------------------------------------------------------------------
                //    Connect inputs of the PWM_DRV module and call the PWM signal generation
                //    update macro.
                // ------------------------------------------------------------------------------

                    if (ClosedFlag == FALSE)
                    {
                        #if (MOTOR_SELECT==MOTOR_A || MOTOR_SELECT == MOTOR_C)
                        // PHASE A:
                        // High side:
                        pwmcntl1.Duty1 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==3)||(hall1.HallGpioAccepted==1)),d_pwm);
                        // Low side:
                        pwmcntl1.Duty2 = _IQ(1)- _IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==6));

                        // PHASE B:
                        // High side:
                        pwmcntl1.Duty3 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==5)),d_pwm);
                        // Low side:
                        pwmcntl1.Duty4 = _IQ(1)-_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==3));

                        // PHASE C:
                        // High side:
                        pwmcntl1.Duty5 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==2)),d_pwm);
                        // Low side:
                        pwmcntl1.Duty6 = _IQ(1)-_IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==5));
                        #endif

                        #if (MOTOR_SELECT==MOTOR_D)
                        // PHASE A:
                        // Low side:
                        pwmcntl1.Duty2 = _IQ(1)- _IQ((hall1.HallGpioAccepted==1)||(hall1.HallGpioAccepted==3));
                        // High side:
                        pwmcntl1.Duty1 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==6)||(hall1.HallGpioAccepted==4)),d_pwm);

                        // PHASE B:
                        // Low side:
                        pwmcntl1.Duty4 = _IQ(1)-_IQ((hall1.HallGpioAccepted==4)||(hall1.HallGpioAccepted==5));
                        // High side:
                        pwmcntl1.Duty3 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==3)),d_pwm);

                        // PHASE C:
                        // Low side:
                        pwmcntl1.Duty6 = _IQ(1)-_IQ((hall1.HallGpioAccepted==2)||(hall1.HallGpioAccepted==6));
                        // High side:
                        pwmcntl1.Duty5 = /*_IQ(1)-*/_IQmpy(_IQ((hall1.HallGpioAccepted==5)||(hall1.HallGpioAccepted==1)),d_pwm);
                        #endif
                    }

                    else{
                        PWM2.theta = theta;
                        PWM2.theta_prime = theta + delta_phiv;
                        if (PWM2.theta_prime >= _IQ(1.0))
                            PWM2.theta_prime -= _IQ(1.0);
                        else if (PWM2.theta_prime < _IQ(0.0))
                            PWM2.theta_prime += _IQ(1.0);
                        PWM2.d_pwm = d_pwm;
                        PWM2.delta = delta;
                        PWM2.x = commutation;
                        PWMgen(PWM2)
                        pwmcntl1.Duty1 = PWM2.g1;
                        pwmcntl1.Duty2 = _IQ(1)-PWM2.g2;
                        pwmcntl1.Duty3 = PWM2.g3;
                        pwmcntl1.Duty4 = _IQ(1)-PWM2.g4;
                        pwmcntl1.Duty5 = PWM2.g5;
                        pwmcntl1.Duty6 = _IQ(1)-PWM2.g6;
                        }
                    PWM_CNTL_MACRO(pwmcntl1)
//                    DlogCh1 = _IQtoQ15(hall1.Revolutions * 10000);
//                    DlogCh1 = 2000;
//                    PwmDacCh1 = hall1.HallGpioAccepted*5000;
//                    int16 ids_compressed = (int16)(idmean1.trans.Ds >> 16);
                    ids_float = _IQ24toF(idmean1.trans.Ds);
//                    PwmDacCh1 = (int)((ids_float+1)*32768);
//                    DlogCh1 = _IQtoQ15(iqIaIn);
//                    PwmDacCh1 = (int)(_IQ24toF(rg1.Out)*5000); ---> angle
                    PwmDacCh1 = (int)(_IQ24toF(idmean1.trans.Ds)*50000+15000);
//                    PwmDacCh1 = (int)((_IQ24toF(iqIbIn)+1)*15000);
//                    PwmDacCh1 = (int)((_IQ24toF(torque.ec)+1)*15000);
//                    PwmDacCh2 = (int)(_IQ24toF(rg1.Out)*5000); //---> angle
                    PwmDacCh2 = (int)(_IQ24toF(idmean1.data.ids)*50000+15000);
                    PwmDacCh3 = (hall1.HallGpioAccepted * 4096.0L);
//                    DlogCh1 = _IQtoQ15(theta);
//                    DlogCh1 = _IQtoQ15(idmean1.trans.Ds);
//                    DlogCh1 = _IQtoQ15(_IQmpy(iqIaIn,_IQ(BASE_CURRENT)));
//                    DlogCh1 = _IQtoQ15(_IQmpy(iqIaIn, _IQ(1)));
                    DlogCh1 = _IQtoQ15(idmean1.data.ids);


                #endif // (BUILDLEVEL==LEVEL11)


// ------------------------------------------------------------------------------
//    Call the PWMDAC update macro.
// ------------------------------------------------------------------------------
	PWMDAC_MACRO(pwmdac1)

// ------------------------------------------------------------------------------
//    Call the DATALOG update function.
// ------------------------------------------------------------------------------
    dlog.update(&dlog);

// ------------------------------------------------------------------------------
//    Increase virtual timer and force 15 bit wrap around
// ------------------------------------------------------------------------------
	VirtualTimer++;
	VirtualTimer &= 0x00007FFF;
   
	}//end if(RunMotor)
   

   

#if (DSP2803x_DEVICE_H==1)
/* Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
*/

	AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		// Clear ADCINT1 flag reinitialize for next SOC

// Acknowledge interrupt to recieve more interrupts from PIE group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

#endif

#if (DSP280x_DEVICE_H==1)
// Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
#endif

}
// ISR Ends Here


//===========================================================================
// No more.
//===========================================================================
