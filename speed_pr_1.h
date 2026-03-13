/*
 * speed_pr_1.h
 *
 *  Created on: Feb 10, 2022
 *      Author: LabUser
 */

#ifndef SPEED_PR_1_H_
#define SPEED_PR_1_H_
/* =================================================================================
File name:        SPEED_PR.H  (IQ version)

Originator: Digital Control Systems Group
            Texas Instruments

Description:
    Header file containing the data types, constants for the period based
    speed measurement  and macro definitions for the SPEED_PR.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2009 Version 1.0
------------------------------------------------------------------------------*/
// For event period estimation (6th-order filter)

typedef struct {
       Uint32 NewTimeStamp;     // Variable : New 'Timestamp' corresponding to a capture event (Q0) - independently with global Q
       Uint32 OldTimeStamp;     // Variable : Old 'Timestamp' corresponding to a capture event (Q0) - independently with global Q
       Uint32 TimeStamp;        // Input : Current 'Timestamp' corresponding to a capture event (Q0) - independently with global Q
       int32 EventPeriod_n;     // Input/Variable :  Event Period (Q0) - independently with global Q
       int32 EventPeriod_n_1;   // Input/Variable :  Event Period (Q0) - independently with global Q
       int32 EventPeriod_n_2;   // Input/Variable :  Event Period (Q0) - independently with global Q
       int32 EventPeriod_n_3;   // Input/Variable :  Event Period (Q0) - independently with global Q
       int32 EventPeriod_n_4;   // Input/Variable :  Event Period (Q0) - independently with global Q
       int32 EventPeriod_n_5;   // Input/Variable :  Event Period (Q0) - independently with global Q
       int32 EventPeriod_n_6;   // Input/Variable :  Event Period (Q0) - independently with global Q
       int16 InputSelect;       // Input : Input selection between TimeStamp (InputSelect=0) and EventPeriod (InputSelect=1)
       } SPEED_MEAS_CAP1;       // Data type created

/*
Note that SpeedScaler = 60*f/rmp_max
where f = CLK_freq/(128*N), N = number of teeth in sprocket
and 128 is pre-determined prescaler of timer 2 (used by capture)
*/

typedef SPEED_MEAS_CAP1 *SPEED_MEAS_CAP_handle1;
/*-----------------------------------------------------------------------------
Default initalizer for the SPEED_MEAS_CAP object.
-----------------------------------------------------------------------------*/

#define SPEED_MEAS_CAP_DEFAULTS1   { 0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                    0, \
                                  }

/*------------------------------------------------------------------------------
    SPEED_PR Macro Definition
------------------------------------------------------------------------------*/

#define SPEED_PR_MACRO1(v)                                       \
   if (v.InputSelect == 0)                                      \
   {                                                            \
     v.EventPeriod_n_6=v.EventPeriod_n_5;                     \
     v.EventPeriod_n_5=v.EventPeriod_n_4;                       \
     v.EventPeriod_n_4=v.EventPeriod_n_3;                       \
     v.EventPeriod_n_3=v.EventPeriod_n_2;                       \
     v.EventPeriod_n_2=v.EventPeriod_n_1;                       \
     v.OldTimeStamp = v.NewTimeStamp;                           \
     v.NewTimeStamp = v.TimeStamp;                              \
     v.EventPeriod_n_1 = v.NewTimeStamp - v.OldTimeStamp;       \
                                                                \
     if (v.EventPeriod_n_1 < 0)                                 \
         v.EventPeriod_n_1 += 32767;   /* 0x7FFF = 32767*/      \
    }                                                           \
                                                                \
     v.EventPeriod_n=-0.33333333333333*v.EventPeriod_n_1 + 0.33333333333333*v.EventPeriod_n_3 + 0.33333333333333*v.EventPeriod_n_4 + 0.33333333333333*v.EventPeriod_n_5 + 0.33333333333333*v.EventPeriod_n_6;\

#endif /* SPEED_PR_1_H_ */
