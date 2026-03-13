/*
 * pid_control.h
 *
 *  Created on: Feb 18, 2022
 *      Author: LabUser
 */

#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

typedef struct {  _iq  Ref;             // Input: reference set-point
                  _iq  Fbk;             // Input: feedback
                  _iq  Out;             // Output: controller output
                } PI_CN_TERMINALS;

typedef struct {  _iq  Kp;              // Parameter: proportional loop gain
                  _iq  Ki;              // Parameter: integral gain
                  _iq  Umax;            // Parameter: upper saturation limit
                  _iq  Umin;            // Parameter: lower saturation limit
                } PI_CN_PARAMETERS;

typedef struct {  _iq  up;              // Data: proportional term
                  _iq  ui;              // Data: integral term
                  _iq  i1;              // Data: integrator storage: ui(k-1)
                  _iq  v1;              // Data: pre-saturated controller output
                  _iq  e1;              // Data: integrator storage: ui(k-1)
                  _iq  e;               // Data: saturation record: [u(k-1) - v(k-1)]
                } PI_CN_DATA;


typedef struct {  PI_CN_TERMINALS  term;
                  PI_CN_PARAMETERS param;
                  PI_CN_DATA       data;
                } PI_CONTROLLER;


typedef PI_CONTROLLER   *PI_handle;


/*-----------------------------------------------------------------------------
Default initalisation values for the PID_GRANDO objects
-----------------------------------------------------------------------------*/

#define PI_TERM_DEFAULTS {              \
                           0,           \
                           0,           \
                           0,           \
                          }

#define PI_PARAM_DEFAULTS {             \
                           _IQ(1.0),    \
                           _IQ(1.0),    \
                           _IQ(0.05),    \
                           _IQ(-0.05)    \
                          }

#define PI_DATA_DEFAULTS {              \
                           _IQ(0.0),    \
                           _IQ(0.0),    \
                           _IQ(0.0),    \
                           _IQ(0.0),    \
                           _IQ(0.0),    \
                           _IQ(0.0)     \
                          }


/*------------------------------------------------------------------------------
    PID_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PI_MACRO(v)                                                                                 \
                                                                                                    \
    /* proportional term */                                                                         \
    v.data.e = -v.term.Ref + v.term.Fbk;                                                             \
    v.data.up = _IQmpy(v.param.Kp,v.data.e);                           \
                                                                                                    \
    /* integral term */                                                                             \
    v.data.ui = (v.term.Out == v.data.v1) ? (_IQmpy(v.param.Ki, v.data.e) + v.data.i1) : v.data.i1;   \
          if(v.data.ui > v.param.Umax)                                                               \
          {                                                                                            \
              v.data.ui = v.param.Umax;                                                              \
          }                                                                                         \
          if(v.data.ui < v.param.Umin)                                                                 \
          {\
              v.data.ui = v.param.Umin;\
          }\
          v.data.i1 = v.data.ui;                                                                               \
                                                                                                    \
    /* control output */                                                                            \
    v.data.v1 = v.data.up + v.data.ui;                                                              \
    v.term.Out = _IQsat(v.data.v1, v.param.Umax, v.param.Umin);                                              \

#define RESET_PI_CTRL(v) \
    v.data.e1 = 0; \
    v.data.ui = 0; \
    v.data.up = 0; \
    v.data.v1 = 0; \
    v.data.i1 = 0; \
    v.data.e  = 0; \
    v.term.Out = 0;    \
    v.param.Umax=_IQ(0.05);    \
    v.param.Umin=_IQ(-0.05);   \


#endif /* PID_CONTROL_H_ */
