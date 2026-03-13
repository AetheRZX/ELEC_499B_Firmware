/*
 * pid_control_SPEED.h
 *
 *  Created on: Feb 28, 2024
 *      Author: Bruce
 */

#ifndef PI_CONTROLSPEED_H_
#define PI_CONTROLSPEED_H_

typedef struct {  _iq  Ref;             // Input: reference set-point
                  _iq  Fbk;             // Input: feedback
                  _iq  Out;             // Output: controller output
                } PI_CN_TERMINALS1;

typedef struct {  _iq  Kp;              // Parameter: proportional loop gain
                  _iq  Ki;              // Parameter: integral gain
                  _iq  Ba;
                  _iq  Umax;            // Parameter: upper saturation limit
                  _iq  Umin;            // Parameter: lower saturation limit
                } PI_CN_PARAMETERS1;

typedef struct {  _iq  up;              // Data: proportional term
                  _iq  ui;              // Data: integral term
                  _iq  i1;              // Data: integrator storage: ui(k-1)
                  _iq  v1;              // Data: pre-saturated controller output
                  _iq  e1;              // Data: integrator storage: ui(k-1)
                  _iq  e;               // Data: saturation record: [u(k-1) - v(k-1)]
                } PI_CN_DATA1;


typedef struct {  PI_CN_TERMINALS1  term;
                  PI_CN_PARAMETERS1 param;
                  PI_CN_DATA1       data;
                } PI_CONTROLLER1;


typedef    PI_CONTROLLER1   *PI_handle1;


/*-----------------------------------------------------------------------------
Default initalisation values for the PID_GRANDO objects
-----------------------------------------------------------------------------*/

#define PI_TERM_SPEED_DEFAULTS {              \
                           0,           \
                           0,           \
                           0,           \
                          }

#define PI_PARAM_SPEED_DEFAULTS {             \
                           _IQ(1.0),    \
                           _IQ(1.0),    \
                           _IQ(0.05),    \
                           _IQ(-0.05)    \
                          }

#define PI_DATA_SPEED_DEFAULTS {              \
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

#define PI_SPEED_MACRO(v)                                                                                 \
                                                                                                    \
    /* proportional term */                                                                         \
    v.data.e = v.term.Ref - v.term.Fbk;                                                             \
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
       v.data.i1 = v.data.ui;                                                                        \
    /* control output */                                                                            \
    v.data.v1 = v.data.up + v.data.ui;                                                              \
    v.data.v1 = v.data.v1 /*- _IQmpy(v.term.Fbk,v.param.Ba)*/  ;  \
    v.term.Out = _IQsat(v.data.v1, v.param.Umax, v.param.Umin);                                 \

    //v.term.Out = _IQsat(v.data.v1, v.param.Umax, v.param.Umin);                                      \
              /*_IQsat(v.data.v1, v.param.Umax, v.param.Umin);\*/
    //v.term.Out =_IQsat(v.term.Out,_IQ(0.1),_IQ(-1));                                              \
    /*v.data.w1 = (v.term.Out == v.data.v1) ? _IQ(1.0) : _IQ(0.0);*/                                \



#endif /* PID_CONTROL_H_ */
