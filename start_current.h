/*
 *Start_Current.h
 *
 *  Created on: May 1, 2024
 *      Author: Bruce
 */

#ifndef Start_current_H_
#define Start_current_H_

typedef struct {
                 _iq  ia;
                 _iq  theta;
                 _iq  i_start;
                 int16 triggered;
               } STARTCURRENT;

typedef STARTCURRENT *STARTCURRENT_handle;

#define STARTCURRENT_DEFAULTS {                                                     \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                           }                                                        \

#define START_CURRENT(v)               \
    if (v.theta >= _IQ(0.249) && !v.triggered) \
    {v.i_start = v.ia; \
    v.i_start = _IQmpy(_IQ(BASE_CURRENT),v.i_start); \
    v.i_start = _IQmpy(_IQ(1.0),v.i_start); \
    v.triggered = TRUE; \
     } \
    if (v.theta < _IQ(0.247)) \
    { \
    v.triggered = FALSE;} \


#endif /* TORQUECAL_H_ */
