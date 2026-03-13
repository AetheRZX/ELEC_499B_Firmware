/*
 * Torque_cal.h
 *
 *  Created on: Feb 11, 2024
 *      Author: Bruce
 */

#ifndef TORQUECAL_H_
#define TORQUECAL_H_

typedef struct { _iq  ea;
                 _iq  eb;
                 _iq  ec;
                 _iq  ia;
                 _iq  ib;
                 _iq  ic;
                 _iq  wr;
                 _iq v1;
                 _iq  Out;
                 _iq test;
                 _iq  test1;

               } TORQUECAL;

typedef TORQUECAL *TORQUECAL_handle;

#define TORQUECAL_DEFAULTS {                                                        \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                           }                                                        \

#define TORQUE_CAL(v)                                                               \
v.v1 = _IQmpy(v.ea,v.ia) + _IQmpy(v.eb,v.ib) + _IQmpy(v.ec,v.ic);           \
v.Out = _IQdiv(v.v1,v.wr);       \
v.test = _IQmpy(_IQ(100),v.Out);  \

 //v.test = _IQsat(v.test,_IQ(0.8),_IQ(0.67)); \


//v.test = v.test - _IQ(0.6); \
//v.test = _IQmpy(_IQ(3),v.test); \
//v.test = _IQsat(v.test,_IQ(0.8),_IQ(0.65)); /


#endif /* TORQUECAL_H_ */
