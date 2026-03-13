/*
 * DutyRatioCal2.h
 *
 *  Created on: Mar 3, 2024
 *      Author: Bruce
 */

#ifndef DutyCal2_H_
#define DutyCal2_H_

typedef struct {
                 _iq Te; // Input
                 _iq Te_ref; //Input
                 _iq Ts; //Constant parameter Controller time size
                 Uint16 commutation; //Input
                 _iq fup; //Input
                 _iq fdown;//Input
                 _iq fup2; //Input
                 _iq fdown2;//Input
                 _iq f1; //output
                 _iq f2; //output
                 _iq num; //Medimum
                 _iq den; //Medimum
                 _iq Out; //Output, duty ratio
                 _iq v1;
                 _iq T;
               } DutyCal2;

typedef DutyCal2 *DutyCal2_handle;

#define DutyCal2_DEFAULTS { 0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                           }                                                        \

#define DUTY_CAL2(v)                                                                \
    v.Ts = _IQdiv(_IQ(0.001),_IQ(ISR_FREQUENCY));                                  \
    v.T = v.Te + _IQdiv((_IQmpy(_IQmpy((_IQ(1)-v.Out),v.f2),v.Ts)+_IQmpy(_IQmpy(v.Out,v.f1),v.Ts)),_IQ(2)); \
    if (v.commutation == 0) \
     {v.f1 = v.fup; \
      v.f2 = v.fdown; \
      v.num = v.Te_ref - v.T - _IQmpy(v.f2,v.Ts)/*+ _IQdiv(_IQmpy(_IQdiv(_IQmpy(v.f1,v.f2),(v.f1-v.f2)),v.Ts),_IQ(2))*/;  } \
    else \
     {v.f1 = v.fup2; \
      v.f2 = v.fdown2; \
     v.num = v.Te_ref /*+_IQ(0.0003)*/ - v.T - _IQmpy(v.f2,v.Ts) /*- _IQdiv(_IQmpy(_IQdiv(_IQmpy(v.f1,v.f2),(v.f1-v.f2)),v.Ts),_IQ(0.8))*/;      }            \
     v.den = _IQmpy(v.Ts,(v.f1 - v.f2));                                            \
     v.v1 = _IQdiv(v.num,v.den);                                                   \
     if (v.commutation == 0) \
     {v.Out = _IQsat(v.v1, _IQ(0.9), _IQ(0.05)); }  /*saturate DutyRatio between 0.9 and 0.05*/      \
     else if (v.commutation == 700) \
     {v.Out =_IQsat(v.v1, _IQ(0.98), _IQ(0.05))/*_IQsat(v.v1, _IQ(0.97), _IQ(0.2))*/; }   \
     else \
      {v.Out = _IQsat(v.v1, _IQ(0.98), _IQ(0.05))/*_IQsat(v.v1, _IQ(0.96), _IQ(0.2))*/;  }  \

#endif /* DutyCal2_H_ */
