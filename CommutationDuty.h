/*
 * CommutationDuty.h
 *
 *  Created on: Apr,23 2024
 *      Author: Bruce
 */

#ifndef ComutationDuty_H_
#define ComutationDuty_H_

typedef struct {
                 _iq  speed_rads;     //input
                 _iq  vdc;         //input
                 _iq  i_start;     //input
                 _iq lambda;       //parameter
                 _iq fup2;
                 _iq fdown2;
                 _iq ea;
                 _iq eb;
                 _iq ec;
                 _iq ex;
                 _iq ey;
                 _iq test1;
                 Uint16 commutation;
               } COMMUTATIONDUTY;

typedef COMMUTATIONDUTY *COMMUTATIONDUTY_handle;

#define COMMUTATIONDUTY_DEFAULTS {                                                 \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                           }                                                        \


#define COMMUTATION_DUTY(v)   \
    /*v.duty = _IQdiv((_IQmpy(v.lambda,v.speed_rads) + _IQmpy(v.i_start,_IQ(RS))),v.vdc);*/  \
    /*v.duty = _IQmpy(_IQ(1.5),v.duty); */   \
    /*v.duty = v.duty+_IQ(0.5);*/   \
    /*v.duty = _IQmpy(_IQ(1.12),v.duty);*/  \
    switch (v.commutation){  \
    case 400: v.ex = v.ea; \
              v.ey = -v.eb; break; \
    case 500: v.ex = -v.eb; \
              v.ey = v.ec; break; \
    case 600: v.ex = v.ec; \
              v.ey = -v.ea; break; \
    case 700: v.ex = -v.ea; \
              v.ey = v.eb; break; \
    case 800: v.ex = v.eb; \
              v.ey = -v.ec; break; \
    case 900: v.ex = -v.ec; \
              v.ey = v.ea; break; \
    } \
    /*v.fup2 = _IQmpy(_IQ(0.03811),v.vdc) - _IQmpy(_IQ(3),_IQmpy(v.speed_rads,_IQ(0.001369)))*/;\
    /*v.fdown2 = v.fup2 - _IQmpy(_IQ(2),_IQmpy(_IQ(0.03589),v.vdc))*/;\
    v.fup2 = _IQdiv(_IQmpy(_IQ(2),_IQmpy(v.ex,v.vdc)),v.speed_rads) - _IQmpy(_IQ(3),_IQmpy(v.speed_rads,_IQ(0.001369))); \
    v.fdown2 = _IQdiv(_IQmpy(_IQ(2),_IQmpy(v.ey,v.vdc)),v.speed_rads) - _IQmpy(_IQ(3),_IQmpy(v.speed_rads,_IQ(0.001369))); \
    v.fup2 = _IQdiv(v.fup2,_IQ(LS)) + v.test1; \
    v.fdown2 = _IQdiv(v.fdown2,_IQ(LS)) + v.test1; \
//    v.fup2 = _IQdiv(v.fup2,_IQ(LS)) -_IQ(1.46);\
//    v.fdown2 = _IQdiv(v.fdown2,_IQ(LS)) -_IQ(1.46);\


#endif /* ComutationDutyt_H_ */
