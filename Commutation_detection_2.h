/*
 * Commutation_detection_2.h
 *
 *  Created on: May,09 2024
 *      Author: Bruce
 */

#ifndef ComutationDuty2_H_
#define ComutationDuty2_H_

typedef struct {
    _iq  speed_rads;     //input
    _iq  vdc;         //input
    _iq  i_start;     //input
    _iq  delta;       //Output
    _iq  delta2;
    _iq  delta_test;
    _iq  number;
    _iq theta;
    _iq lambda_md;
    Uint16 commutation;
    Uint16 commutation_2;
               } COMMUTATIONDETEC2;

typedef COMMUTATIONDETEC2 *COMMUTATIONDETEC2_handle;

#define COMMUTATIONDETEC_DEFAULTS2 {                                                 \
                                   _IQ(0),                                    \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   }                                                        \

#define COMMUTATION_DETEC2(v)                    \
        v.delta = _IQdiv(v.vdc,_IQ(3))-_IQmpy(_IQmpy(v.speed_rads,_IQ(0.037)),_IQ(50)); \
        v.delta = _IQdiv(_IQmpy(v.i_start,_IQ(LS)),v.delta); \
        v.delta = _IQdiv(_IQmpy(v.delta,v.speed_rads),_IQ(PI)); \
        v.delta = _IQmpy(_IQ(200),v.delta);  \
        v.delta2 = _IQmpy(_IQ(1.5),v.delta);\
        v.commutation =  (v.theta>=_IQ(0.916666666666667) && (v.theta<_IQ(0.916666666666667)+v.delta))*400 + (v.theta>=_IQ(0.083333333333333) && (v.theta<_IQ(0.083333333333333)+v.delta))*500 + (v.theta>=_IQ(0.25)&& (v.theta<_IQ(0.25)+v.delta))*600 + (v.theta>=_IQ(0.416666666666667) && (v.theta<_IQ(0.416666666666667)+v.delta))*700 + (v.theta>=_IQ(0.583333333333333) && (v.theta<_IQ(0.583333333333333)+v.delta))*800 + (v.theta>=_IQ(0.75) && (v.theta<_IQ(0.75)+v.delta))*900;\
        v.commutation_2 =  (v.theta>=_IQ(0.916666666666667)&&(v.theta<_IQ(0.916666666666667)+v.delta2))*400 + (v.theta>=_IQ(0.083333333333333)&&(v.theta<_IQ(0.083333333333333)+v.delta2))*500 + (v.theta>=_IQ(0.25)&&(v.theta<_IQ(0.25)+v.delta2))*600 + (v.theta>=_IQ(0.416666666666667)&&(v.theta<_IQ(0.416666666666667)+v.delta2))*700 + (v.theta>=_IQ(0.583333333333333)&&(v.theta<_IQ(0.583333333333333)+v.delta2))*800 + (v.theta>=_IQ(0.75)&&(v.theta<_IQ(0.75)+v.delta2))*900;\

#endif /* ComutationDetec2_H_ */
