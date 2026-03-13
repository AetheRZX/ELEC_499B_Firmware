/*
 * Commutation_Detection.h
 *
 *  Created on: Apr,17 2024
 *      Author: Bruce
 */

#ifndef ComutationDetec_H_
#define ComutationDetec_H_

typedef struct {
                 _iq  speed_rads;     //input
                 _iq  vdc;         //input
                 _iq  i_start;     //input
                 _iq  delta;       //Output
                 _iq  delta2;
                 _iq  number;
                 _iq theta;
                 Uint16 commutation;
                 Uint16 commutation_2;
//                 int16 count;
//                 int16 count_1;
//                 Uint16 y;
//                 int16 trigger;
//                 _iq theta;
               } COMMUTATIONDETEC;

typedef COMMUTATIONDETEC *COMMUTATIONDETEC_handle;

#define COMMUTATIONDETEC_DEFAULTS {                                                 \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                           }                                                        \

#define COMMUTATION_DETEC(v)                    \
    v.delta = _IQdiv(v.i_start,v.vdc);  \
    v.delta = _IQmpy(v.delta,_IQ(LS));  \
    v.delta = _IQmpy(_IQ(2.0),v.delta);  \
    v.number = _IQmpy(_IQmpy(_IQ(20),v.delta),_IQ(100)); \
    v.number = _IQmpy(_IQ(10),v.number); \
    v.delta = _IQdiv(_IQmpy(v.delta,v.speed_rads),_IQ(PI));     \
    v.delta = _IQmpy(_IQ(100),v.delta);  \
    v.delta2 = _IQmpy(_IQ(1.5),v.delta);\
    v.commutation =  (v.theta>=_IQ(0.916666666666667) && (v.theta<_IQ(0.916666666666667)+_IQmpy(_IQ(1),v.delta)))*400 + (v.theta>=_IQ(0.083333333333333) && (v.theta<_IQ(0.083333333333333)+v.delta))*500 + (v.theta>=_IQ(0.25)&& (v.theta<_IQ(0.25)+v.delta))*600 + (v.theta>=_IQ(0.416666666666667) && (v.theta<_IQ(0.416666666666667)+v.delta))*700 + (v.theta>=_IQ(0.583333333333333) && (v.theta<_IQ(0.583333333333333)+v.delta))*800 + (v.theta>=_IQ(0.75) && (v.theta<_IQ(0.75)+v.delta))*900;\
    v.commutation_2 =  (v.theta>=_IQ(0.916666666666667)&&(v.theta<_IQ(0.916666666666667)+v.delta2))*400 + (v.theta>=_IQ(0.083333333333333)&&(v.theta<_IQ(0.083333333333333)+v.delta2))*500 + (v.theta>=_IQ(0.25)&&(v.theta<_IQ(0.25)+v.delta2))*600 + (v.theta>=_IQ(0.416666666666667)&&(v.theta<_IQ(0.416666666666667)+v.delta2))*700 + (v.theta>=_IQ(0.583333333333333)&&(v.theta<_IQ(0.583333333333333)+v.delta2))*800 + (v.theta>=_IQ(0.75)&&(v.theta<_IQ(0.75)+v.delta2))*900;\
//    v.count = _IQint(v.number); \
//    if (v.theta == _IQ(0.25) && !v.trigger) \
//    {v.y = 400; \
//    v.trigger = TRUE; \
//    v.count_1 = v.count;} \
//    if (v.trigger == TRUE) \
//    { \
//        v.count_1--; \
//        if (v.count_1 == 0) \
//        {v.trigger = FALSE; \
//        v.y = 0; \
//        } \
//    } \



#endif /* ComutationDetec_H_ */
