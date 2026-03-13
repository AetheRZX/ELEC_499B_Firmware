/*
 * Three_Phase_conduct.h
 *
 *  Created on: May,09 2024
 *      Author: Bruce
 */

#ifndef ThreePhase_H_
#define ThreePhase_H_

typedef struct {
                 _iq theta;
                 Uint16 TP;
               } THREEPHASE;

typedef THREEPHASE *THREEPHASE_handle;

#define THREEPHASE_DEFAULTS {                                                 \
                                   _IQ(0),                                    \
                                   0,                                               \
                            }                                                        \

#define THREEPHASE(v)                    \
    v.TP =  (v.theta>=_IQ(0.916666666666667)-ad && (v.theta<_IQ(0.916666666666667)+ad)) + (v.theta>=_IQ(0.083333333333333)-ad && (v.theta<_IQ(0.083333333333333)+ad))+ (v.theta>=_IQ(0.25)-ad && (v.theta<_IQ(0.25)+ad)) + (v.theta>=_IQ(0.416666666666667)-ad && (v.theta<_IQ(0.416666666666667)+ad))+ (v.theta>=_IQ(0.583333333333333)-ad && (v.theta<_IQ(0.583333333333333)+ad)) + (v.theta>=_IQ(0.75)-ad && (v.theta<_IQ(0.75)+ad)); \

#endif /* ThreePhase_H_*/
