/*
 * BackEMF_cal.h
 *
 *  Created on: Feb 23, 2024
 *      Author: Bruce
 */

#ifndef BACKEMF_H_
#define BACKEMF_H_

typedef struct { _iq  ea;  //OUTPUT
                 _iq  eb;  //OUTPUT
                 _iq  ec;  //OUTPUT
                 _iq  theta;   //INPUT
                 _iq  speed;   //INPUT
                 _iq  lambda;  //constant
                 _iq  ua;
                 _iq  ex;
                 _iq  theta_x;
               } BACKEMF;

typedef BACKEMF *BACKEMF_handle;

#define BACKEMF_DEFAULTS {                                                        \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                             }                                                                                   \

#define BACKEMF_CAL(v)                                                               \
/*v.ua = _IQmpy(_IQsinPU(v.theta),v.speed);*/\
v.ea = _IQmpy(_IQcosPU(v.theta),_IQmpy(v.speed, v.lambda));\
v.eb = _IQmpy(_IQcosPU(v.theta-DEG_120),_IQmpy(v.speed, v.lambda));        \
v.ec = _IQmpy(_IQcosPU(v.theta+DEG_120),_IQmpy(v.speed, v.lambda));       \
v.theta_x = _IQdiv(v.theta,_IQ(0.1666666666667));\
v.theta_x = _IQmpy(_IQfrac(v.theta_x),_IQ(0.166666666666667));\
v.theta_x = v.theta_x + _IQ(0.1666666666667);\
v.ex = _IQsinPU(v.theta_x);\
v.ex = _IQmpy(v.ex,v.lambda);\


#endif /*BACKEMF_H_ */
