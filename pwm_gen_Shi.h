/*
 * PWM_gen_Shi.h
 *
 *  Created on: Jun,21 2024
 *      Author: Bruce
 */

#ifndef PWMgen_Shi_H_
#define PWMgen_Shi_H_

typedef struct {
                 _iq  theta;     //input
                 _iq delta;
                 _iq d_pwm;
                 _iq  g1;       //Output
                 _iq  g2;       //Output
                 _iq  g3;       //Output
                 _iq  g4;       //Output
                 _iq  g5;       //Output
                 _iq  g6;       //Output
                 int16 x;
               } PWMgen_Shi;

typedef PWMgen_Shi *PWMgen_Shi_handle;

#define PWMgenShi_DEFAULTS {                                                 \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                           }                                                        \
// Use first for Motor A, use second for motor B
#define PWMgenShi(v) \
                    { \
                     v.g3 = _IQ((v.theta>=_IQ(0.083333333333333)-ad)&&(v.theta<_IQ(0.416666666666667)+ad));\
                     v.g4 = _IQmpy(v.d_pwm,_IQ(((v.theta>=_IQ(0.583333333333333)-ad)&&(v.theta<_IQ(0.916666666666667)+ad))));\
                     v.g5 = _IQ((((theta>=_IQ(0.75)-ad)&&(theta<_IQ(1))))||(theta>=_IQ(0))&&(theta<_IQ(0.083333333333333)+ad));\
                     v.g6 = _IQmpy(v.d_pwm,_IQ((v.theta>=_IQ(0.25)-ad)&&(v.theta<_IQ(0.583333333333333)+ad)));\
                     v.g1 = _IQ((v.theta>=_IQ(0.416666666666667)-ad)&&(v.theta<_IQ(0.75)+ad));\
                     v.g2 = _IQmpy(v.d_pwm,_IQ((((v.theta>=_IQ(0.916666666666667)-ad)&&(v.theta<_IQ(1))))||(v.theta>=_IQ(0))&&(v.theta<_IQ(0.25)+ad)));\
                     } \

                    // MOTOR D
//                    { \
//                     v.g4 = _IQ((v.theta>=_IQ(0.083333333333333)-ad)&&(v.theta<_IQ(0.416666666666667)+ad));\
//                     v.g3 = _IQmpy(v.d_pwm,_IQ(((v.theta>=_IQ(0.583333333333333)-ad)&&(v.theta<_IQ(0.916666666666667)+ad))));\
//                     v.g6 = _IQ((((theta>=_IQ(0.75)-ad)&&(theta<_IQ(1))))||(theta>=_IQ(0))&&(theta<_IQ(0.083333333333333)+ad));\
//                     v.g5 = _IQmpy(v.d_pwm,_IQ((v.theta>=_IQ(0.25)-ad)&&(v.theta<_IQ(0.583333333333333)+ad)));\
//                     v.g2 = _IQ((v.theta>=_IQ(0.416666666666667)-ad)&&(v.theta<_IQ(0.75)+ad));\
//                     v.g1 = _IQmpy(v.d_pwm,_IQ((((v.theta>=_IQ(0.916666666666667)-ad)&&(v.theta<_IQ(1))))||(v.theta>=_IQ(0))&&(v.theta<_IQ(0.25)+ad)));\
//                     } \
//            v.g4 = _IQ((v.theta>=_IQ(0.0))&&(v.theta<_IQ(0.5)));\
//            v.g3 = _IQmpy(v.d_pwm,_IQ(((v.theta>=_IQ(0.5))&&(v.theta<_IQ(1)))));\
//            v.g6 = _IQ((((theta>=_IQ(0.666666666666667))&&(theta<_IQ(1))))||(theta>=_IQ(0))&&(theta<_IQ(0.166666666666667)));\
//            v.g5 = _IQmpy(v.d_pwm,_IQ((v.theta>=_IQ(0.16666666666667))&&(v.theta<_IQ(0.666666666666667))));\
//            v.g2 = _IQ((v.theta>=_IQ(0.333333333333333))&&(v.theta<_IQ(0.83333333333333)));\
//            v.g1 = _IQmpy(v.d_pwm,_IQ((((v.theta>=_IQ(0.83333333333333))&&(v.theta<_IQ(1))))||(v.theta>=_IQ(0))&&(v.theta<_IQ(0.333333333333333))));\
//            break; \
//        }
//         v.g1 = _IQ(v.theta>=_IQ(0.916666666666667)&&(v.theta<(_IQ(0.916666666666667)+v.delta))) + _IQmpy(v.d_pwm,_IQ(  (v.theta>=(_IQ(0.916666666666667)+v.delta))||(v.theta<_IQ(0.0833333333333333))||((v.theta>=(_IQ(0.0833333333333333)+v.delta))&&(v.theta<_IQ(0.25)))  )) + _IQmpy(v.d_pwm2,_IQ((v.theta>=_IQ(0.0833333333333333))&&(v.theta<(_IQ(0.0833333333333333)+v.delta))) ); \
//         v.g2 = _IQ( ((v.theta>=_IQ(0.416666666666667))&&(v.theta<_IQ(0.583333333333333))) || ((v.theta>=(_IQ(0.583333333333333)+v.delta))&&(v.theta<_IQ(0.75)))  )  + _IQmpy(v.d_pwm2,_IQ((v.theta>=_IQ(0.583333333333333))&&(v.theta<(_IQ(0.583333333333333)+v.delta))) ) ; \
//         v.g3 = _IQ(v.theta>=_IQ(0.583333333333333)&&(v.theta<_IQ(0.583333333333333)+v.delta)) + _IQmpy(v.d_pwm,_IQ( ((v.theta>=_IQ(0.583333333333333)+v.delta)&&(v.theta<_IQ(0.75))) || ((v.theta>=_IQ(0.75)+v.delta)&&(v.theta<_IQ(0.916666666666667))) )) + _IQmpy(v.d_pwm2,_IQ((v.theta>=_IQ(0.75))&&(v.theta<(_IQ(0.75)+v.delta))) );\
//         v.g4 = _IQ( ((v.theta>=_IQ(0.083333333333333))&&(v.theta<_IQ(0.25))) || ((v.theta>=_IQ(0.25)+v.delta)&&(v.theta<_IQ(0.416666666666667)))  )  + _IQmpy(v.d_pwm2,_IQ((v.theta>=_IQ(0.25))&&(v.theta<(_IQ(0.25)+v.delta))) ) ;\
//         v.g5 = _IQ(v.theta>=_IQ(0.25)&&(v.theta<_IQ(0.25)+v.delta)) + _IQmpy(v.d_pwm,_IQ( ((v.theta>=_IQ(0.25)+v.delta)&&(v.theta<_IQ(0.416666666666667))) || ((v.theta>=_IQ(0.416666666666667)+v.delta)&&(v.theta<_IQ(0.583333333333333))) )) + _IQmpy(v.d_pwm2,_IQ((v.theta>=_IQ(0.416666666666667))&&(v.theta<(_IQ(0.416666666666667)+v.delta))) );\
//         v.g6 = _IQ( ((v.theta>=_IQ(0.75))&&(v.theta<_IQ(0.916666666666667))) || (v.theta>=_IQ(0.916666666666667)+v.delta) || (v.theta<_IQ(0.083333333333333))  )  +  _IQmpy(v.d_pwm2,_IQ((v.theta>=_IQ(0.916666666666667))&&(v.theta<(_IQ(0.916666666666667)+v.delta))) );\


#endif /* PWMgen_H_ */
