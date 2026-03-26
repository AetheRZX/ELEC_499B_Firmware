/*
 * PWM_gen.h
 *
 *  Created on: Apr,25 2024
 *      Author: Bruce
 */

#ifndef PWMgen_H_
#define PWMgen_H_




typedef struct {
                 _iq  theta;     //input
                 _iq theta_prime; // advanced firing angle adjusted
                 _iq delta;
                 _iq d_pwm;
                 _iq  g1;       //Output
                 _iq  g2;       //Output
                 _iq  g3;       //Output
                 _iq  g4;       //Output
                 _iq  g5;       //Output
                 _iq  g6;       //Output
                 int16 x;
               } PWMgen;

typedef PWMgen *PWMgen_handle;

#define PWMgen_DEFAULTS {                                                 \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                                   0,                                               \
                           }                                                        \

#define PWMgen(v)                    \
            switch (v.x){                                                                \
            /**case 400: v.g1 = _IQ(1); \
                    v.g2 = _IQ(0); v.g3 = _IQ(0); v.g4 = _IQ(0); v.g5 = _IQ(0); \
                    v.g6 = v.d_pwm; \
                      break;                      \
            case 500: v.g4 = _IQ(1); \
                    v.g2 = _IQ(0); v.g3 = _IQ(0); v.g6 = _IQ(0); v.g5 = _IQ(0); \
                    v.g1 = v.d_pwm; \
                     break;                         \
            case 600: v.g5 = _IQ(1); \
                    v.g2 = _IQ(0); v.g3 = _IQ(0); v.g1 = _IQ(0); v.g6 = _IQ(0); \
                    v.g4 = v.d_pwm; \
                    break;      \
            case 700:\
                    v.g2 = _IQ(1); \
                    v.g1 = _IQ(0); v.g3 = _IQ(0); v.g4 = _IQ(0); v.g6 = _IQ(0); \
                    v.g5 = v.d_pwm; \
                    break; \
            case 800:\
                    v.g3 = _IQ(1); \
                    v.g1 = _IQ(0); v.g6 = _IQ(0); v.g4 = _IQ(0); v.g5 = _IQ(0); \
                    v.g2 = v.d_pwm; \
                    break;      \
            case 900:\
                    v.g6 = _IQ(1); \
                    v.g2 = _IQ(0); v.g1 = _IQ(0); v.g4 = _IQ(0); v.g5 = _IQ(0); \
                    v.g3 = v.d_pwm; \
                    break;      \ */\
            case 0:\
                /*ad = 0; ad=0 for 120-deg commutation */\
                /*v.g4 = _IQ((((theta_shifted>=DEG_270-ad)&&(theta_shifted<DEG_360)))||(theta_shifted>=DEG_0)&&(theta_shifted<DEG_30+ad));\
                v.g3 = _IQmpy(v.d_pwm,_IQ(((theta_shifted>=DEG_90-ad)&&(theta_shifted<DEG_210+ad))));\
                v.g6 = _IQ((theta_shifted>=DEG_30-ad)&&(theta_shifted<DEG_150+ad));\
                v.g5 = _IQmpy(v.d_pwm,_IQ((theta_shifted>=DEG_210-ad)&&(theta_shifted<DEG_330+ad)));\
                v.g2 = _IQ((theta_shifted>=DEG_150-ad)&&(theta_shifted<DEG_270+ad));\
                v.g1 = _IQmpy(v.d_pwm,_IQ((((theta_shifted>=DEG_330-ad)&&(theta_shifted<DEG_360)))||(theta_shifted>=DEG_0)&&(theta_shifted<DEG_90+ad)));\*/\
                v.g1 = _IQmpy(v.d_pwm,_IQ((((v.theta_prime>=SEC5_NEXTANGLE)&&(v.theta_prime<SEC3_NEXTANGLE)))));\
                v.g2 = _IQ(((v.theta_prime>=SEC2_NEXTANGLE)&&(v.theta_prime<DEG_360))||(v.theta_prime>=DEG_0) && (v.theta_prime < SEC4_NEXTANGLE));\
                v.g3 = _IQmpy(v.d_pwm,_IQ(((v.theta_prime>=SEC6_NEXTANGLE)&&(v.theta_prime<SEC5_NEXTANGLE))));\
                v.g4 = _IQ((((v.theta_prime>=SEC1_NEXTANGLE)&&(v.theta_prime<SEC2_NEXTANGLE))));\
                v.g5 = _IQmpy(v.d_pwm,_IQ((v.theta_prime>=SEC3_NEXTANGLE)&&(v.theta_prime<DEG_360)));\
                v.g6 = _IQ((v.theta_prime>=SEC4_NEXTANGLE)&&(v.theta_prime<SEC1_NEXTANGLE));\
                 break; \
                  }
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
