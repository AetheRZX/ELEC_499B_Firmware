/*
 * speed_est_1.h
 *
 *  Created on: Feb 8, 2022
 *      Author: LabUser
 */

#ifndef SPEED_EST_1_H_
#define SPEED_EST_1_H_


typedef struct {
       _iq nexttheta;
       _iq currenttheta;
       _iq dtheta;
       _iq phic;
       Uint32 Timestamp;
       Uint32 Timestamp1;
       Uint32 Timestamp2;
       int32 Period;
       Uint16 HallGpio;
       Uint32 speedscaler;
       Uint32 BaseRpm;             // Parameter : Scaler converting GLOBAL_Q speed to rpm (Q0) speed - independently with global Q
       int32 EventPeriod;
       _iq nextspeed;
       _iq realspeed;
       _iq realtheta;
       int32 speedrpm;
       _iq speed_rads;
       _iq  StepAngleMax;  // Parameter: Maximum step angle (pu)
       _iq ad;
       _iq ad1;
       Uint16 SUflag;} SPEED_ESTIMATION1;      // Data type created

typedef SPEED_ESTIMATION1 *SPEED_ESTIMATION_handle1;
/*-----------------------------------------------------------------------------
Default initalizer for the SPEED_ESTIMATION object.
-----------------------------------------------------------------------------*/
#define SPEED_ESTIMATION_DEFAULTS1   {0, \
                                      0, \
                                      0, \
                                      0, \
                                      0, \
                                      0, \
                                      0, \
                                      0, \
                                      0,\
                                      0, \
                                      0, \
                                      0, \
                                      TRUE,\
                                    }


/*------------------------------------------------------------------------------
 SPEED_EST Macro Definition
------------------------------------------------------------------------------*/

/*           case 6: v.nexttheta = _IQ(0.0833333333333333)-v.ad; break;                      \
//            case 2: v.nexttheta = _IQ(0.916666666666667)-v.ad; break;                     \
//            case 3: v.nexttheta = _IQ(0.75)-v.ad; break;                                   \
//            case 1: v.nexttheta = _IQ(0.583333333333333)-v.ad; break;                      \
//            case 5: v.nexttheta = _IQ(0.416666666666667)-v.ad; break;                      \
//            case 4: v.nexttheta = _IQ(0.25)-v.ad; break;*/

#define SE_MACRO1(v)                                                                            \
        /*v.ad = _IQ(-0.05); \ this is MTPA approximation for Motor A*/\
        /*v.ad = DEG_30 + adv_angle; set second term = 0 at lower speed for filter-only, might have to tune to = -0.02 at higher speed */\
        /*Mark (Mar 23, 2025), the first 30-deg shift is to account for the 30 -deg mechanical shift in the motor hall placement*/\
        \
        if (v.SUflag == TRUE){                                                                  \
            v.dtheta = _IQ(0.166666666666667);                                                  \
        }else{                                                                                  \
            switch (v.HallGpio){                            \
            case 6: v.nexttheta = SEC5_NEXTANGLE; break;                      \
            case 4: v.nexttheta = SEC4_NEXTANGLE; break;                     \
            case 5: v.nexttheta = SEC6_NEXTANGLE; break;                                   \
            case 1: v.nexttheta = SEC2_NEXTANGLE; break;                      \
            case 3: v.nexttheta = SEC3_NEXTANGLE; break;                      \
            case 2: v.nexttheta = SEC1_NEXTANGLE; break;                                   \
            }                                                                                   \
            v.dtheta=v.nexttheta+v.phic-v.currenttheta;                                                \
            if (v.dtheta<_IQ(0))                                                                \
                v.dtheta+=_IQ(1);                                                               \
        }                                                                                       \
     \
        if (v.HallGpio == 5)  \
            {v.Timestamp2 = v.Timestamp1;             \
             v.Timestamp1 = v.Timestamp;  \
             v.Period = v.Timestamp1 - v.Timestamp2;  \
            if (v.Period<0) \
            {v.Period += 32767; }} \
         if (v.EventPeriod < 0)                                  \
         {   v.EventPeriod += 32767;   /* 0x7FFF = 32767*/       \
            }  \
        v.realspeed = _IQdiv(v.speedscaler,v.Period);        \
        v.nextspeed=_IQdiv(v.speedscaler,v.EventPeriod);                                        \
        v.nextspeed=_IQmpy(v.nextspeed,v.dtheta);                                               \
        /*v.speedrpm = _IQmpy(v.realspeed,v.BaseRpm);*/  \
        v.speed_rads = _IQmpy(v.realspeed,_IQ(17));  \
        v.speed_rads = _IQdiv(v.speed_rads,_IQ(100)); \
        v.speed_rads = _IQmpy(v.speed_rads,_IQ(34)); \
        v.ad1 = _IQ(-0.052)+_IQmpy(_IQ(0.026),v.speed_rads); \
        v.ad1 = _IQsat(v.ad1,_IQ(0.076),_IQ(0));  \
      //  v.ad1 = _IQ(0.076);  \
        //v.speed_rads = _IQmpy(v.speed_rads,_IQ(23)); \
//        v.speed_rads = _IQmpy(_IQdiv((v.nextspeed+v.oldspeed),_IQ(2)),_IQ(35.44));  \
//        v.speed_rads = _IQmpy(v.speed_rads,_IQ(2)); /*after using theta for determining the sector, the speed calculated by speed.est is exactly 2 times larger than real speed, don't know why*/ \
//        v.speed_rads = _IQmpy(v.speed_rads,_IQ(35.44))/*actual angular speed!(rad/s) 35.44^2=2*PI*BASE_FREQ*/; \
//        v.oldspeed = v.nextspeed;  \
            case 6: v.nexttheta = _IQ(0.0833333333333333)/*_IQ(0.75)/*_IQ(0.916666666666667)*/-_IQ(-0.01); break;                      \
            case 2: v.nexttheta = _IQ(0.916666666666667)/*_IQ(0.583333333333333)/*_IQ(0.0833333333333333)*/-_IQ(-0.01); break;                     \
            case 3: v.nexttheta = _IQ(0.75)/*_IQ(0.25)*/-_IQ(-0.01); break;                                   \
            case 1: v.nexttheta = _IQ(0.583333333333333)/*_IQ(0.25)/*_IQ(0.416666666666667)*/-_IQ(-0.01); break;                      \
            case 5: v.nexttheta = _IQ(0.416666666666667)/*_IQ(0.583333333333333)*/-_IQ(-0.01); break;                      \
            case 4: v.nexttheta = _IQ(0.25)/*_IQ(0.916666666666667)/*_IQ(0.75)*/-_IQ(-0.01); break;


#endif /* SPEED_EST_1_H_ */
