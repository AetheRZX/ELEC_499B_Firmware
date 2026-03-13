/*
 * id_mean.h
 *
 *  Created on: Feb 18, 2022
 *      Author: LabUser
 */

#ifndef ID_MEAN_H_
#define ID_MEAN_H_

//#define TWO_THIRD _IQ(0.666666666666667)

typedef struct {  _iq Angle;
                  _iq As;
                  _iq Bs;
                  _iq Cs;
                  _iq Sine;
                  _iq Sine2;
                  _iq Sine3;
                  _iq Ds;
                } MEAN_TR;

typedef struct {  _iq idtemp;
                  Uint16 countertemp;
                  _iq ids;
                  _iq ids_storage[6];
                  _iq sector;
                  _iq prev_sector;
                } MEAN_DATA;


typedef struct {  MEAN_TR  trans;
                  MEAN_DATA data;
                } ID_MEAN;


typedef ID_MEAN   *MEAN_handle;


/*-----------------------------------------------------------------------------
Default initalisation values
-----------------------------------------------------------------------------*/

#define MEAN_TR_DEFAULTS {              \
                           0,           \
                           0,           \
                           0,           \
                           0,           \
                           0,           \
                           0,           \
                           0,           \
                           0,           \
                          }

#define MEAN_DATA_DEFAULTS {            \
                           _IQ(0.0),    \
                           1,    \
                           _IQ(0.0),    \
                           {_IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0)},\
                           _IQ(0.0),    \
                           _IQ(0.0),    \
                          }

#define MEAN_FILTER_DEFAULTS {          \
                           _IQ(0.0),    \
                           _IQ(0.0),    \
                           _IQ(0.0),    \
                           _IQ(0.0),    \
                          }


/*------------------------------------------------------------------------------
    PID_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define MEAN_MACRO(v)                                                                                                    \
        /*--------------------------------Transform--------------------------------------------------------------------*/\
        v.trans.Sine=_IQsinPU(v.trans.Angle);                                                                            \
        v.trans.Sine2=_IQmpy(_IQsinPU(v.trans.Angle),_IQ(-0.5))+_IQmpy(_IQcosPU(v.trans.Angle),_IQ(0.866025403784439));  \
        v.trans.Sine3=_IQmpy(_IQsinPU(v.trans.Angle),_IQ(-0.5))-_IQmpy(_IQcosPU(v.trans.Angle),_IQ(0.866025403784439));  \
        /*if (v.data.sector==_IQ(5)){                                                                                      \
            v.trans.Ds = _IQmpy(v.trans.As,(v.trans.Sine-v.trans.Sine2)) + _IQmpy(v.trans.Bs,(v.trans.Sine3-v.trans.Sine2));\
        } else if (v.data.sector==_IQ(3)){                                                                               \
            v.trans.Ds = _IQmpy(v.trans.As,(v.trans.Sine-v.trans.Sine3)) + _IQmpy(v.trans.Cs,(v.trans.Sine2-v.trans.Sine3));\
        } else if (v.data.sector==_IQ(1)){                                                                               \
            v.trans.Ds = _IQmpy(v.trans.Bs,(v.trans.Sine3-v.trans.Sine)) + _IQmpy(v.trans.Cs,(v.trans.Sine2-v.trans.Sine));\
        }        */                                                                                                        \
                                        \
        /*if (v.data.sector ==_IQ(4)){                                                                                      \
            v.trans.Ds = _IQmpy(v.trans.As,v.trans.Sine-v.trans.Sine2);                                                  \
        } else if (v.data.sector==_IQ(6)){                                                                               \
            v.trans.Ds = _IQmpy(v.trans.As,v.trans.Sine-v.trans.Sine3);                                                  \
        } else if (v.data.sector==_IQ(2)){                                                                               \
            v.trans.Ds = _IQmpy(v.trans.Bs,v.trans.Sine2-v.trans.Sine3);                                                 \
        } else if (v.data.sector==_IQ(3)){                                                                               \
            v.trans.Ds = _IQmpy(v.trans.Bs,v.trans.Sine2-v.trans.Sine);                                                  \
        } else if (v.data.sector==_IQ(1)){                                                                               \
            v.trans.Ds = _IQmpy(v.trans.Cs,v.trans.Sine3-v.trans.Sine);                                                  \
        } else if (v.data.sector==_IQ(5)){                                                                               \
            v.trans.Ds = _IQmpy(v.trans.Cs,v.trans.Sine3-v.trans.Sine2);   \
        }*/                                                                                                                \
        v.trans.Ds = _IQmpy(v.trans.As,_IQsinPU(v.trans.Angle))+_IQmpy(v.trans.Bs,_IQsinPU(v.trans.Angle-DEG_120))+_IQmpy(v.trans.Cs,_IQsinPU(v.trans.Angle+DEG_120));\
                                                                                                                         \
        /*--------------------------------Sum--------------------------------------------------------------------------*/\
        v.data.idtemp += v.trans.Ds;                                                                                       \
        v.data.countertemp += 1;                                                                                      \
                                                                                                                         \

#define MEAN_FILTER(v)                                                                                                   \
        v.data.ids_storage[0] = v.data.ids_storage[1];\
        v.data.ids_storage[1] = v.data.ids_storage[2];\
        v.data.ids_storage[2] = v.data.ids_storage[3];\
        v.data.ids_storage[3] = v.data.ids_storage[4];\
        v.data.ids_storage[4] = v.data.ids_storage[5];\
        v.data.ids_storage[5] = v.data.ids;\
        v.data.ids = _IQdiv(v.data.ids_storage[0]+v.data.ids_storage[1]+v.data.ids_storage[2]+v.data.ids_storage[3]+v.data.ids_storage[4]+v.data.ids_storage[5], _IQ(6));\
        /*v.filter.ids3=v.filter.ids2;                                                                                     \
        v.filter.ids2=v.filter.ids1;                                                                                     \
        v.filter.ids1=v.data.ids;                                                                                        \
        v.filter.Out=v.filter.ids1+v.filter.ids2+v.filter.ids3;                                                          \
        v.filter.Out=_IQdiv(v.filter.Out,_IQ(3));*/                                                                        \

#endif /* ID_MEAN_H_ */
