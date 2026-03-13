/*
 * FiringAngle.h
 *
 *  Created on: Feb 16, 2022
 *      Author: LabUser
 */

#ifndef FIRINGANGLE_H_
#define FIRINGANGLE_H_

typedef struct { _iq  Angle;
                 _iq  phiv;
                 _iq  phiv1;
                 _iq  phiv2;
                 _iq  phiv3;
                 _iq  Out;
               } FIREANGLE;

typedef FIREANGLE *FIREANGLE_handle;


#define FIREANGLE_DEFAULTS {0,        \
                            _IQ(-0.16),        \
                            _IQ(-0.16),        \
                            _IQ(-0.16),        \
                            _IQ(-0.16),        \
                            0,        \
                           }


#define FA_FILTER(v)                                \
                                                    \
    v.phiv=v.phiv1+v.phiv2+v.phiv3;                 \
    v.phiv=_IQdiv(v.phiv,_IQ(3));                   \
    v.phiv3=v.phiv2;                                \
    v.phiv2=v.phiv1;                                \

#define FA_MACRO(v)                                 \
                                                    \
/* Compute the angle rate */                        \
    v.Out = v.Angle + v.phiv;                       \
                                                    \
/* Saturate the angle rate within (0,1) */          \
    if (v.Out>=_IQ(1.0))                          \
        v.Out -= _IQ(1.0);                        \
    else if (v.Out<_IQ(0.0))                      \
        v.Out += _IQ(1.0);                        \




#endif /* FIRINGANGLE_H_ */
