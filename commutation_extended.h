/*
 * Commutation_extended.h
 *
 *  Created on: June,19 2024
 *      Author: Bruce
 */

#ifndef ComutationExt_H_
#define ComutationExt_H_

typedef struct {
                 _iq D;
                 _iq ad;
                 Uint16 i;
                }COMMUTATIONEXT;

typedef COMMUTATIONEXT *COMMUTATIONEXT_handle;

#define COMMUTATIONEXT_DEFAULTS {                                                 \
                                   _IQ(0),                                    \
                                   _IQ(0),       \
                                   0,                                               \
                                   }                                                        \

#define COMMUTATION_EXT(v)                    \
           if (v.D>_IQ(0.9))   \
                {  \
                  v.i++;   \
                }else    \
                {    \
                    v.i = 0;   \
                }    \
if (v.i >= 5)   \
{    \
    v.ad = v.ad + _IQ(0.1);   \
    v.i = 0;    \
}    \
{v.ad = _IQsat(v.ad, _IQ(0.08), _IQ(0.0)); } \


#endif /* ComutationDetec2_H_ */
