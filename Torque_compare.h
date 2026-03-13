/*
 * Torque_compare.h
 *
 *  Created on: Feb 29, 2024
 *      Author: Bruce
 */

#ifndef TORQUECOM_H_
#define TORQUECOM_H_

typedef struct { _iq  torque_ref;
                 _iq  torque;
                 Uint32  Out;
                 _iq torque_error;
               } TORQUECOM;

typedef TORQUECOM *TORQUECOM_handle;

#define TORQUECOM_DEFAULTS {                                                        \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                           }                                                        \

#define TORQUE_COM(v)                                                               \
    if (v.torque_ref > v.torque)                                                    \
    {                                                                               \
    v.Out = 1;                                                                      \
    }                                                                               \
    else                                                                            \
    {                                                                               \
    v.Out = 0;                                                                      \
    }                                                                               \
    v.torque_error = v.torque_ref - v.torque;                                       \

#endif /* TORQUECOM_H_ */
