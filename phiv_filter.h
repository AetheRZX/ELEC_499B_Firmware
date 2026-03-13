/*
 * phiv_filter.h
 *
 *  Created on: Apr 3, 2022
 *      Author: LabUser
 */

#ifndef PHIV_FILTER_H_
#define PHIV_FILTER_H_

typedef struct {  _iq x;
                  _iq x_1;
                  _iq y_1;
                  _iq y;
                  _iq K;
                } PHIV_FILTER;




typedef PHIV_FILTER   *PHIV_FILTER_handle;


/*-----------------------------------------------------------------------------
Default initalisation values
-----------------------------------------------------------------------------*/

#define PHIV_FILTER_DEFAULTS {               \
                           _IQ(0),           \
                           _IQ(0),           \
                           _IQ(0),           \
                           _IQ(0),           \
                           _IQ(0),           \
                          }



/*------------------------------------------------------------------------------
    PID_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PHIV_FILTER_MACRO(v)                                                                                            \
        v.y=_IQmpy(v.K,v.y_1)+v.x_1-_IQmpy(v.K,v.x_1);                                                                  \
        v.y_1=v.y;                                                                                                      \
        v.x_1=v.x;                                                                                                      \



#endif /* PHIV_FILTER_H_ */
