/*
 * Current_reconstruct2.h
 *
 *  Created on: April 26, 2024
 *      Author: Bruce
 */

#ifndef Current_reconstruct_H_
#define Current_reconstruct_H_

typedef struct {
                 _iq  ia;
                 _iq  ib;
                 _iq  ic;
                 Uint16 commutation;
                 _iq  ia_new;
                 _iq  ib_new;
                 _iq  ic_new;
               } CURRENTRECONSTRUCT;

typedef CURRENTRECONSTRUCT *CURRENTRECONSTRUCT_handle;

#define CURRENTRECONSTRUCT_DEFAULTS {                                                        \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                           }                                                        \

#define Current_recons(v)                                                               \
        switch (v.commutation){      \
          case 400: v.ia_new = -v.ib-v.ic; \
                  v.ib_new = v.ib; \
                  v.ic_new = v.ic;  break;\
          case 500: v.ic_new = -v.ib-v.ia; \
                  v.ia_new = v.ia; \
                  v.ib_new = v.ib;  break;\
          case 600: v.ic_new = -v.ib-v.ia; \
                  v.ib_new = v.ib; \
                  v.ia_new = v.ia;  break;\
          case 700: v.ib_new = -v.ia-v.ic; \
                  v.ia_new = v.ia; \
                  v.ic_new = v.ic;  break;\
          case 800: v.ib_new = -v.ia-v.ic; \
                  v.ia_new = v.ia; \
                  v.ic_new = v.ic;  break;\
          case 900: v.ia_new = -v.ib-v.ic; \
                  v.ib_new = v.ib; \
                  v.ic_new = v.ic;  break;\
          case 0: v.ia_new = v.ia; \
                  v.ib_new = v.ib; \
                  v.ic_new = v.ic;  break;\
        } \

#endif /* TORQUECAL_H_ */
