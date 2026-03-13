/*
 * DutyRatioCal.h
 *
 *  Created on: Feb 29, 2024
 *      Author: Bruce
 */

#ifndef DutyCal_H_
#define DutyCal_H_

typedef struct {
                 _iq theta; //Input,for determining the sector
                 _iq speed; //Input, mechanical angular speed
                 _iq Te; // Input
                 _iq vdc; //Input DC-Voltage
                 _iq fup; //output
                 _iq fdown;//output
                 _iq fdown2;
                 _iq test1;
                 _iq ex;
                 Uint16 TP;
                 _iq ea;
                 _iq eb;
                 _iq ec;
               } DutyCal;

typedef DutyCal *DutyCal_handle;

#define DutyCal_DEFAULTS {  0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                            0,                                                      \
                           }                                                        \

#define DUTY_CAL(v)                                                               \
     v.test1 = -_IQmpy(v.Te, _IQ(RS)); \
     v.test1 = _IQdiv(v.test1,_IQ(LS)); \
     switch (v.TP){ \
     case 1: v.fdown = _IQmpy(_IQ(-6.845),v.speed)+v.test1;   /*three phase conduction*/ \
            /*v.fup = v.fdown + _IQdiv(_IQmpy(_IQ(0.074),v.vdc),_IQ(LS));*/ \
            v.fup = _IQdiv(_IQdiv(_IQmpy(v.ex,v.vdc),_IQ(0.5)),_IQ(LS))+v.fdown; \
     break; \
     case 0:v.fdown = -_IQmpy(_IQmpy(_IQ(2.611456),_IQ(0.002738)),v.speed); /*lambda^2*(1.616)^2*/  /*two phase conduction*/ \
            v.fdown = _IQdiv(v.fdown,_IQ(LS))+v.test1; \
            v.fup = v.fdown + _IQdiv(_IQmpy(_IQ(0.059792),v.vdc),_IQ(LS)); /*lambda*1.616*/ \
     break; }\

//     if (v.TP == 0) \
//     v.fup = v.fdown + _IQdiv(_IQmpy(_IQ(0.059792),v.vdc),_IQ(LS)); \
//     else \
//     v.fup = v.fdown + _IQdiv(_IQmpy(_IQ(0.074),v.vdc),_IQ(LS)); \


//     if ( _IQ(((v.theta >= _IQ(0.916666666666666667))&&(v.theta<_IQ(1)))||((v.theta < _IQ(0.0833333333333333333))&&(v.theta >= _IQ(0))))     )    \
//     {     \
//         v.ex = v.ea;   \
//         v.ey = v.ec;   \
//     }     \
//     if ( _IQ((v.theta >= _IQ(0.0833333333333333333))&&(v.theta < _IQ(0.25)))     )   \
//     {     \
//         v.ex = v.ea;   \
//         v.ey = v.eb;   \
//     }     \
//     if ( _IQ((v.theta >= _IQ(0.25))&&(v.theta < _IQ(0.416666666666666666667)))     )     \
//     {     \
//         v.ex = v.ec;   \
//         v.ey = v.eb;   \
//     }     \
//     if ( _IQ((v.theta >= _IQ(0.41666666666666666667))&&(v.theta < _IQ(0.583333333333333333)))     )  /*two phase c-a*/   \
//     {     \
//         v.ex = v.ec;   \
//         v.ey = v.ea;   \
//     }     \
//     if ( _IQ((v.theta >= _IQ(0.583333333333333333))&&(v.theta < _IQ(0.75)))     ) /*two phase b-a*/   \
//     {     \
//          v.ex = v.eb;   \
//          v.ey = v.ea;   \
//     }     \
//      if ( _IQ((v.theta >= _IQ(0.75))&&(v.theta < _IQ(0.916666666666666667)))     ) /*two phase b-c*/  \
//     {     \
//          v.ex = v.eb;   \
//          v.ey = v.ec;   \
//     }     \
//     v.fdown = -_IQdiv(_IQmpy((v.ex-v.ey),(v.ex-v.ey)), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
//     v.fup = _IQdiv(_IQmpy((v.ex-v.ey),(v.vdc-(v.ex-v.ey))), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
     /*v.fdown = _IQ(0) -_IQdiv( ((_IQmpy(v.ea,v.ea)+_IQmpy(v.eb,v.eb)+_IQmpy(v.ec,v.ec))),_IQmpy(v.speed,_IQ(LS)) ) + v.test1; \
     if( _IQ((v.theta < _IQ(0.16666666666667))&&(v.theta >= _IQ(0))) )           \
     {v.fup = _IQdiv(_IQmpy((-v.vdc),v.eb),_IQmpy(v.speed,_IQ(LS))) + v.fdown; \
      } \
     if( _IQ((v.theta < _IQ(0.33333333333333))&&(v.theta >= _IQ(0.16666666666667))) )    \
     {v.fup = _IQdiv(_IQmpy((v.vdc),v.ea),_IQmpy(v.speed,_IQ(LS))) + v.fdown; \
      }\
     if( _IQ((v.theta < _IQ(0.5))&&(v.theta >= _IQ(0.33333333333333))) )          \
     {v.fup = _IQdiv(_IQmpy((-v.vdc),v.ec),_IQmpy(v.speed,_IQ(LS))) + v.fdown; \
      }\
     if( _IQ((v.theta < _IQ(0.66666666666667))&&(v.theta >= _IQ(0.5))) )                                                \
     {v.fup = _IQdiv(_IQmpy((v.vdc),v.eb),_IQmpy(v.speed,_IQ(LS))) + v.fdown; \
      }\
     if( _IQ((v.theta < _IQ(0.83333333333333))&&(v.theta >= _IQ(0.66666666666667))) )                                                \
     {v.fup = _IQdiv(_IQmpy((-v.vdc),v.ea),_IQmpy(v.speed,_IQ(LS))) + v.fdown; \
       }\
     if( _IQ((v.theta < _IQ(1))&&(v.theta >= _IQ(0.83333333333333))) )                                                \
     {v.fup = _IQdiv(_IQmpy((v.vdc),v.ec),_IQmpy(v.speed,_IQ(LS))) + v.fdown; \
       }*/ \
//     if ( _IQ((v.theta >= _IQ(0.916666666666666667)-theta_ref3)&&(v.theta < _IQ(0.916666666666666667)+theta_ref3))     ) /*three phase ab-c*/   \
//     {   \
//           v.fdown = -_IQdiv( ((_IQmpy(v.ea,v.ea)+_IQmpy(v.eb,v.eb)+_IQmpy(v.ec,v.ec))),_IQmpy(v.speed,_IQ(LS)) ) + v.test1;    \
//           v.fup = _IQdiv(_IQmpy((-v.vdc),v.ec),_IQmpy(v.speed,_IQ(LS))) + v.fdown;      \
//     }   \
//     if ( _IQ(((v.theta >= _IQ(0.916666666666666667)+theta_ref3)&&(v.theta<_IQ(1)))||((v.theta < _IQ(0.0833333333333333333)-theta_ref3)&&(v.theta >= _IQ(1))))     ) /*two phase a-c*/   \
//     {    \
//           v.fdown = _IQdiv(_IQmpy((v.ea-v.ec),-(v.ea-v.ec)), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
//           v.fup = _IQdiv(_IQmpy((v.ea-v.ec),(v.vdc-(v.ea-v.ec))), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
//     }    \
//     if ( _IQ((v.theta >= _IQ(0.0833333333333333333)-theta_ref3)&&(v.theta < _IQ(0.0833333333333333333)+theta_ref3))     ) /*three phase a-bc*/   \
//     {   \
//           v.fdown = -_IQdiv( ((_IQmpy(v.ea,v.ea)+_IQmpy(v.eb,v.eb)+_IQmpy(v.ec,v.ec))),_IQmpy(v.speed,_IQ(LS)) ) + v.test1;    \
//           v.fup = _IQdiv(_IQmpy((v.vdc),v.ea),_IQmpy(v.speed,_IQ(LS))) + v.fdown;      \
//     }   \
//     if ( _IQ((v.theta >= _IQ(0.0833333333333333333)+theta_ref3)&&(v.theta < _IQ(0.25)-theta_ref3))     ) /*two phase a-b*/   \
//     {    \
//      v.fdown = _IQdiv(_IQmpy((v.ea-v.eb),-(v.ea-v.eb)), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
//      v.fup = _IQdiv(_IQmpy((v.ea-v.eb),(v.vdc-(v.ea-v.eb))), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
//     }    \
//     if ( _IQ((v.theta >= _IQ(0.25)-theta_ref3)&&(v.theta < _IQ(0.25)+theta_ref3))     ) /*three phase ac-b*/   \
//     {   \
//           v.fdown = -_IQdiv( ((_IQmpy(v.ea,v.ea)+_IQmpy(v.eb,v.eb)+_IQmpy(v.ec,v.ec))),_IQmpy(v.speed,_IQ(LS)) ) + v.test1;    \
//           v.fup = _IQdiv(_IQmpy((-v.vdc),v.eb),_IQmpy(v.speed,_IQ(LS))) + v.fdown;      \
//     }   \
//     if ( _IQ((v.theta >= _IQ(0.25)+theta_ref3)&&(v.theta < _IQ(0.416666666666666666667)-theta_ref3))     ) /*two phase c-b*/   \
//     {    \
//      v.fdown = _IQdiv(_IQmpy((v.ec-v.eb),-(v.ec-v.eb)), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
//      v.fup = _IQdiv(_IQmpy((v.ec-v.eb),(v.vdc-(v.ec-v.eb))), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
//     }    \
//     if ( _IQ((v.theta >= _IQ(0.41666666666666666667)-theta_ref3)&&(v.theta < _IQ(0.4166666666666666666667)+theta_ref3))     ) /*three phase c-ab*/   \
//     {   \
//           v.fdown = -_IQdiv( ((_IQmpy(v.ea,v.ea)+_IQmpy(v.eb,v.eb)+_IQmpy(v.ec,v.ec))),_IQmpy(v.speed,_IQ(LS)) ) + v.test1;    \
//           v.fup = _IQdiv(_IQmpy((v.vdc),v.ec),_IQmpy(v.speed,_IQ(LS))) + v.fdown;      \
//     }   \
//     if ( _IQ((v.theta >= _IQ(0.41666666666666666667)+theta_ref3)&&(v.theta < _IQ(0.583333333333333333)-theta_ref3))     ) /*two phase c-a*/   \
//     {    \
//      v.fdown = _IQdiv(_IQmpy((v.ec-v.ea),-(v.ec-v.ea)), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
//      v.fup = _IQdiv(_IQmpy((v.ec-v.ea),(v.vdc-(v.ec-v.ea))), _IQmpy(_IQ(2),_IQmpy(_IQ(LS),v.speed)) ) + v.test1;   \
//     }    \


#endif /* DutyCal_H_ */
