/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState pos;
    DifferentialState vel;
    DifferentialState acc;
    DifferentialState pos2;
    DifferentialState vel2;
    DifferentialState acc2;
    DifferentialState pos3;
    DifferentialState vel3;
    DifferentialState acc3;
    Control jerk;
    Control jerk2;
    Control jerk3;
    OnlineData x_TL1; 
    OnlineData x_TL2; 
    OnlineData x_TL3; 
    OnlineData x_TL4; 
    OnlineData s_stop_active; 
    OnlineData x_stop_active; 
    OnlineData x_dwell_active; 
    OnlineData w_stop_active; 
    OnlineData k_road; 
    OnlineData Vmax; 
    OnlineData dt_schd; 
    OnlineData s_st; 
    OnlineData s_hor; 
    OnlineData x_TL1_tail; 
    OnlineData x_TL2_tail; 
    OnlineData x_TL3_tail; 
    OnlineData x_TL4_tail; 
    OnlineData s_TL1_active; 
    OnlineData s_TL2_active; 
    OnlineData s_TL3_active; 
    OnlineData s_TL4_active; 
    OnlineData x_TL1_2; 
    OnlineData x_TL2_2; 
    OnlineData x_TL3_2; 
    OnlineData x_TL4_2; 
    OnlineData s_stop_active_2; 
    OnlineData x_stop_active_2; 
    OnlineData x_dwell_active_2; 
    OnlineData w_stop_active_2; 
    OnlineData k_road_2; 
    OnlineData Vmax_2; 
    OnlineData dt_schd_2; 
    OnlineData s_st_2; 
    OnlineData s_hor_2; 
    OnlineData x_TL1_tail_2; 
    OnlineData x_TL2_tail_2; 
    OnlineData x_TL3_tail_2; 
    OnlineData x_TL4_tail_2; 
    OnlineData s_TL1_active_2; 
    OnlineData s_TL2_active_2; 
    OnlineData s_TL3_active_2; 
    OnlineData s_TL4_active_2; 
    OnlineData x_TL1_3; 
    OnlineData x_TL2_3; 
    OnlineData x_TL3_3; 
    OnlineData x_TL4_3; 
    OnlineData s_stop_active_3; 
    OnlineData x_stop_active_3; 
    OnlineData x_dwell_active_3; 
    OnlineData w_stop_active_3; 
    OnlineData k_road_3; 
    OnlineData Vmax_3; 
    OnlineData dt_schd_3; 
    OnlineData s_st_3; 
    OnlineData s_hor_3; 
    OnlineData x_TL1_tail_3; 
    OnlineData x_TL2_tail_3; 
    OnlineData x_TL3_tail_3; 
    OnlineData x_TL4_tail_3; 
    OnlineData s_TL1_active_3; 
    OnlineData s_TL2_active_3; 
    OnlineData s_TL3_active_3; 
    OnlineData s_TL4_active_3; 
    IntermediateState intS1 = vel;
    IntermediateState intS2 = acc;
    IntermediateState intS3 = jerk;
    IntermediateState intS4 = vel2;
    IntermediateState intS5 = acc2;
    IntermediateState intS6 = jerk2;
    IntermediateState intS7 = vel3;
    IntermediateState intS8 = acc3;
    IntermediateState intS9 = jerk3;
    IntermediateState intS10 = (5.20000000000000000000e+03-pos);
    IntermediateState intS11 = (5.20000000000000000000e+03-pos2);
    IntermediateState intS12 = (5.20000000000000000000e+03-pos3);
    IntermediateState intS13 = jerk;
    IntermediateState intS14 = jerk2;
    IntermediateState intS15 = jerk3;
    IntermediateState intS16 = acc;
    IntermediateState intS17 = acc2;
    IntermediateState intS18 = acc3;
    IntermediateState intS19 = (-1/3.14159265358979311600e+00*atan((-6.00000000000000000000e+00-7.79999999999999982236e+00+pos-pos2))+5.00000000000000000000e-01)*(-3.00000000000000000000e+00-7.79999999999999982236e+00+pos-pos2)/3.00000000000000000000e+00;
    IntermediateState intS20 = (-1/3.14159265358979311600e+00*atan((-6.00000000000000000000e+00-7.79999999999999982236e+00+pos2-pos3))+5.00000000000000000000e-01)*(-3.00000000000000000000e+00-7.79999999999999982236e+00+pos2-pos3)/3.00000000000000000000e+00;
    IntermediateState intS21 = (pos-s_stop_active)/5.20000000000000000000e+03*w_stop_active;
    IntermediateState intS22 = (pos2-s_stop_active_2)/5.20000000000000000000e+03*w_stop_active_2;
    IntermediateState intS23 = (pos3-s_stop_active_3)/5.20000000000000000000e+03*w_stop_active_3;
    IntermediateState intS24 = (pos-s_stop_active)/5.00000000000000000000e-01*x_dwell_active;
    IntermediateState intS25 = (pos2-s_stop_active_2)/5.00000000000000000000e-01*x_dwell_active_2;
    IntermediateState intS26 = (pos3-s_stop_active_3)/5.00000000000000000000e-01*x_dwell_active_3;
    IntermediateState intS27 = 1/1.00000000000000005551e-01*vel*x_dwell_active;
    IntermediateState intS28 = 1/1.00000000000000005551e-01*vel2*x_dwell_active_2;
    IntermediateState intS29 = 1/1.00000000000000005551e-01*vel3*x_dwell_active_3;
    IntermediateState intS30 = 1/2.99999999999999988898e-01*acc*x_dwell_active;
    IntermediateState intS31 = 1/2.99999999999999988898e-01*acc2*x_dwell_active_2;
    IntermediateState intS32 = 1/2.99999999999999988898e-01*acc3*x_dwell_active_3;
    BMatrix acadodata_M1;
    acadodata_M1.read( "NMPC_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "NMPC_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << 1/5.20000000000000000000e+03*intS10;
    acadodata_f1 << 1/5.20000000000000000000e+03*intS11;
    acadodata_f1 << 1/5.20000000000000000000e+03*intS12;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS13;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS14;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS15;
    acadodata_f1 << 1/1.50000000000000000000e+00*intS16;
    acadodata_f1 << 1/1.50000000000000000000e+00*intS17;
    acadodata_f1 << 1/1.50000000000000000000e+00*intS18;
    acadodata_f1 << intS21;
    acadodata_f1 << intS22;
    acadodata_f1 << intS23;
    acadodata_f1 << intS24;
    acadodata_f1 << intS25;
    acadodata_f1 << intS26;
    acadodata_f1 << intS27;
    acadodata_f1 << intS28;
    acadodata_f1 << intS29;
    acadodata_f1 << intS30;
    acadodata_f1 << intS31;
    acadodata_f1 << intS32;
    acadodata_f1 << intS19;
    acadodata_f1 << intS20;
    Function acadodata_f2;
    acadodata_f2 << 1/5.20000000000000000000e+03*intS10;
    acadodata_f2 << 1/5.20000000000000000000e+03*intS11;
    acadodata_f2 << 1/5.20000000000000000000e+03*intS12;
    IntermediateState intS33 = (-2.00000000000000000000e+00+k_road*vel*vel);
    IntermediateState intS34 = (-2.00000000000000000000e+00+k_road_2*vel2*vel2);
    IntermediateState intS35 = (-2.00000000000000000000e+00+k_road_3*vel3*vel3);
    IntermediateState intS36 = (-Vmax+vel);
    IntermediateState intS37 = (-Vmax_2+vel2);
    IntermediateState intS38 = (-Vmax_3+vel3);
    IntermediateState intS39 = (-(1.00000000000000000000e+00-x_stop_active)*s_stop_active-5.20000000000000000000e+03*x_stop_active+pos);
    IntermediateState intS40 = (-(1.00000000000000000000e+00-x_stop_active_2)*s_stop_active_2-5.20000000000000000000e+03*x_stop_active_2+pos2);
    IntermediateState intS41 = (-(1.00000000000000000000e+00-x_stop_active_3)*s_stop_active_3-5.20000000000000000000e+03*x_stop_active_3+pos3);
    IntermediateState intS42 = (-(1.00000000000000000000e+00-x_dwell_active)*5.20000000000000000000e+03-(5.00000000000000000000e-01+s_stop_active)*x_dwell_active+pos);
    IntermediateState intS43 = ((-5.00000000000000000000e-01+s_stop_active)*x_dwell_active-(1.00000000000000000000e+00-x_dwell_active)*5.20000000000000000000e+03-pos);
    IntermediateState intS44 = (-(1.00000000000000000000e+00-x_dwell_active_2)*5.20000000000000000000e+03-(5.00000000000000000000e-01+s_stop_active_2)*x_dwell_active_2+pos2);
    IntermediateState intS45 = ((-5.00000000000000000000e-01+s_stop_active_2)*x_dwell_active_2-(1.00000000000000000000e+00-x_dwell_active_2)*5.20000000000000000000e+03-pos2);
    IntermediateState intS46 = (-(1.00000000000000000000e+00-x_dwell_active_3)*5.20000000000000000000e+03-(5.00000000000000000000e-01+s_stop_active_3)*x_dwell_active_3+pos3);
    IntermediateState intS47 = ((-5.00000000000000000000e-01+s_stop_active_3)*x_dwell_active_3-(1.00000000000000000000e+00-x_dwell_active_3)*5.20000000000000000000e+03-pos3);
    IntermediateState intS48 = (-(1.00000000000000000000e+00-x_dwell_active)*1.38888888888888892836e+01-1.00000000000000005551e-01*x_dwell_active+vel);
    IntermediateState intS49 = (-(1.00000000000000000000e+00-x_dwell_active)*1.50000000000000000000e+00-2.99999999999999988898e-01*x_dwell_active+acc);
    IntermediateState intS50 = ((1.00000000000000000000e+00-x_dwell_active)*1.50000000000000000000e+00+2.99999999999999988898e-01*x_dwell_active+acc);
    IntermediateState intS51 = (-(1.00000000000000000000e+00-x_dwell_active_2)*1.38888888888888892836e+01-1.00000000000000005551e-01*x_dwell_active_2+vel2);
    IntermediateState intS52 = (-(1.00000000000000000000e+00-x_dwell_active_2)*1.50000000000000000000e+00-2.99999999999999988898e-01*x_dwell_active_2+acc2);
    IntermediateState intS53 = ((1.00000000000000000000e+00-x_dwell_active_2)*1.50000000000000000000e+00+2.99999999999999988898e-01*x_dwell_active_2+acc2);
    IntermediateState intS54 = (-(1.00000000000000000000e+00-x_dwell_active_3)*1.38888888888888892836e+01-1.00000000000000005551e-01*x_dwell_active_3+vel3);
    IntermediateState intS55 = (-(1.00000000000000000000e+00-x_dwell_active_3)*1.50000000000000000000e+00-2.99999999999999988898e-01*x_dwell_active_3+acc3);
    IntermediateState intS56 = ((1.00000000000000000000e+00-x_dwell_active_3)*1.50000000000000000000e+00+2.99999999999999988898e-01*x_dwell_active_3+acc3);
    IntermediateState intS57 = (-(1.00000000000000000000e+00-x_TL1)*s_TL1_active-5.20000000000000000000e+03*x_TL1+pos);
    IntermediateState intS58 = (-(1.00000000000000000000e+00-x_TL2)*s_TL2_active-5.20000000000000000000e+03*x_TL2+pos);
    IntermediateState intS59 = (-(1.00000000000000000000e+00-x_TL3)*s_TL3_active-5.20000000000000000000e+03*x_TL3+pos);
    IntermediateState intS60 = (-(1.00000000000000000000e+00-x_TL4)*s_TL4_active-5.20000000000000000000e+03*x_TL4+pos);
    IntermediateState intS61 = (-(1.00000000000000000000e+00-x_TL1_tail)*s_TL1_active-5.20000000000000000000e+03*x_TL1_tail-7.79999999999999982236e+00+pos);
    IntermediateState intS62 = (-(1.00000000000000000000e+00-x_TL2_tail)*s_TL2_active-5.20000000000000000000e+03*x_TL2_tail-7.79999999999999982236e+00+pos);
    IntermediateState intS63 = (-(1.00000000000000000000e+00-x_TL3_tail)*s_TL3_active-5.20000000000000000000e+03*x_TL3_tail-7.79999999999999982236e+00+pos);
    IntermediateState intS64 = (-(1.00000000000000000000e+00-x_TL4_tail)*s_TL4_active-5.20000000000000000000e+03*x_TL4_tail-7.79999999999999982236e+00+pos);
    IntermediateState intS65 = (-(1.00000000000000000000e+00-x_TL1_2)*s_TL1_active_2-5.20000000000000000000e+03*x_TL1_2+pos2);
    IntermediateState intS66 = (-(1.00000000000000000000e+00-x_TL2_2)*s_TL2_active_2-5.20000000000000000000e+03*x_TL2_2+pos2);
    IntermediateState intS67 = (-(1.00000000000000000000e+00-x_TL3_2)*s_TL3_active_2-5.20000000000000000000e+03*x_TL3_2+pos2);
    IntermediateState intS68 = (-(1.00000000000000000000e+00-x_TL4_2)*s_TL4_active_2-5.20000000000000000000e+03*x_TL4_2+pos2);
    IntermediateState intS69 = (-(1.00000000000000000000e+00-x_TL1_tail_2)*s_TL1_active_2-5.20000000000000000000e+03*x_TL1_tail_2-7.79999999999999982236e+00+pos2);
    IntermediateState intS70 = (-(1.00000000000000000000e+00-x_TL2_tail_2)*s_TL2_active_2-5.20000000000000000000e+03*x_TL2_tail_2-7.79999999999999982236e+00+pos2);
    IntermediateState intS71 = (-(1.00000000000000000000e+00-x_TL3_tail_2)*s_TL3_active_2-5.20000000000000000000e+03*x_TL3_tail_2-7.79999999999999982236e+00+pos2);
    IntermediateState intS72 = (-(1.00000000000000000000e+00-x_TL4_tail_2)*s_TL4_active_2-5.20000000000000000000e+03*x_TL4_tail_2-7.79999999999999982236e+00+pos2);
    IntermediateState intS73 = (-(1.00000000000000000000e+00-x_TL1_3)*s_TL1_active_3-5.20000000000000000000e+03*x_TL1_3+pos3);
    IntermediateState intS74 = (-(1.00000000000000000000e+00-x_TL2_3)*s_TL2_active_3-5.20000000000000000000e+03*x_TL2_3+pos3);
    IntermediateState intS75 = (-(1.00000000000000000000e+00-x_TL3_3)*s_TL3_active_3-5.20000000000000000000e+03*x_TL3_3+pos3);
    IntermediateState intS76 = (-(1.00000000000000000000e+00-x_TL4_3)*s_TL4_active_3-5.20000000000000000000e+03*x_TL4_3+pos3);
    IntermediateState intS77 = (-(1.00000000000000000000e+00-x_TL1_tail_3)*s_TL1_active_3-5.20000000000000000000e+03*x_TL1_tail_3-7.79999999999999982236e+00+pos3);
    IntermediateState intS78 = (-(1.00000000000000000000e+00-x_TL2_tail_3)*s_TL2_active_3-5.20000000000000000000e+03*x_TL2_tail_3-7.79999999999999982236e+00+pos3);
    IntermediateState intS79 = (-(1.00000000000000000000e+00-x_TL3_tail_3)*s_TL3_active_3-5.20000000000000000000e+03*x_TL3_tail_3-7.79999999999999982236e+00+pos3);
    IntermediateState intS80 = (-(1.00000000000000000000e+00-x_TL4_tail_3)*s_TL4_active_3-5.20000000000000000000e+03*x_TL4_tail_3-7.79999999999999982236e+00+pos3);
    OCP ocp1(0, 50, 50);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo(acc <= 1.50000000000000000000e+00);
    ocp1.subjectTo(acc >= (-1.50000000000000000000e+00));
    ocp1.subjectTo(acc2 <= 1.50000000000000000000e+00);
    ocp1.subjectTo(acc2 >= (-1.50000000000000000000e+00));
    ocp1.subjectTo(acc3 <= 1.50000000000000000000e+00);
    ocp1.subjectTo(acc3 >= (-1.50000000000000000000e+00));
    ocp1.subjectTo(jerk <= 5.00000000000000000000e-01);
    ocp1.subjectTo(jerk >= (-5.00000000000000000000e-01));
    ocp1.subjectTo(jerk2 <= 5.00000000000000000000e-01);
    ocp1.subjectTo(jerk2 >= (-5.00000000000000000000e-01));
    ocp1.subjectTo(jerk3 <= 5.00000000000000000000e-01);
    ocp1.subjectTo(jerk3 >= (-5.00000000000000000000e-01));
    ocp1.subjectTo(intS33 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS34 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS35 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS36 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS37 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS38 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(vel >= (-1.00000000000000002082e-02));
    ocp1.subjectTo(vel2 >= (-1.00000000000000002082e-02));
    ocp1.subjectTo(vel3 >= (-1.00000000000000002082e-02));
    ocp1.subjectTo((-7.79999999999999982236e+00+pos-pos2) >= 5.00000000000000000000e-01);
    ocp1.subjectTo((-7.79999999999999982236e+00+pos2-pos3) >= 5.00000000000000000000e-01);
    ocp1.subjectTo(intS39 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS40 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS41 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS42 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS43 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS44 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS45 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS46 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS47 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS48 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS49 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS50 >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS51 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS52 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS53 >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS54 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS55 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS56 >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS57 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS58 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS59 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS60 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS61 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS62 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS63 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS64 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS65 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS66 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS67 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS68 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS69 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS70 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS71 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS72 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS73 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS74 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS75 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS76 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS77 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS78 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS79 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS80 <= 0.00000000000000000000e+00);
    DifferentialEquation acadodata_f3;
    acadodata_f3 << dot(pos) == intS1;
    acadodata_f3 << dot(vel) == intS2;
    acadodata_f3 << dot(acc) == intS3;
    acadodata_f3 << dot(pos2) == intS4;
    acadodata_f3 << dot(vel2) == intS5;
    acadodata_f3 << dot(acc2) == intS6;
    acadodata_f3 << dot(pos3) == intS7;
    acadodata_f3 << dot(vel3) == intS8;
    acadodata_f3 << dot(acc3) == intS9;

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 3 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 63 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_RK4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES3 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( GENERATE_SIMULINK_INTERFACE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_SIMULINK_INTERFACE");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-04 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( MAX_NUM_QP_ITERATIONS, 1400 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: MAX_NUM_QP_ITERATIONS");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_NMPC_R_1" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

