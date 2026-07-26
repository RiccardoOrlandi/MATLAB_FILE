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
    DifferentialState pos4;
    DifferentialState vel4;
    DifferentialState acc4;
    Control jerk;
    Control jerk2;
    Control jerk3;
    Control jerk4;
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
    OnlineData x_TL1_4; 
    OnlineData x_TL2_4; 
    OnlineData x_TL3_4; 
    OnlineData x_TL4_4; 
    OnlineData s_stop_active_4; 
    OnlineData x_stop_active_4; 
    OnlineData x_dwell_active_4; 
    OnlineData w_stop_active_4; 
    OnlineData k_road_4; 
    OnlineData Vmax_4; 
    OnlineData dt_schd_4; 
    OnlineData s_st_4; 
    OnlineData s_hor_4; 
    OnlineData x_TL1_tail_4; 
    OnlineData x_TL2_tail_4; 
    OnlineData x_TL3_tail_4; 
    OnlineData x_TL4_tail_4; 
    OnlineData s_TL1_active_4; 
    OnlineData s_TL2_active_4; 
    OnlineData s_TL3_active_4; 
    OnlineData s_TL4_active_4; 
    IntermediateState intS1 = vel;
    IntermediateState intS2 = acc;
    IntermediateState intS3 = jerk;
    IntermediateState intS4 = vel2;
    IntermediateState intS5 = acc2;
    IntermediateState intS6 = jerk2;
    IntermediateState intS7 = vel3;
    IntermediateState intS8 = acc3;
    IntermediateState intS9 = jerk3;
    IntermediateState intS10 = vel4;
    IntermediateState intS11 = acc4;
    IntermediateState intS12 = jerk4;
    IntermediateState intS13 = (5.20000000000000000000e+03-pos);
    IntermediateState intS14 = (5.20000000000000000000e+03-pos2);
    IntermediateState intS15 = (5.20000000000000000000e+03-pos3);
    IntermediateState intS16 = (5.20000000000000000000e+03-pos4);
    IntermediateState intS17 = jerk;
    IntermediateState intS18 = jerk2;
    IntermediateState intS19 = jerk3;
    IntermediateState intS20 = jerk4;
    IntermediateState intS21 = acc;
    IntermediateState intS22 = acc2;
    IntermediateState intS23 = acc3;
    IntermediateState intS24 = acc4;
    IntermediateState intS25 = (-3.00000000000000000000e+00-7.79999999999999982236e+00+pos-pos2);
    IntermediateState intS26 = (-3.00000000000000000000e+00-7.79999999999999982236e+00+pos-pos2)*(-3.00000000000000000000e+00-7.79999999999999982236e+00+pos-pos2);
    IntermediateState intS27 = (-3.00000000000000000000e+00-7.79999999999999982236e+00+pos2-pos3);
    IntermediateState intS28 = (-3.00000000000000000000e+00-7.79999999999999982236e+00+pos2-pos3)*(-3.00000000000000000000e+00-7.79999999999999982236e+00+pos2-pos3);
    IntermediateState intS29 = (-3.00000000000000000000e+00-7.79999999999999982236e+00+pos3-pos4);
    IntermediateState intS30 = (-3.00000000000000000000e+00-7.79999999999999982236e+00+pos3-pos4)*(-3.00000000000000000000e+00-7.79999999999999982236e+00+pos3-pos4);
    IntermediateState intS31 = (intS25-sqrt((1.00000000000000005551e-01+intS26)))/2.00000000000000000000e+00/3.00000000000000000000e+00;
    IntermediateState intS32 = (intS27-sqrt((1.00000000000000005551e-01+intS28)))/2.00000000000000000000e+00/3.00000000000000000000e+00;
    IntermediateState intS33 = (intS29-sqrt((1.00000000000000005551e-01+intS30)))/2.00000000000000000000e+00/3.00000000000000000000e+00;
    IntermediateState intS34 = (pos-s_stop_active)/5.20000000000000000000e+03*w_stop_active;
    IntermediateState intS35 = (pos2-s_stop_active_2)/5.20000000000000000000e+03*w_stop_active_2;
    IntermediateState intS36 = (pos3-s_stop_active_3)/5.20000000000000000000e+03*w_stop_active_3;
    IntermediateState intS37 = (pos4-s_stop_active_4)/5.20000000000000000000e+03*w_stop_active_4;
    IntermediateState intS38 = (pos-s_stop_active)/5.00000000000000000000e-01*x_dwell_active;
    IntermediateState intS39 = (pos2-s_stop_active_2)/5.00000000000000000000e-01*x_dwell_active_2;
    IntermediateState intS40 = (pos3-s_stop_active_3)/5.00000000000000000000e-01*x_dwell_active_3;
    IntermediateState intS41 = (pos4-s_stop_active_4)/5.00000000000000000000e-01*x_dwell_active_4;
    IntermediateState intS42 = 1/1.00000000000000005551e-01*vel*x_dwell_active;
    IntermediateState intS43 = 1/1.00000000000000005551e-01*vel2*x_dwell_active_2;
    IntermediateState intS44 = 1/1.00000000000000005551e-01*vel3*x_dwell_active_3;
    IntermediateState intS45 = 1/1.00000000000000005551e-01*vel4*x_dwell_active_4;
    IntermediateState intS46 = 1/2.99999999999999988898e-01*acc*x_dwell_active;
    IntermediateState intS47 = 1/2.99999999999999988898e-01*acc2*x_dwell_active_2;
    IntermediateState intS48 = 1/2.99999999999999988898e-01*acc3*x_dwell_active_3;
    IntermediateState intS49 = 1/2.99999999999999988898e-01*acc4*x_dwell_active_4;
    DVector acadodata_v1(45);
    acadodata_v1(0) = 0;
    acadodata_v1(1) = 1;
    acadodata_v1(2) = 2;
    acadodata_v1(3) = 3;
    acadodata_v1(4) = 4;
    acadodata_v1(5) = 5;
    acadodata_v1(6) = 6;
    acadodata_v1(7) = 7;
    acadodata_v1(8) = 8;
    acadodata_v1(9) = 9;
    acadodata_v1(10) = 10;
    acadodata_v1(11) = 11;
    acadodata_v1(12) = 12;
    acadodata_v1(13) = 13;
    acadodata_v1(14) = 14;
    acadodata_v1(15) = 1.520000E+01;
    acadodata_v1(16) = 1.640000E+01;
    acadodata_v1(17) = 1.760000E+01;
    acadodata_v1(18) = 1.880000E+01;
    acadodata_v1(19) = 20;
    acadodata_v1(20) = 2.120000E+01;
    acadodata_v1(21) = 2.240000E+01;
    acadodata_v1(22) = 2.360000E+01;
    acadodata_v1(23) = 2.480000E+01;
    acadodata_v1(24) = 26;
    acadodata_v1(25) = 2.720000E+01;
    acadodata_v1(26) = 2.840000E+01;
    acadodata_v1(27) = 2.960000E+01;
    acadodata_v1(28) = 3.080000E+01;
    acadodata_v1(29) = 32;
    acadodata_v1(30) = 3.320000E+01;
    acadodata_v1(31) = 3.440000E+01;
    acadodata_v1(32) = 3.560000E+01;
    acadodata_v1(33) = 3.680000E+01;
    acadodata_v1(34) = 38;
    acadodata_v1(35) = 3.920000E+01;
    acadodata_v1(36) = 4.040000E+01;
    acadodata_v1(37) = 4.160000E+01;
    acadodata_v1(38) = 4.280000E+01;
    acadodata_v1(39) = 44;
    acadodata_v1(40) = 4.520000E+01;
    acadodata_v1(41) = 4.640000E+01;
    acadodata_v1(42) = 4.760000E+01;
    acadodata_v1(43) = 4.880000E+01;
    acadodata_v1(44) = 50;
    BMatrix acadodata_M1;
    acadodata_M1.read( "NMPC_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "NMPC_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << 1/5.20000000000000000000e+03*intS13;
    acadodata_f1 << 1/5.20000000000000000000e+03*intS14;
    acadodata_f1 << 1/5.20000000000000000000e+03*intS15;
    acadodata_f1 << 1/5.20000000000000000000e+03*intS16;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS17;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS18;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS19;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS20;
    acadodata_f1 << 1/1.50000000000000000000e+00*intS21;
    acadodata_f1 << 1/1.50000000000000000000e+00*intS22;
    acadodata_f1 << 1/1.50000000000000000000e+00*intS23;
    acadodata_f1 << 1/1.50000000000000000000e+00*intS24;
    acadodata_f1 << intS34;
    acadodata_f1 << intS35;
    acadodata_f1 << intS36;
    acadodata_f1 << intS37;
    acadodata_f1 << intS38;
    acadodata_f1 << intS39;
    acadodata_f1 << intS40;
    acadodata_f1 << intS41;
    acadodata_f1 << intS42;
    acadodata_f1 << intS43;
    acadodata_f1 << intS44;
    acadodata_f1 << intS45;
    acadodata_f1 << intS46;
    acadodata_f1 << intS47;
    acadodata_f1 << intS48;
    acadodata_f1 << intS49;
    acadodata_f1 << intS31;
    acadodata_f1 << intS32;
    acadodata_f1 << intS33;
    Function acadodata_f2;
    acadodata_f2 << 1/5.20000000000000000000e+03*intS13;
    acadodata_f2 << 1/5.20000000000000000000e+03*intS14;
    acadodata_f2 << 1/5.20000000000000000000e+03*intS15;
    acadodata_f2 << 1/5.20000000000000000000e+03*intS16;
    IntermediateState intS50 = (-2.00000000000000000000e+00+k_road*vel*vel);
    IntermediateState intS51 = (-2.00000000000000000000e+00+k_road_2*vel2*vel2);
    IntermediateState intS52 = (-2.00000000000000000000e+00+k_road_3*vel3*vel3);
    IntermediateState intS53 = (-2.00000000000000000000e+00+k_road_4*vel4*vel4);
    IntermediateState intS54 = (-Vmax+vel);
    IntermediateState intS55 = (-Vmax_2+vel2);
    IntermediateState intS56 = (-Vmax_3+vel3);
    IntermediateState intS57 = (-Vmax_4+vel4);
    IntermediateState intS58 = (-(1.00000000000000000000e+00-x_stop_active)*(1.00000000000000005551e-01+s_stop_active)-5.20000000000000000000e+03*x_stop_active+pos);
    IntermediateState intS59 = (-(1.00000000000000000000e+00-x_stop_active_2)*(1.00000000000000005551e-01+s_stop_active_2)-5.20000000000000000000e+03*x_stop_active_2+pos2);
    IntermediateState intS60 = (-(1.00000000000000000000e+00-x_stop_active_3)*(1.00000000000000005551e-01+s_stop_active_3)-5.20000000000000000000e+03*x_stop_active_3+pos3);
    IntermediateState intS61 = (-(1.00000000000000000000e+00-x_stop_active_4)*(1.00000000000000005551e-01+s_stop_active_4)-5.20000000000000000000e+03*x_stop_active_4+pos4);
    IntermediateState intS62 = (-(1.00000000000000000000e+00-x_dwell_active)*5.20000000000000000000e+03-(5.00000000000000000000e-01+s_stop_active)*x_dwell_active+pos);
    IntermediateState intS63 = ((-5.00000000000000000000e-01+s_stop_active)*x_dwell_active-(1.00000000000000000000e+00-x_dwell_active)*5.20000000000000000000e+03-pos);
    IntermediateState intS64 = (-(1.00000000000000000000e+00-x_dwell_active_2)*5.20000000000000000000e+03-(5.00000000000000000000e-01+s_stop_active_2)*x_dwell_active_2+pos2);
    IntermediateState intS65 = ((-5.00000000000000000000e-01+s_stop_active_2)*x_dwell_active_2-(1.00000000000000000000e+00-x_dwell_active_2)*5.20000000000000000000e+03-pos2);
    IntermediateState intS66 = (-(1.00000000000000000000e+00-x_dwell_active_3)*5.20000000000000000000e+03-(5.00000000000000000000e-01+s_stop_active_3)*x_dwell_active_3+pos3);
    IntermediateState intS67 = ((-5.00000000000000000000e-01+s_stop_active_3)*x_dwell_active_3-(1.00000000000000000000e+00-x_dwell_active_3)*5.20000000000000000000e+03-pos3);
    IntermediateState intS68 = (-(1.00000000000000000000e+00-x_dwell_active_4)*5.20000000000000000000e+03-(5.00000000000000000000e-01+s_stop_active_4)*x_dwell_active_4+pos4);
    IntermediateState intS69 = ((-5.00000000000000000000e-01+s_stop_active_4)*x_dwell_active_4-(1.00000000000000000000e+00-x_dwell_active_4)*5.20000000000000000000e+03-pos4);
    IntermediateState intS70 = (-(1.00000000000000000000e+00-x_dwell_active)*1.38888888888888892836e+01-1.00000000000000005551e-01*x_dwell_active+vel);
    IntermediateState intS71 = (-(1.00000000000000000000e+00-x_dwell_active)*1.50000000000000000000e+00-2.99999999999999988898e-01*x_dwell_active+acc);
    IntermediateState intS72 = ((1.00000000000000000000e+00-x_dwell_active)*1.50000000000000000000e+00+2.99999999999999988898e-01*x_dwell_active+acc);
    IntermediateState intS73 = (-(1.00000000000000000000e+00-x_dwell_active_2)*1.38888888888888892836e+01-1.00000000000000005551e-01*x_dwell_active_2+vel2);
    IntermediateState intS74 = (-(1.00000000000000000000e+00-x_dwell_active_2)*1.50000000000000000000e+00-2.99999999999999988898e-01*x_dwell_active_2+acc2);
    IntermediateState intS75 = ((1.00000000000000000000e+00-x_dwell_active_2)*1.50000000000000000000e+00+2.99999999999999988898e-01*x_dwell_active_2+acc2);
    IntermediateState intS76 = (-(1.00000000000000000000e+00-x_dwell_active_3)*1.38888888888888892836e+01-1.00000000000000005551e-01*x_dwell_active_3+vel3);
    IntermediateState intS77 = (-(1.00000000000000000000e+00-x_dwell_active_3)*1.50000000000000000000e+00-2.99999999999999988898e-01*x_dwell_active_3+acc3);
    IntermediateState intS78 = ((1.00000000000000000000e+00-x_dwell_active_3)*1.50000000000000000000e+00+2.99999999999999988898e-01*x_dwell_active_3+acc3);
    IntermediateState intS79 = (-(1.00000000000000000000e+00-x_dwell_active_4)*1.38888888888888892836e+01-1.00000000000000005551e-01*x_dwell_active_4+vel4);
    IntermediateState intS80 = (-(1.00000000000000000000e+00-x_dwell_active_4)*1.50000000000000000000e+00-2.99999999999999988898e-01*x_dwell_active_4+acc4);
    IntermediateState intS81 = ((1.00000000000000000000e+00-x_dwell_active_4)*1.50000000000000000000e+00+2.99999999999999988898e-01*x_dwell_active_4+acc4);
    IntermediateState intS82 = (-(1.00000000000000000000e+00-x_TL1)*s_TL1_active-5.20000000000000000000e+03*x_TL1+pos);
    IntermediateState intS83 = (-(1.00000000000000000000e+00-x_TL2)*s_TL2_active-5.20000000000000000000e+03*x_TL2+pos);
    IntermediateState intS84 = (-(1.00000000000000000000e+00-x_TL3)*s_TL3_active-5.20000000000000000000e+03*x_TL3+pos);
    IntermediateState intS85 = (-(1.00000000000000000000e+00-x_TL4)*s_TL4_active-5.20000000000000000000e+03*x_TL4+pos);
    IntermediateState intS86 = (-(1.00000000000000000000e+00-x_TL1_tail)*s_TL1_active-5.20000000000000000000e+03*x_TL1_tail-7.79999999999999982236e+00+pos);
    IntermediateState intS87 = (-(1.00000000000000000000e+00-x_TL2_tail)*s_TL2_active-5.20000000000000000000e+03*x_TL2_tail-7.79999999999999982236e+00+pos);
    IntermediateState intS88 = (-(1.00000000000000000000e+00-x_TL3_tail)*s_TL3_active-5.20000000000000000000e+03*x_TL3_tail-7.79999999999999982236e+00+pos);
    IntermediateState intS89 = (-(1.00000000000000000000e+00-x_TL4_tail)*s_TL4_active-5.20000000000000000000e+03*x_TL4_tail-7.79999999999999982236e+00+pos);
    IntermediateState intS90 = (-(1.00000000000000000000e+00-x_TL1_2)*s_TL1_active_2-5.20000000000000000000e+03*x_TL1_2+pos2);
    IntermediateState intS91 = (-(1.00000000000000000000e+00-x_TL2_2)*s_TL2_active_2-5.20000000000000000000e+03*x_TL2_2+pos2);
    IntermediateState intS92 = (-(1.00000000000000000000e+00-x_TL3_2)*s_TL3_active_2-5.20000000000000000000e+03*x_TL3_2+pos2);
    IntermediateState intS93 = (-(1.00000000000000000000e+00-x_TL4_2)*s_TL4_active_2-5.20000000000000000000e+03*x_TL4_2+pos2);
    IntermediateState intS94 = (-(1.00000000000000000000e+00-x_TL1_tail_2)*s_TL1_active_2-5.20000000000000000000e+03*x_TL1_tail_2-7.79999999999999982236e+00+pos2);
    IntermediateState intS95 = (-(1.00000000000000000000e+00-x_TL2_tail_2)*s_TL2_active_2-5.20000000000000000000e+03*x_TL2_tail_2-7.79999999999999982236e+00+pos2);
    IntermediateState intS96 = (-(1.00000000000000000000e+00-x_TL3_tail_2)*s_TL3_active_2-5.20000000000000000000e+03*x_TL3_tail_2-7.79999999999999982236e+00+pos2);
    IntermediateState intS97 = (-(1.00000000000000000000e+00-x_TL4_tail_2)*s_TL4_active_2-5.20000000000000000000e+03*x_TL4_tail_2-7.79999999999999982236e+00+pos2);
    IntermediateState intS98 = (-(1.00000000000000000000e+00-x_TL1_3)*s_TL1_active_3-5.20000000000000000000e+03*x_TL1_3+pos3);
    IntermediateState intS99 = (-(1.00000000000000000000e+00-x_TL2_3)*s_TL2_active_3-5.20000000000000000000e+03*x_TL2_3+pos3);
    IntermediateState intS100 = (-(1.00000000000000000000e+00-x_TL3_3)*s_TL3_active_3-5.20000000000000000000e+03*x_TL3_3+pos3);
    IntermediateState intS101 = (-(1.00000000000000000000e+00-x_TL4_3)*s_TL4_active_3-5.20000000000000000000e+03*x_TL4_3+pos3);
    IntermediateState intS102 = (-(1.00000000000000000000e+00-x_TL1_tail_3)*s_TL1_active_3-5.20000000000000000000e+03*x_TL1_tail_3-7.79999999999999982236e+00+pos3);
    IntermediateState intS103 = (-(1.00000000000000000000e+00-x_TL2_tail_3)*s_TL2_active_3-5.20000000000000000000e+03*x_TL2_tail_3-7.79999999999999982236e+00+pos3);
    IntermediateState intS104 = (-(1.00000000000000000000e+00-x_TL3_tail_3)*s_TL3_active_3-5.20000000000000000000e+03*x_TL3_tail_3-7.79999999999999982236e+00+pos3);
    IntermediateState intS105 = (-(1.00000000000000000000e+00-x_TL4_tail_3)*s_TL4_active_3-5.20000000000000000000e+03*x_TL4_tail_3-7.79999999999999982236e+00+pos3);
    IntermediateState intS106 = (-(1.00000000000000000000e+00-x_TL1_4)*s_TL1_active_4-5.20000000000000000000e+03*x_TL1_4+pos4);
    IntermediateState intS107 = (-(1.00000000000000000000e+00-x_TL2_4)*s_TL2_active_4-5.20000000000000000000e+03*x_TL2_4+pos4);
    IntermediateState intS108 = (-(1.00000000000000000000e+00-x_TL3_4)*s_TL3_active_4-5.20000000000000000000e+03*x_TL3_4+pos4);
    IntermediateState intS109 = (-(1.00000000000000000000e+00-x_TL4_4)*s_TL4_active_4-5.20000000000000000000e+03*x_TL4_4+pos4);
    IntermediateState intS110 = (-(1.00000000000000000000e+00-x_TL1_tail_4)*s_TL1_active_4-5.20000000000000000000e+03*x_TL1_tail_4-7.79999999999999982236e+00+pos4);
    IntermediateState intS111 = (-(1.00000000000000000000e+00-x_TL2_tail_4)*s_TL2_active_4-5.20000000000000000000e+03*x_TL2_tail_4-7.79999999999999982236e+00+pos4);
    IntermediateState intS112 = (-(1.00000000000000000000e+00-x_TL3_tail_4)*s_TL3_active_4-5.20000000000000000000e+03*x_TL3_tail_4-7.79999999999999982236e+00+pos4);
    IntermediateState intS113 = (-(1.00000000000000000000e+00-x_TL4_tail_4)*s_TL4_active_4-5.20000000000000000000e+03*x_TL4_tail_4-7.79999999999999982236e+00+pos4);
    Grid grid_acadodata_v1(acadodata_v1);
    OCP ocp1(grid_acadodata_v1);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo(acc <= 1.50000000000000000000e+00);
    ocp1.subjectTo(acc >= (-1.50000000000000000000e+00));
    ocp1.subjectTo(acc2 <= 1.50000000000000000000e+00);
    ocp1.subjectTo(acc2 >= (-1.50000000000000000000e+00));
    ocp1.subjectTo(acc3 <= 1.50000000000000000000e+00);
    ocp1.subjectTo(acc3 >= (-1.50000000000000000000e+00));
    ocp1.subjectTo(acc4 <= 1.50000000000000000000e+00);
    ocp1.subjectTo(acc4 >= (-1.50000000000000000000e+00));
    ocp1.subjectTo(jerk <= 5.00000000000000000000e-01);
    ocp1.subjectTo(jerk >= (-5.00000000000000000000e-01));
    ocp1.subjectTo(jerk2 <= 5.00000000000000000000e-01);
    ocp1.subjectTo(jerk2 >= (-5.00000000000000000000e-01));
    ocp1.subjectTo(jerk3 <= 5.00000000000000000000e-01);
    ocp1.subjectTo(jerk3 >= (-5.00000000000000000000e-01));
    ocp1.subjectTo(jerk4 <= 5.00000000000000000000e-01);
    ocp1.subjectTo(jerk4 >= (-5.00000000000000000000e-01));
    ocp1.subjectTo(intS50 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS51 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS52 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS53 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS54 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS55 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS56 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS57 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(vel >= (-2.00000000000000004163e-02));
    ocp1.subjectTo(vel2 >= (-2.00000000000000004163e-02));
    ocp1.subjectTo(vel3 >= (-2.00000000000000004163e-02));
    ocp1.subjectTo(vel4 >= (-2.00000000000000004163e-02));
    ocp1.subjectTo((-7.79999999999999982236e+00+pos-pos2) >= 1.00000000000000000000e+00);
    ocp1.subjectTo((-7.79999999999999982236e+00+pos2-pos3) >= 1.00000000000000000000e+00);
    ocp1.subjectTo((-7.79999999999999982236e+00+pos3-pos4) >= 1.00000000000000000000e+00);
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
    ocp1.subjectTo(intS72 >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS73 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS74 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS75 >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS76 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS77 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS78 >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS79 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS80 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS81 >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS82 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS83 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS84 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS85 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS86 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS87 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS88 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS89 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS90 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS91 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS92 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS93 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS94 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS95 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS96 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS97 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS98 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS99 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS100 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS101 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS102 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS103 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS104 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS105 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS106 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS107 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS108 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS109 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS110 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS111 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS112 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS113 <= 0.00000000000000000000e+00);
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
    acadodata_f3 << dot(pos4) == intS10;
    acadodata_f3 << dot(vel4) == intS11;
    acadodata_f3 << dot(acc4) == intS12;

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 4 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 84 );
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
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 44 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES3 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( GENERATE_SIMULINK_INTERFACE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_SIMULINK_INTERFACE");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-04 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_NMPC_R_1" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

