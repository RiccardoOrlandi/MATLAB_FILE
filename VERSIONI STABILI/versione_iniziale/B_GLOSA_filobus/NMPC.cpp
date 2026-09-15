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
    Control jerk;
    Control s_Vmax;
    Control s_Ay;
    Control s_Jmin;
    Control s_Jmax;
    OnlineData x_TL1; 
    OnlineData x_TL2; 
    OnlineData x_TL3; 
    OnlineData x_TL4; 
    OnlineData x_TL5; 
    OnlineData x_TL6; 
    OnlineData x_TL7; 
    OnlineData x_TL8; 
    OnlineData x_TL9; 
    OnlineData x_TL10; 
    OnlineData x_TL11; 
    OnlineData x_stop1; 
    OnlineData x_stop2; 
    OnlineData x_stop3; 
    OnlineData x_stop4; 
    OnlineData x_stop5; 
    OnlineData x_stop6; 
    OnlineData x_stop7; 
    OnlineData k_road; 
    OnlineData Vmin; 
    OnlineData Vmax; 
    OnlineData Ax_lim_mot; 
    OnlineData dt_schd; 
    OnlineData s_st; 
    OnlineData s_hor; 
    IntermediateState intS1 = vel;
    IntermediateState intS2 = acc;
    IntermediateState intS3 = jerk;
    IntermediateState intS4 = (5.20000000000000000000e+03-pos);
    IntermediateState intS5 = acc;
    IntermediateState intS6 = jerk;
    IntermediateState intS7 = ((5.00000000000000000000e-01+vel)/2.00000000000000000000e+00+1/2.00000000000000000000e+00*sqrt((1.00000000000000008180e-05+pow((5.00000000000000000000e-01-vel),2.00000000000000000000e+00))));
    IntermediateState intS8 = (-(-pos+s_st)/intS7+dt_schd)*(3.18309886183790691216e-01*atan((pos+s_hor-s_st)*1.00000000000000000000e+02)+5.00000000000000000000e-01)/1.80000000000000000000e+02;
    IntermediateState intS9 = s_Vmax;
    IntermediateState intS10 = s_Ay;
    IntermediateState intS11 = s_Jmax;
    IntermediateState intS12 = s_Jmin;
    BMatrix acadodata_M1;
    acadodata_M1.read( "NMPC_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "NMPC_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << 1/5.20000000000000000000e+03*intS4;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS6;
    acadodata_f1 << 1/1.50000000000000000000e+00*intS5;
    acadodata_f1 << intS8;
    acadodata_f1 << 1/1.38888888888888892836e+01*intS9;
    acadodata_f1 << 1/2.00000000000000000000e+00*intS10;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS11;
    acadodata_f1 << 1/(-5.00000000000000000000e-01)*intS12;
    Function acadodata_f2;
    acadodata_f2 << 1/5.20000000000000000000e+03*intS4;
    IntermediateState intS13 = (-5.00000000000000000000e-01+jerk-s_Jmax);
    IntermediateState intS14 = (-(-5.00000000000000000000e-01)+jerk-s_Jmin);
    IntermediateState intS15 = (-2.00000000000000000000e+00+k_road*pow(vel,2.00000000000000000000e+00)-s_Ay);
    IntermediateState intS16 = (-Ax_lim_mot+acc);
    IntermediateState intS17 = (-Vmax-s_Vmax+vel);
    IntermediateState intS18 = (-(1.00000000000000000000e+00-x_TL1)*4.55000000000000000000e+01-5.20000000000000000000e+03*x_TL1+pos);
    IntermediateState intS19 = (-(1.00000000000000000000e+00-x_TL2)*2.03199999999999988631e+02-5.20000000000000000000e+03*x_TL2+pos);
    IntermediateState intS20 = (-(1.00000000000000000000e+00-x_TL3)*3.84199999999999988631e+02-5.20000000000000000000e+03*x_TL3+pos);
    IntermediateState intS21 = (-(1.00000000000000000000e+00-x_TL4)*5.89399999999999977263e+02-5.20000000000000000000e+03*x_TL4+pos);
    IntermediateState intS22 = (-(1.00000000000000000000e+00-x_TL5)*7.73600000000000022737e+02-5.20000000000000000000e+03*x_TL5+pos);
    IntermediateState intS23 = (-(1.00000000000000000000e+00-x_TL6)*1.00420000000000004547e+03-5.20000000000000000000e+03*x_TL6+pos);
    IntermediateState intS24 = (-(1.00000000000000000000e+00-x_TL7)*1.22529999999999995453e+03-5.20000000000000000000e+03*x_TL7+pos);
    IntermediateState intS25 = (-(1.00000000000000000000e+00-x_TL8)*1.41970000000000004547e+03-5.20000000000000000000e+03*x_TL8+pos);
    IntermediateState intS26 = (-(1.00000000000000000000e+00-x_TL9)*1.50779999999999995453e+03-5.20000000000000000000e+03*x_TL9+pos);
    IntermediateState intS27 = (-(1.00000000000000000000e+00-x_TL10)*1.73900000000000000000e+03-5.20000000000000000000e+03*x_TL10+pos);
    IntermediateState intS28 = (-(1.00000000000000000000e+00-x_TL11)*1.82300000000000000000e+03-5.20000000000000000000e+03*x_TL11+pos);
    IntermediateState intS29 = (-(1.00000000000000000000e+00-x_stop1)*8.00000000000000000000e+01-5.20000000000000000000e+03*x_stop1+pos);
    IntermediateState intS30 = (-(1.00000000000000000000e+00-x_stop2)*4.47000000000000000000e+02-5.20000000000000000000e+03*x_stop2+pos);
    IntermediateState intS31 = (-(1.00000000000000000000e+00-x_stop3)*7.56000000000000000000e+02-5.20000000000000000000e+03*x_stop3+pos);
    IntermediateState intS32 = (-(1.00000000000000000000e+00-x_stop4)*1.08400000000000000000e+03-5.20000000000000000000e+03*x_stop4+pos);
    IntermediateState intS33 = (-(1.00000000000000000000e+00-x_stop5)*1.30400000000000000000e+03-5.20000000000000000000e+03*x_stop5+pos);
    IntermediateState intS34 = (-(1.00000000000000000000e+00-x_stop6)*1.50700000000000000000e+03-5.20000000000000000000e+03*x_stop6+pos);
    IntermediateState intS35 = (-(1.00000000000000000000e+00-x_stop7)*1.82200000000000000000e+03-5.20000000000000000000e+03*x_stop7+pos);
    OCP ocp1(0, 50, 50);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo(acc <= 1.50000000000000000000e+00);
    ocp1.subjectTo(acc >= (-1.50000000000000000000e+00));
    ocp1.subjectTo(intS13 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(s_Jmax >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS14 >= 0.00000000000000000000e+00);
    ocp1.subjectTo(s_Jmin >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS15 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(s_Ay >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS16 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS17 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(s_Vmax >= 0.00000000000000000000e+00);
    ocp1.subjectTo(vel >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS18 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS19 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS20 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS21 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS22 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS23 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS24 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS25 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS26 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS27 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS28 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS29 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS30 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS31 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS32 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS33 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS34 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS35 <= 0.00000000000000000000e+00);
    DifferentialEquation acadodata_f3;
    acadodata_f3 << dot(pos) == intS1;
    acadodata_f3 << dot(vel) == intS2;
    acadodata_f3 << dot(acc) == intS3;

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 5 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 25 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 500 );
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

