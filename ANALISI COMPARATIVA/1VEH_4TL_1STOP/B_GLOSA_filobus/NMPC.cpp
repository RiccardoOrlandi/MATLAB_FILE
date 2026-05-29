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
    IntermediateState intS1 = vel;
    IntermediateState intS2 = acc;
    IntermediateState intS3 = jerk;
    IntermediateState intS4 = (5.20000000000000000000e+03-pos);
    IntermediateState intS5 = jerk;
    IntermediateState intS6 = acc;
    IntermediateState intS7 = (pos-s_stop_active)/5.20000000000000000000e+03*w_stop_active;
    IntermediateState intS8 = (pos-s_stop_active)/5.00000000000000000000e-01*x_dwell_active;
    IntermediateState intS9 = 1/1.00000000000000005551e-01*vel*x_dwell_active;
    IntermediateState intS10 = 1/2.99999999999999988898e-01*acc*x_dwell_active;
    BMatrix acadodata_M1;
    acadodata_M1.read( "NMPC_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "NMPC_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << 1/5.20000000000000000000e+03*intS4;
    acadodata_f1 << 1/5.00000000000000000000e-01*intS5;
    acadodata_f1 << 1/1.50000000000000000000e+00*intS6;
    acadodata_f1 << intS7;
    acadodata_f1 << intS8;
    acadodata_f1 << intS9;
    acadodata_f1 << intS10;
    Function acadodata_f2;
    acadodata_f2 << 1/5.20000000000000000000e+03*intS4;
    IntermediateState intS11 = (-2.00000000000000000000e+00+k_road*vel*vel);
    IntermediateState intS12 = (-Vmax+vel);
    IntermediateState intS13 = (-(1.00000000000000000000e+00-x_stop_active)*s_stop_active-5.20000000000000000000e+03*x_stop_active+pos);
    IntermediateState intS14 = (-(1.00000000000000000000e+00-x_dwell_active)*5.20000000000000000000e+03-(5.00000000000000000000e-01+s_stop_active)*x_dwell_active+pos);
    IntermediateState intS15 = ((-5.00000000000000000000e-01+s_stop_active)*x_dwell_active-(1.00000000000000000000e+00-x_dwell_active)*5.20000000000000000000e+03-pos);
    IntermediateState intS16 = (-(1.00000000000000000000e+00-x_dwell_active)*1.38888888888888892836e+01-1.00000000000000005551e-01*x_dwell_active+vel);
    IntermediateState intS17 = (-(1.00000000000000000000e+00-x_dwell_active)*1.50000000000000000000e+00-2.99999999999999988898e-01*x_dwell_active+acc);
    IntermediateState intS18 = ((1.00000000000000000000e+00-x_dwell_active)*1.50000000000000000000e+00+2.99999999999999988898e-01*x_dwell_active+acc);
    IntermediateState intS19 = (-(1.00000000000000000000e+00-x_TL1)*s_TL1_active-5.20000000000000000000e+03*x_TL1+pos);
    IntermediateState intS20 = (-(1.00000000000000000000e+00-x_TL2)*s_TL2_active-5.20000000000000000000e+03*x_TL2+pos);
    IntermediateState intS21 = (-(1.00000000000000000000e+00-x_TL3)*s_TL3_active-5.20000000000000000000e+03*x_TL3+pos);
    IntermediateState intS22 = (-(1.00000000000000000000e+00-x_TL4)*s_TL4_active-5.20000000000000000000e+03*x_TL4+pos);
    IntermediateState intS23 = (-(1.00000000000000000000e+00-x_TL1_tail)*s_TL1_active-3.00000000000000000000e+01-5.20000000000000000000e+03*x_TL1_tail+pos);
    IntermediateState intS24 = (-(1.00000000000000000000e+00-x_TL2_tail)*s_TL2_active-3.00000000000000000000e+01-5.20000000000000000000e+03*x_TL2_tail+pos);
    IntermediateState intS25 = (-(1.00000000000000000000e+00-x_TL3_tail)*s_TL3_active-3.00000000000000000000e+01-5.20000000000000000000e+03*x_TL3_tail+pos);
    IntermediateState intS26 = (-(1.00000000000000000000e+00-x_TL4_tail)*s_TL4_active-3.00000000000000000000e+01-5.20000000000000000000e+03*x_TL4_tail+pos);
    OCP ocp1(0, 50, 50);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo(acc <= 1.50000000000000000000e+00);
    ocp1.subjectTo(acc >= (-1.50000000000000000000e+00));
    ocp1.subjectTo(jerk <= 5.00000000000000000000e-01);
    ocp1.subjectTo(jerk >= (-5.00000000000000000000e-01));
    ocp1.subjectTo(intS11 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS12 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(vel >= (-1.00000000000000002082e-02));
    ocp1.subjectTo(intS13 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS14 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS15 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS16 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS17 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS18 >= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS19 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS20 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS21 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS22 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS23 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS24 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS25 <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS26 <= 0.00000000000000000000e+00);
    DifferentialEquation acadodata_f3;
    acadodata_f3 << dot(pos) == intS1;
    acadodata_f3 << dot(vel) == intS2;
    acadodata_f3 << dot(acc) == intS3;

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 1 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 21 );
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
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_NMPC_R_1" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

