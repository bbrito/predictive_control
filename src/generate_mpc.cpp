/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
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
 *    \file   examples/ocp/bioreactor.cpp
 *    \author Boris Houska, Filip Logist, Rien Quirynen
 *    \date   2014
 */
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>

/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     x,y,theta;
    Control               v,w   ;
    DifferentialEquation  f    ;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    f << dot(x) == v*cos(theta);
    f << dot(y) == v*sin(theta);
    f << dot(theta) == w;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( 0.0, 5.0, 50.0 );
    ocp.minimizeLagrangeTerm( (x-1)*(x-1)+(y-1)*(y-1)+(theta-1)*(theta-1)+v*v+w*w );  // weight this with the physical cost!!!
    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, x ==  2.0 );
    ocp.subjectTo( AT_START, y == 2.0 );
    ocp.subjectTo( AT_START, theta == 2.0 );

    ocp.subjectTo( -1.0 <= v <= 1.0 );
    ocp.subjectTo( -1.0 <= w <= 1.0 );


    // DEFINE A PLOT WINDOW:
    // ---------------------
    GnuplotWindow window;
    window.addSubplot( x ,"X"  );
    window.addSubplot( y ,"Y"  );
    window.addSubplot( theta ,"Theta"  );
    window.addSubplot( v,"V" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    /*OptimizationAlgorithm algorithm(ocp);
    //RealTimeAlgorithm algorithm(ocp);
    algorithm.set( HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE );
    algorithm.set(PRINTLEVEL, NONE);                       // default MEDIUM (NONE, MEDIUM, HIGH)
    algorithm.set(PRINT_SCP_METHOD_PROFILE, false);        // default false
    algorithm.set(PRINT_COPYRIGHT, false);                 // default true
    algorithm.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    Grid t(0,5.0,50);
    VariablesGrid s2(3,0,5.0,50),c2(2,0,5.0,50);
    DVector state_ini(4);
    state_ini.setAll(2.0);
    state_ini(0)=0;
    algorithm.initializeDifferentialStates(s2);
    algorithm.initializeControls          (c2);

    algorithm.set( MAX_NUM_ITERATIONS, 100 );
    algorithm.set( KKT_TOLERANCE, 1e-8 );
    algorithm << window;
    //algorithm.solve(0.0,state_ini);
    algorithm.solve();
    VariablesGrid s3,c3;
    algorithm.getDifferentialStates(s3);
    algorithm.getControls          (c3);*/

    // DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
    // ----------------------------------------------------------
    OCPexport mpc( ocp );

    mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN  		);
    mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING 	);
    mpc.set( INTEGRATOR_TYPE,             INT_RK4   			);
    mpc.set( NUM_INTEGRATOR_STEPS,        18            		);
    mpc.set( QP_SOLVER,                   QP_QPOASES    		);
    mpc.set( HOTSTART_QP,                 NO             		);
    mpc.set( GENERATE_TEST_FILE,          YES            		);
    mpc.set( GENERATE_MAKE_FILE,          YES            		);
    mpc.set( GENERATE_MATLAB_INTERFACE,   YES            		);
    mpc.set( SPARSE_QP_SOLUTION, 		  FULL_CONDENSING_N2	);
//    mpc.set( DYNAMIC_SENSITIVITY, 		  SYMMETRIC				);
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO 					);
    mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 				);

    mpc.exportCode( "generated_mpc" );
    mpc.printDimensionsQP( );
    // ----------------------------------------------------------
    return 0;
}
/* <<< end tutorial code <<< */