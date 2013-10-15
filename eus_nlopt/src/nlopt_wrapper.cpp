#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <nlopt.h>

#include "nlopt_solver.h"
//#include "eus_function.cpp"

int result = 0;

extern "C"{
int get_result(){ return result+5 ; }
}

extern "C"{
double* optimize(double* x,
		const double* x_min,
		const double* x_max,
		int (*f)(double*,double*), int (*df)(double*,double*),
		int (*g)(double*,double*), int (*dg)(double*,double*),
		int (*h)(double*,double*), int (*dh)(double*,double*),
		int m_x, int m_g, int m_h,
		double ftol, double xtol, double eqthre, int log,
		Optimization::NLopt::Algorithm algorithm,
		double* fbuf, double* dfbuf, double* gbuf, double* dgbuf, double* hbuf, double* dhbuf) {
	NLoptSolver nos(x, x_min, x_max, f, df, g, dg, h, dh, m_x, m_g, m_h, ftol,xtol,eqthre,
			(Optimization::NLopt::Algorithm) algorithm);
	nos.fbuf = fbuf ; nos.dfbuf = dfbuf ;
	nos.gbuf = gbuf ; nos.dgbuf = dgbuf ;
	nos.hbuf = hbuf ; nos.dhbuf = dhbuf ;
	result = nos.Optimize();
	if ( log ) nos.output_result(result) ;
	return x;
}
}
