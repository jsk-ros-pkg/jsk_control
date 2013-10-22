#include <nlopt.hpp>
#include <nlopt.h>
#include <vector>
#include <iostream>

//#include "my_param.h"
//#include "eus_function.cpp"
#include "nlopt_solver.h"


int f(double* x, double* ret)
{
	ret[0] = sqrt(x[1]) ;
	return 0 ;
 }

int df(double* x, double* grad){
	grad[0] = 0.0;
	grad[1] = 0.5 / sqrt(x[1]);
	return 0;
}

int g1(double* x, double* ret)
{
	ret[0] = x[0] + x[1] - 10 ;
	return 0 ;
}

int dg1(double* x, double* grad) {
	grad[0] = grad[1] = 1 ;
	return 0;
}

int h1(double* x, double* ret) {
	double a = 2, b = 0;
	ret[0] = ((a * x[0] + b) * (a * x[0] + b) * (a * x[0] + b) - x[1]);
	return 0;
}

int dh1(double* x, double* grad) {
	double a = 2, b = 0;
	grad[0] = 3 * a * (a * x[0] + b) * (a * x[0] + b);
	grad[1] = -1.0;
	return 0;
}

int nop(double* x, double* grad) {
	return 0;
}

int main(){
	double x[2] = { 1, 9 };
	double x_min[2] = {0,0} ;
	double x_max[2] = {10,10} ;
	NLoptSolver nos(x,x_min,x_max,f,df,g1,dg1,h1,dh1,2,1,1,1e-16,1e-8,1e-4,-1,-1,Optimization::NLopt::COBYLA) ;
	//NLoptSolver nos(x,x_min,x_max,f,df,nop,nop,nop,nop,2,1,1,Optimization::NLopt::DIRECT) ;
	nos.output_result(nos.Optimize()) ;
	return 0 ;
}
