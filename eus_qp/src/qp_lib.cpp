
#include <iostream>
#include <Eigen/Dense>

#include "eiquadprog.hpp"

using namespace Eigen;

#define print(var)  \
  std::cout<<#var"= "<<std::endl<<var<<std::endl

int flag = -1 ;

extern "C" {
int get_constraints_check_flag(){
	return flag ;
}
}

extern "C" {
int check_constraints(double* CE, double* ce0, double* CI, double* ci0,
		double* x, int x_len, int ce_len, int ci_len, double eqthre, double* ce_err,
		double* ci_err) {
	int ret = ce_len + ci_len ;
	for (int i = 0; i < ce_len; i++) {
		for (int j = 0; j < x_len; j++) {
			ce_err[i] += CE[i * x_len + j] * x[j];
		}
		ce_err[i] += ce0[i];
		if ( ce_err[i] < eqthre && ce_err[i] > -eqthre ) ret-- ;
	}
	for (int i = 0; i < ci_len; i++) {
		for (int j = 0; j < x_len; j++) {
			ci_err[i] += CI[i * x_len + j] * x[j];
		}
		ci_err[i] += ci0[i];
		if ( ci_err[i] > -eqthre ) ret-- ;
	}
	return ret;
}
}


extern "C" {
double* solve_eiquadprog(double* G, double* g0, double* CE, double* ce0, double* CI,
		double* ci0, double* x,
		int x_len, int ce_len, int ci_len,
		double eqthre,
		int debug,
		double* ret_buf, double* ce_err, double* ci_err) {
	MatrixXd G_buf(x_len, x_len);
	Eigen::VectorXd g0_buf(x_len);
	MatrixXd CE_buf(x_len, ce_len);
	Eigen::VectorXd ce0_buf(ce_len);
	MatrixXd CI_buf(x_len, ci_len);
	Eigen::VectorXd ci0_buf(ci_len);
	Eigen::VectorXd x_buf(x_len);

	for ( int i=0 ; i<x_len ; i++ ){
		for ( int j=0 ; j<x_len ; j++ ){
			G_buf(i,j) = G[i+ x_len*j] ;
//			std::cout << "(" << i << "," << j << ")" ;
//			std::cout << " -" << i+x_len*j << "-" ;
//			std::cout << " -> " << G[i+ x_len*j] << std::endl ;
			if ( i == 0 ){
				g0_buf(j) = g0[j] ;
				x_buf(j) = x[j] ;
			}
		}
		for ( int j=0 ; j<ce_len ; j++ ){
			CE_buf(i,j) = CE[i+ x_len*j] ;
			if ( i == 0 ){
				ce0_buf(j) = ce0[j] ;
			}
		}
		for ( int j=0 ; j<ci_len ; j++ ){
			CI_buf(i,j) = CI[i+ x_len*j] ;
			if ( i == 0 ){
				ci0_buf(j) = ci0[j] ;
			}
		}
	}


	if (debug>1) {
		print(G_buf);
		print(g0_buf);
		print(CE_buf);
		print(ce0_buf);
		print(CI_buf);
		print(ci0_buf);
	}

	ret_buf[0] = solve_quadprog(G_buf, g0_buf, CE_buf, ce0_buf, CI_buf,
							ci0_buf, x_buf) ;
	for (int i = 0; i < x_buf.size(); i++)
				x[i] = x_buf(i) ;

	flag = check_constraints(CE, ce0, CI, ci0, x, x_len, ce_len, ci_len, eqthre, ce_err, ci_err);
	//flag = check_constraints(CE, ce0, CI, ci0, x, x_len, eqthre, ce_len, ci_len, ce_err, ci_err);
	if (debug>0) {
		std::cout << "[eus-eiquadprog]" << std::endl;
		std::cout << "  :minimized-object "
				<< ret_buf[0] << std::endl;
		std::cout << "  :optimal-state [";
		for (int i = 0; i < x_buf.size(); i++)
			std::cout << x[i]  << ' ';
		std::cout << "]" << std::endl;

		std::cout << "  :eq-constraint || " ;
		for ( int i=0 ; i <ce_len ; i++ ) std::cout << ce_err[i] << " " ;
		std::cout << "|| < " << eqthre ;
		std::cout << std::endl ;
		std::cout << "  :iq-constraint [" ;
		for ( int i=0 ; i <ci_len ; i++ ) std::cout << ci_err[i] << " " ;
		std::cout << "] > " << -eqthre ;
		std::cout << std::endl ;
		std::cout << "  :constraint-check " ;
		std::cout << flag  ;
		std::cout << std::endl ;
	}

	return x;
}
}
