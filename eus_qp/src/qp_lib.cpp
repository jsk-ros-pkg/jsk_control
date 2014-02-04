
#include <iostream>
#include <Eigen/Dense>

#include "eiquadprog.hpp"

using namespace Eigen;

#define print(var)  \
  std::cout<<#var"= "<<std::endl<<var<<std::endl

extern "C" {
double* solve_eiquadprog(double* G, double* g0, double* CE, double* ce0, double* CI,
		double* ci0, double* x,
		int x_len, int ce_len, int ci_len,
		int debug, double* ret_buf
		) {
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


	if (debug>0) {
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

	if (debug > 0){
		std::cout << "f: "
				<< ret_buf[0] << std::endl;
		std::cout << "x: ";
		for (int i = 0; i < x_buf.size(); i++)
			std::cout << x[i]  << ' ';
		std::cout << std::endl;
	}

	return x;
}
}
