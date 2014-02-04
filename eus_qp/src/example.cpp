/* file exmaple.C

 This file is an example on how eiquadprog can be used
 by invoking solve_quadprog() function

 In order to compile this example, Eigen library must be installed
 on your system.
 
 The test problem is the following:
 
 Given:
 G =  2.1 0.0 1.0   g0^T = [6.0 1.0 1.0]
      1.5 2.2 0.0      
      1.2 1.3 3.1 
 Solve:
 min f(x) = 1/2 x G x + g0 x
 s.t.
   x_1 + 2*x_2 + x_3 + -4 = 0

   x_1 >= 0
   x_2 >= 0
   x_3 >= 0	
   -x_1 - x_2 >= -10
 
 The solution is x^T = [0 2 0] and f(x) = 6.4
 
 LICENSE
 
 Copyright (2010) Gael Guennebaud
 Copyright (2008) Angelo Furfaro
 


This file is a part of eiquadprog. 

uquadprog is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

uquadprog is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with uquadprog; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/




#include <iostream>
#include <Eigen/Dense>

#include "eiquadprog.hpp"
#include "qp_lib.cpp"

using namespace Eigen;

template<typename Vec, typename Mat> void foo() {
  Mat G(3,3); 
  Vec g0(3);
  Mat CE(3,1);
  Vec ce0(1);
  Mat CI(3,4); 
  Vec ci0(4);
  Vec x(3);

  
  G(0,0)=2.1; G(0,1)=0.0; G(0,2)=1.0;
  G(1,0)=1.5; G(1,1)=2.2; G(1,2)=0.0;
  G(2,0)=1.2; G(2,1)=1.3; G(2,2)=3.1;
  
  
  g0(0)=6.0; g0(1)=1.0; g0(2)=1.0;

  CE(0,0)=1.0;  
  CE(1,0)=2.0;  
  CE(2,0)=-1.0; 
  
  ce0(0)=-4;

  CI(0,0)=1.0; CI(0,1)=0.0;CI(0,2)=0.0; CI(0,3)=-1.0;
  CI(1,0)=0.0; CI(1,1)=1.0;CI(1,2)=0.0; CI(1,3)=-1.0;
  CI(2,0)=0.0; CI(2,1)=0.0;CI(2,2)=1.0; CI(2,3)=0.0;


  ci0(0)=0.0; ci0(1)=0.0;ci0(2)=0.0; ci0(3)=10.0;


  std::cout << "f: " ;
  std::cout << solve_quadprog(G, g0,  CE, ce0,  CI, ci0, x) ;
  std::cout << std::endl;
  std::cout << "x: ";
  for (int i = 0; i < x.size(); i++)
    std::cout << x(i) << ' ';
  std::cout << std::endl;
}

void bar() {
  double g0[3];
  double G[3*3] ;
  double CE[1*3];
  double ce0[1];
  double CI[4*3];
  double ci0[4];
  double x[3];
  double ret_buf[1];

  int x_len = 3;
  int ce_len = 1;
  int ci_len = 4;

  g0[0]=6.0; g0[1]=1.0; g0[2]=1.0;

  CE[0]=1.0;
  CE[1]=2.0;
  CE[2]=-1.0;

  ce0[0]=-4;

  G[0]=2.1; G[3]=0.0; G[6]=1.0;
  G[1]=1.5; G[4]=2.2; G[7]=0.0;
  G[2]=1.2; G[5]=1.3; G[8]=3.1;

  CI[0]=1.0; CI[3]=0.0; CI[6]=0.0; CI[9]=-1.0;
  CI[1]=0.0; CI[4]=1.0; CI[7]=0.0; CI[10]=-1.0;
  CI[2]=0.0; CI[5]=0.0; CI[8]=1.0; CI[11]=0.0;

  ci0[0]=0.0; ci0[1]=0.0; ci0[2]=0.0; ci0[3]=10.0;

  double ce_err[ce_len] ;
  double ci_err[ci_len] ;
  solve_eiquadprog(G, g0, CE, ce0, CI, ci0, x,
  		x_len, ce_len, ci_len, 1e-1,
  		2, ret_buf, ce_err, ci_err) ;
}

int main(int argc, char** argv){
  	
  foo<Eigen::VectorXd, MatrixXd>();
  std::cout << "----------------------------" << std::endl ;
  bar();
}

