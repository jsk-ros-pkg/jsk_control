#include <iostream>
#include <qpOASES.hpp>

using namespace qpOASES;
extern "C" {
  double* solve_qpoases_qp (double* ret,
                            double* eval_weight_matrix, double* eval_coeff_vector,
                            double* state_min_vector, double* state_max_vector,
                            double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                            int state_len, int inequality_len, PrintLevel print_level, double* ret_status) {
    real_t* H = new real_t[state_len*state_len];
    real_t* A = new real_t[inequality_len*state_len];
    real_t* g = new real_t[state_len];
    real_t* ub = new real_t[state_len];
    real_t* lb = new real_t[state_len];
    real_t* ubA = new real_t[inequality_len];
    real_t* lbA = new real_t[inequality_len];
    for (int i = 0; i < state_len*state_len; i++) {
      H[i] = eval_weight_matrix[i];
    }
    for (int i = 0; i < state_len; i++) {
      ub[i] = state_max_vector[i];
    }
    for (int i = 0; i < state_len; i++) {
      lb[i] = state_min_vector[i];
    }
    for (int i = 0; i < state_len; i++) {
      g[i] = eval_coeff_vector[i];
    }
    for (int i = 0; i < inequality_len; i++) {
      ubA[i] = inequality_max_vector[i];
    }
    for (int i = 0; i < inequality_len; i++) {
      lbA[i] = inequality_min_vector[i];
    }
    for (int i = 0; i < inequality_len*state_len; i++) {
      A[i] = inequality_matrix[i];
    }

    QProblem example( state_len,inequality_len );
    Options options;
    options.printLevel = print_level;
    example.setOptions( options );
    /* Solve first QP. */
    int nWSR = 1000;
    returnValue status = example.init( H,g,A,lb,ub,lbA,ubA, nWSR );
    //printf("%d %d %d\n", print_level, PL_NONE, PL_MEDIUM);
    ret_status[0] = getSimpleStatus(status, (print_level != PL_NONE)? BT_TRUE:BT_FALSE);
    /* Get and print solution of second QP. */
    real_t* xOpt = new real_t[state_len];
    example.getPrimalSolution( xOpt );
    for (int i = 0; i < state_len; i++) {
      ret[i] = xOpt[i];
    }
    delete[] H;
    delete[] A;
    delete[] g;
    delete[] ub;
    delete[] lb;
    delete[] ubA;
    delete[] lbA;
    delete[] xOpt;

    return ret;
  }

  double* solve_qpoases_lp (double* ret,
                            double* eval_coeff_vector,
                            double* state_min_vector, double* state_max_vector,
                            double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                            int state_len, int inequality_len, PrintLevel print_level, double* ret_status) {
    real_t* H = new real_t[state_len*state_len];
    real_t* A = new real_t[inequality_len*state_len];
    real_t* g = new real_t[state_len];
    real_t* ub = new real_t[state_len];
    real_t* lb = new real_t[state_len];
    real_t* ubA = new real_t[inequality_len];
    real_t* lbA = new real_t[inequality_len];

    for (int i = 0; i < state_len*state_len; i++) {
      H[i] = 0.0;
    }
    for (int i = 0; i < inequality_len*state_len; i++) {
      A[i] = inequality_matrix[i];
    }
    for (int i = 0; i < state_len; i++) {
      g[i] = eval_coeff_vector[i];
    }
    for (int i = 0; i < state_len; i++) {
      ub[i] = state_max_vector[i];
    }
    for (int i = 0; i < state_len; i++) {
      lb[i] = state_min_vector[i];
    }
    for (int i = 0; i < inequality_len; i++) {
      ubA[i] = inequality_max_vector[i];
    }
    for (int i = 0; i < inequality_len; i++) {
      lbA[i] = inequality_min_vector[i];
    }

    /* Setting up QProblem object with zero Hessian matrix. */
    QProblem example( state_len, inequality_len, HST_ZERO );

    Options options;
    options.printLevel = print_level;
    example.setOptions( options );

    /* Solve first LP. */
    int nWSR = 1000;
    returnValue status = example.init( H,g,A,lb,ub,lbA,ubA, nWSR);
    ret_status[0] = getSimpleStatus(status, (print_level != PL_NONE)? BT_TRUE:BT_FALSE);
    real_t* xOpt = new real_t[state_len];
    example.getPrimalSolution( xOpt );
    for (int i = 0; i < state_len; i++) {
      ret[i] = xOpt[i];
    }

    delete[] H;
    delete[] A;
    delete[] g;
    delete[] ub;
    delete[] lb;
    delete[] ubA;
    delete[] lbA;
    delete[] xOpt;

    return ret;
  }
}
