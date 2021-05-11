#include <iostream>
#include <qpOASES.hpp>
#include <map>

using namespace qpOASES;

// Solve QP/LP without hotstart, e.g., generate QProblem instance always.
double* solve_qpoases_qp_common (double* ret,
                                 double* eval_weight_matrix, double* eval_coeff_vector,
                                 double* state_min_vector, double* state_max_vector,
                                 double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                                 int state_len, int inequality_len, PrintLevel print_level, double* ret_status,
                                 bool solve_lp) {
  real_t* H = new real_t[state_len*state_len];
  real_t* A = new real_t[inequality_len*state_len];
  real_t* g = new real_t[state_len];
  real_t* ub = new real_t[state_len];
  real_t* lb = new real_t[state_len];
  real_t* ubA = new real_t[inequality_len];
  real_t* lbA = new real_t[inequality_len];
  if (!solve_lp) {
    for (int i = 0; i < state_len*state_len; i++) {
      H[i] = eval_weight_matrix[i];
    }
  } else {
    for (int i = 0; i < state_len*state_len; i++) {
      H[i] = 0.0;
    }
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

  QProblem example( state_len,inequality_len, (solve_lp ? HST_ZERO : HST_UNKNOWN));
  Options options;
  options.printLevel = print_level;
  example.setOptions( options );
  /* Solve first QP/LP. */
  int nWSR = 10000;
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

// Solve QP/LP with hotstart.
//   QProblem instance are re-used.
//   g, ub, lb, ubA, lbA can be updated.
//   H and A should be constant.
std::map<std::pair<int, int>, QProblem*> qp_map;
double* solve_qpoases_qp_with_hotstart_common (double* ret,
                                               double* eval_weight_matrix, double* eval_coeff_vector,
                                               double* state_min_vector, double* state_max_vector,
                                               double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                                               int state_len, int inequality_len, PrintLevel print_level, double* ret_status,
                                               bool solve_lp) {
  real_t* H = new real_t[state_len*state_len];
  real_t* A = new real_t[inequality_len*state_len];
  real_t* g = new real_t[state_len];
  real_t* ub = new real_t[state_len];
  real_t* lb = new real_t[state_len];
  real_t* ubA = new real_t[inequality_len];
  real_t* lbA = new real_t[inequality_len];
  if (!solve_lp) {
    for (int i = 0; i < state_len*state_len; i++) {
      H[i] = eval_weight_matrix[i];
    }
  } else {
    for (int i = 0; i < state_len*state_len; i++) {
      H[i] = 0.0;
    }
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

  QProblem* example;
  std::pair<int, int> tmp_pair(state_len, inequality_len);
  std::map<std::pair<int, int>, QProblem*>::iterator it = qp_map.find(tmp_pair);
  bool is_initial = (it == qp_map.end());
  if (is_initial) {
    example = new QProblem ( state_len,inequality_len, (solve_lp ? HST_ZERO : HST_UNKNOWN));
    qp_map.insert(std::pair<std::pair<int, int>, QProblem*>(tmp_pair, example));
  } else {
    example = it->second;
  }

  Options options;
  options.printLevel = print_level;
  example->setOptions( options );
  int nWSR = 10000;
  returnValue status;
  if (is_initial) {
    /* Solve first QP/LP. */
    status = example->init( H,g,A,lb,ub,lbA,ubA, nWSR );
  } else {
    /* Solve second QP/LP. */
    status = example->hotstart( g,lb,ub,lbA,ubA, nWSR );
  }
  //printf("qp [size = (%d,%d), pointer = %d, is_initial = %d, status = %d, solved = %d]\n", state_len, inequality_len, example, is_initial, status, example->isSolved());
  //printf("%d %d %d\n", print_level, PL_NONE, PL_MEDIUM);
  ret_status[0] = getSimpleStatus(status, (print_level != PL_NONE)? BT_TRUE:BT_FALSE);
  if (example->isSolved() == BT_FALSE && ret_status[0] == 0) ret_status[0] = -100; // Not qpOASES original, check isSolved(). For example, hotstart.
  /* Get and print solution of second QP/LP. */
  real_t* xOpt = new real_t[state_len];
  example->getPrimalSolution( xOpt );
  for (int i = 0; i < state_len; i++) {
    ret[i] = xOpt[i];
  }
  // Delete unsolved qp
  if ( ret_status[0] != 0 ) {
    delete qp_map[tmp_pair];
    qp_map.erase(tmp_pair);
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

// Solve Sequential QP/LP with hotstart.
//   SQProblem instance are re-used.
//   H, g, A, ub, lb, ubA, lbA can be updated.
std::map<std::pair<int, int>, SQProblem*> sqp_map;
double* solve_qpoases_sqp_with_hotstart_common (double* ret,
                                                double* eval_weight_matrix, double* eval_coeff_vector,
                                                double* state_min_vector, double* state_max_vector,
                                                double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                                                int state_len, int inequality_len, PrintLevel print_level, double* ret_status,
                                                bool solve_lp) {
  real_t* H = new real_t[state_len*state_len];
  real_t* A = new real_t[inequality_len*state_len];
  real_t* g = new real_t[state_len];
  real_t* ub = new real_t[state_len];
  real_t* lb = new real_t[state_len];
  real_t* ubA = new real_t[inequality_len];
  real_t* lbA = new real_t[inequality_len];
  if (!solve_lp) {
    for (int i = 0; i < state_len*state_len; i++) {
      H[i] = eval_weight_matrix[i];
    }
  } else {
    for (int i = 0; i < state_len*state_len; i++) {
      H[i] = 0.0;
    }
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

  SQProblem* example;
  std::pair<int, int> tmp_pair(state_len, inequality_len);
  std::map<std::pair<int, int>, SQProblem*>::iterator it = sqp_map.find(tmp_pair);
  bool is_initial = (it == sqp_map.end());
  if (is_initial) {
    example = new SQProblem ( state_len,inequality_len, (solve_lp ? HST_ZERO : HST_UNKNOWN));
    sqp_map.insert(std::pair<std::pair<int, int>, SQProblem*>(tmp_pair, example));
  } else {
    example = it->second;
  }

  Options options;
  options.printLevel = print_level;
  example->setOptions( options );
  int nWSR = 10000;
  returnValue status;
  if (is_initial) {
    /* Solve first QP/LP. */
    status = example->init( H,g,A,lb,ub,lbA,ubA, nWSR );
  } else {
    /* Solve second QP/LP. */
    status = example->hotstart( H,g,A,lb,ub,lbA,ubA, nWSR );
  }
  //printf("sqp [size = (%d,%d), pointer = %d, is_initial = %d, status = %d, solved = %d]\n", state_len, inequality_len, example, is_initial, status, example->isSolved());
  //printf("%d %d %d\n", print_level, PL_NONE, PL_MEDIUM);
  ret_status[0] = getSimpleStatus(status, (print_level != PL_NONE)? BT_TRUE:BT_FALSE);
  if (example->isSolved() == BT_FALSE && ret_status[0] == 0) ret_status[0] = -100; // Not qpOASES original, check isSolved(). For example, hotstart.
  /* Get and print solution of second QP/LP. */
  real_t* xOpt = new real_t[state_len];
  example->getPrimalSolution( xOpt );
  for (int i = 0; i < state_len; i++) {
    ret[i] = xOpt[i];
  }
  // Delete unsolved sqp
  if ( ret_status[0] != 0 ) {
    delete sqp_map[tmp_pair];
    sqp_map.erase(tmp_pair);
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

extern "C" {
  double* solve_qpoases_qp (double* ret,
                            double* eval_weight_matrix, double* eval_coeff_vector,
                            double* state_min_vector, double* state_max_vector,
                            double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                            int state_len, int inequality_len, PrintLevel print_level, double* ret_status) {
    solve_qpoases_qp_common(ret,
                            eval_weight_matrix, eval_coeff_vector,
                            state_min_vector, state_max_vector,
                            inequality_matrix, inequality_min_vector, inequality_max_vector,
                            state_len, inequality_len, print_level, ret_status, false);
  };

  double* solve_qpoases_lp (double* ret,
                            double* eval_coeff_vector,
                            double* state_min_vector, double* state_max_vector,
                            double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                            int state_len, int inequality_len, PrintLevel print_level, double* ret_status) {
    solve_qpoases_qp_common(ret,
                            NULL, eval_coeff_vector,
                            state_min_vector, state_max_vector,
                            inequality_matrix, inequality_min_vector, inequality_max_vector,
                            state_len, inequality_len, print_level, ret_status, true);
  };

  double* solve_qpoases_qp_with_hotstart (double* ret,
                                          double* eval_weight_matrix, double* eval_coeff_vector,
                                          double* state_min_vector, double* state_max_vector,
                                          double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                                          int state_len, int inequality_len, PrintLevel print_level, double* ret_status) {
    solve_qpoases_qp_with_hotstart_common(ret,
                                          eval_weight_matrix, eval_coeff_vector,
                                          state_min_vector, state_max_vector,
                                          inequality_matrix, inequality_min_vector, inequality_max_vector,
                                          state_len, inequality_len, print_level, ret_status, false);
  };

  double* solve_qpoases_lp_with_hotstart (double* ret,
                                          double* eval_coeff_vector,
                                          double* state_min_vector, double* state_max_vector,
                                          double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                                          int state_len, int inequality_len, PrintLevel print_level, double* ret_status) {
    solve_qpoases_qp_with_hotstart_common(ret,
                                          NULL, eval_coeff_vector,
                                          state_min_vector, state_max_vector,
                                          inequality_matrix, inequality_min_vector, inequality_max_vector,
                                          state_len, inequality_len, print_level, ret_status, true);
  };

  double* solve_qpoases_sqp_with_hotstart (double* ret,
                                           double* eval_weight_matrix, double* eval_coeff_vector,
                                           double* state_min_vector, double* state_max_vector,
                                           double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                                           int state_len, int inequality_len, PrintLevel print_level, double* ret_status) {
    solve_qpoases_sqp_with_hotstart_common(ret,
                                           eval_weight_matrix, eval_coeff_vector,
                                           state_min_vector, state_max_vector,
                                           inequality_matrix, inequality_min_vector, inequality_max_vector,
                                           state_len, inequality_len, print_level, ret_status, false);
  };

  double* solve_qpoases_slp_with_hotstart (double* ret,
                                           double* eval_coeff_vector,
                                           double* state_min_vector, double* state_max_vector,
                                           double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                                           int state_len, int inequality_len, PrintLevel print_level, double* ret_status) {
    solve_qpoases_sqp_with_hotstart_common(ret,
                                           NULL, eval_coeff_vector,
                                           state_min_vector, state_max_vector,
                                           inequality_matrix, inequality_min_vector, inequality_max_vector,
                                           state_len, inequality_len, print_level, ret_status, true);
  };
}
