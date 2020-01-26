#include <osqp.h>
#include <map>
#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <iostream>
// Solve QP
// ref. example of OSQP:
// https://github.com/oxfordcontrol/osqp/blob/master/examples/osqp_demo.c
// https://osqp.org/docs/examples/setup-and-solve.html
double* solve_osqp_common (double* ret,
                           double* eval_weight_matrix, double* eval_coeff_vector,
                           double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                           int state_len, int inequality_len, int verbose, double* ret_status,
                           int eval_weight_matrix_sparce_given, double* eval_weight_matrix_sparce,
                           int inequality_matrix_sparce_given, double* inequality_matrix_sparce) {
  // Load problem data
  c_float *P_x;
  c_int P_nnz;
  c_int *P_i;
  c_int *P_p;
  c_float *q = new c_float[state_len];
  c_float *A_x;
  c_int A_nnz;
  c_int *A_i;
  c_int *A_p;
  c_float *l = new c_float[inequality_len];
  c_float *u = new c_float[inequality_len];

  if(!eval_weight_matrix_sparce_given){
    P_x = new c_float[state_len*state_len];
    P_nnz = state_len*state_len;
    P_i = new c_int[state_len*state_len];
    P_p = new c_int[state_len+1];

    // for文のindexのi,jは，i行j列を表す
    // ref. document of sparse matrix:
    // https://people.sc.fsu.edu/~jburkardt/data/cc/cc.html
    for (c_int i = 0; i < state_len; i++) {
      for (c_int j = 0; j < state_len; j++) {
        // make P_x symmetric
        P_x[state_len*j+i] = (eval_weight_matrix[state_len*i+j] + eval_weight_matrix[state_len*j+i]) / 2.0;
      }
    }
    for (c_int i = 0; i < state_len; i++) {
      for (c_int j = 0; j < state_len; j++) {
        P_i[state_len*j+i] = i;
      }
    }
    for (c_int j = 0; j < state_len+1; j++) {
      P_p[j] = state_len*j;
    }
  }else{
    std::vector<double> Psparse(state_len*state_len,0);
    for(size_t i = 0; i < state_len; i++){
        for(size_t j = 0; j < state_len; j++){
          // make Psparse symmetric
          Psparse[i*state_len+j] = (eval_weight_matrix_sparce[i*state_len+j]==0 || eval_weight_matrix_sparce[j*state_len+i]==0)? 0.0 : 1.0;
        }
    }
    P_nnz = Psparse.size() - std::count(Psparse.begin(), Psparse.end(), 0);
    P_x = new c_float[P_nnz];
    P_i = new c_int[P_nnz];
    P_p = new c_int[state_len+1];
    P_p[0] = 0;
    for (size_t j = 0; j < state_len; j++) {
      size_t num=0;
      for(size_t i = 0; i < state_len; i++){
        if(eval_weight_matrix_sparce[i*state_len+j]!=0){
          P_i[P_p[j]+num] = i;
          // make P_x symmetric
          P_x[P_p[j]+num] = (eval_weight_matrix[i*state_len+j] + eval_weight_matrix[j*state_len+i]) / 2.0;
          num++;
        }
      }
      P_p[j+1] = P_p[j] + num;
    }
  }
  for (c_int i = 0; i < state_len; i++) {
    q[i] = eval_coeff_vector[i];
  }
  if (!inequality_matrix_sparce_given){
    A_x = new c_float[state_len*inequality_len];
    A_nnz = state_len*inequality_len;
    A_i = new c_int[state_len*inequality_len];
    A_p = new c_int[state_len+1];

    for (c_int i = 0; i < inequality_len; i++) {
      for (c_int j = 0; j < state_len; j++) {
        A_x[inequality_len*j+i] = inequality_matrix[state_len*i+j];
      }
    }
    for (c_int i = 0; i < inequality_len; i++) {
      for (c_int j = 0; j < state_len; j++) {
        A_i[inequality_len*j+i] = i;
      }
    }
    for (c_int j = 0; j < state_len+1; j++) {
      A_p[j] = inequality_len*j;
    }
  }else{
    std::vector<double> Asparse(state_len*inequality_len,0);
    for(size_t i = 0; i < inequality_len; i++){
      for(size_t j = 0; j < state_len; j++){
        Asparse[i*state_len+j] = (inequality_matrix_sparce[i*state_len+j]==0)? 0.0 : 1.0;
      }
    }
    A_nnz = Asparse.size() - std::count(Asparse.begin(), Asparse.end(), 0);
    A_x = new c_float[A_nnz];
    A_i = new c_int[A_nnz];
    A_p = new c_int[state_len+1];
    A_p[0] = 0;
    for (size_t j = 0; j < state_len; j++) {
      size_t num=0;
      for(size_t i = 0; i < inequality_len; i++){
        if(Asparse[i*state_len+j]!=0){
          A_i[A_p[j]+num] = i;
          A_x[A_p[j]+num] = inequality_matrix[i*state_len+j];
          num++;
        }
      }
      A_p[j+1] = A_p[j] + num;
    }
  }
  for (c_int i = 0; i < inequality_len; i++) {
    l[i] = inequality_min_vector[i];
  }
  for (c_int i = 0; i < inequality_len; i++) {
    u[i] = inequality_max_vector[i];
  }

  // Problem settings
  OSQPSettings * settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));

  // Structures
  OSQPWorkspace * work = NULL;  // Workspace
  OSQPData * data = NULL;  // OSQPData

  // Populate data
  data = (OSQPData *)c_malloc(sizeof(OSQPData));
  data->n = state_len;
  data->m = inequality_len;
  csc* csc_P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
  data->P = csc_to_triu(csc_P);
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
  data->l = l;
  data->u = u;

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  //settings->rho = 1e-6;
  //settings->alpha = 0.1;
  settings->verbose = verbose;
  settings->max_iter = 4000;
  //settings->max_iter = 100000;
  settings->eps_abs = 1e-06; // improve accuracy
  settings->eps_rel = 1e-06; // improve accuracy
  settings->scaled_termination = true; // avoid too severe termination check
  settings->polish = true; // improve accuracy. but cause oscillatory solution when convex error
  //settings->delta = 1e-4; // in polish, too small delta causes non-convex error, too large delta causes failure(unsuccessful)

  // Setup workspace
  int setup_ret = osqp_setup(&work, data, settings);

  // Solve Problem
  if (setup_ret==0) {
    osqp_solve(work);
    for (c_int i = 0; i < state_len; i++) {
      ret[i] = work->solution->x[i];
    }
    *ret_status = (double)(work->info->status_val);
  } else {
    *ret_status = -100.0;
  }

  // Cleanup
  osqp_cleanup(work);
  csc_spfree(data->A); // free A_x, A_i, A_p
  csc_spfree(csc_P); // free P_x, P_i, P_p
  csc_spfree(data->P);
  c_free(data->q); // free q
  c_free(data->l); // free l
  c_free(data->u); // free u
  c_free(data);
  c_free(settings);

  return ret;
}

// Solve Sequential QP/LP with hotstart.
class osqp_solver {
public:
  osqp_solver(size_t _state_len,
              size_t _inequality_len,
              csc* _P,// have to be symmetric
              csc* _A,// have to be symmetric
              int verbose)
    : state_len(_state_len),
      inequality_len(_inequality_len),
      P(_P),
      q(new c_float[state_len]),
      A(_A),
      l(new c_float[inequality_len]),
      u(new c_float[inequality_len]),
      settings((OSQPSettings *)c_malloc(sizeof(OSQPSettings))),
      data((OSQPData *)c_malloc(sizeof(OSQPData))),
      work(NULL)
  {
    for (c_int i = 0; i < state_len; i++) {
      q[i] = 0.0;
    }

    for(size_t j = 0; j < inequality_len ; j++){
      l[j] = 0.0;
      u[j] = 0.0;
    }

    data->n = state_len;
    data->m = inequality_len;
    data->P = csc_to_triu(P);
    data->q = q;
    data->A = A;
    data->l = l;
    data->u = u;

    osqp_set_default_settings(settings);
    //settings->rho = 1e-6;
    //settings->alpha = 0.1;
    settings->verbose = verbose;
    settings->max_iter = 4000;
    //settings->max_iter = 100000;
    settings->eps_abs = 1e-06; // improve accuracy
    settings->eps_rel = 1e-06; // improve accuracy
    settings->scaled_termination = true; // avoid too severe termination check
    settings->polish = true; // improve accuracy. but cause oscillatory solution when convex error
    //settings->delta = 1e-4; // in polish, too small delta causes non-convex error, too large delta causes failure(unsuccessful)

    if(osqp_setup(&work, data, settings) == 0) initialized = true;
    else initialized = false;

    return;
  }

  ~osqp_solver(){
    osqp_cleanup(work);
    csc_spfree(P);
    csc_spfree(data->A); // free A
    csc_spfree(data->P);
    c_free(data->q); // free q
    c_free(data->l); // free l
    c_free(data->u); // free u
    c_free(data);
    c_free(settings);

  }

  double* solve(double* ret,
                double* eval_weight_matrix,
                double* eval_coeff_vector,
                double* inequality_matrix,
                double* inequality_min_vector,
                double* inequality_max_vector,
                int verbose,
                double* ret_status){
    // Load problem data
    {
      size_t n = 0;
      for(size_t j = 0; j < state_len; j++){
        for(size_t i = P->p[j]; i < P->p[j+1]; i++){
          P->x[n] = (eval_weight_matrix[P->i[i]*state_len+j]  + eval_weight_matrix[j*state_len+P->i[i]]) / 2.0;
          n++;
        }
      }
    }

    for (c_int i = 0; i < state_len; i++) {
      q[i] = eval_coeff_vector[i];
    }

    {
      size_t n = 0;
      for(size_t j = 0; j < state_len; j++){
        for(size_t i = A->p[j]; i < A->p[j+1]; i++){
          A->x[n] = inequality_matrix[A->i[i]*state_len+j];
          n++;
        }
      }
    }

    for(size_t j = 0; j < inequality_len ; j++){
      l[j] = inequality_min_vector[j];
      u[j] = inequality_max_vector[j];
    }

    csc *P_triu = csc_to_triu(P); // P should be upper requangle
    osqp_update_P_A(work, P_triu->x, OSQP_NULL, P_triu->p[P_triu->n] , A->x, OSQP_NULL, A->nzmax);
    c_free(P_triu);
    if(osqp_update_lin_cost(work,q) != 0){
      *ret_status = -100.0;
      return ret;
    }
    if(osqp_update_bounds(work,l,u) != 0){
      *ret_status = -100.0;
      return ret;
    }
    if(osqp_update_verbose(work,verbose) != 0){
      *ret_status = -100.0;
      return ret;
    }

    osqp_solve(work);
    for (c_int i = 0; i < state_len; i++) {
      ret[i] = work->solution->x[i];
    }
    *ret_status = (double)(work->info->status_val);
    return ret;
  }

  bool initialized;
private:
  size_t state_len;
  size_t inequality_len;

  csc *P;
  c_float *q;
  csc *A;
  c_float *l;
  c_float *u;

  OSQPData * data;
  OSQPSettings * settings;

  OSQPWorkspace* work;

};

std::vector<std::pair<std::pair<csc*, csc*>, boost::shared_ptr<osqp_solver> > > sqp_map;

// Solve SQP
double* solve_osqp_hotstart (double* ret,
                             double* eval_weight_matrix, double* eval_coeff_vector,
                             double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                             int state_len, int inequality_len, int verbose, double* ret_status,
                             int eval_weight_matrix_sparce_given, double* eval_weight_matrix_sparce,
                             int inequality_matrix_sparce_given, double* inequality_matrix_sparce) {
  boost::shared_ptr<osqp_solver> solver;

  std::vector<double> Psparse(state_len*state_len,0);
  if(!eval_weight_matrix_sparce_given){
    for(size_t i = 0; i < state_len; i++){
      for(size_t j = 0; j < state_len; j++){
        Psparse[i*state_len+j] = 1.0;
      }
    }
  }else{
    for(size_t i = 0; i < state_len; i++){
        for(size_t j = 0; j < state_len; j++){
          // make Psparse symmetric
          Psparse[i*state_len+j] = (eval_weight_matrix_sparce[i*state_len+j]==0 || eval_weight_matrix_sparce[j*state_len+i]==0)? 0.0 : 1.0;
        }
    }
  }
  c_int P_nnz(Psparse.size() - std::count(Psparse.begin(), Psparse.end(), 0));
  c_float *P_x(new c_float[P_nnz]);
  c_int *P_i(new c_int[P_nnz]);
  c_int *P_p(new c_int[state_len+1]);
  csc *P(csc_matrix(state_len, state_len, P_nnz, P_x, P_i, P_p));
  P_p[0] = 0;
  for (size_t j = 0; j < state_len; j++) {
    size_t num=0;
    for(size_t i = 0; i < state_len; i++){
      if(Psparse[i*state_len+j]!=0){
        P_i[P_p[j]+num] = i;
        P_x[P_p[j]+num] = 0.0;
        num++;
      }
    }
    P_p[j+1] = P_p[j] + num;
  }

  c_int A_nnz(0);
  if(!inequality_matrix_sparce_given){
    A_nnz = inequality_len * state_len;
  }else{
    for(size_t i = 0; i < inequality_len; i++){
      for(size_t j = 0; j < state_len; j++){
        if(inequality_matrix_sparce[i*state_len+j]!=0) A_nnz++;
      }
    }
  }
  c_float *A_x(new c_float[A_nnz]);
  c_int *A_i(new c_int[A_nnz]);
  c_int *A_p(new c_int[state_len+1]);
  csc *A(csc_matrix(inequality_len, state_len, A_nnz, A_x, A_i, A_p));
  A_p[0] = 0;
  if(!inequality_matrix_sparce_given){
    for (size_t j = 0; j < state_len; j++) {
      size_t num=0;
      for(size_t i = 0; i < inequality_len; i++){
        A_i[A_p[j]+num] = i;
        A_x[A_p[j]+num] = 0.0;
        num++;
      }
      A_p[j+1] = A_p[j] + num;
    }
  }else{
    for (size_t j = 0; j < state_len; j++) {
      size_t num=0;
      for(size_t i = 0; i < inequality_len; i++){
        if(inequality_matrix_sparce[i*state_len+j]!=0){
          A_i[A_p[j]+num] = i;
          A_x[A_p[j]+num] = 0.0;
          num++;
        }
      }
      A_p[j+1] = A_p[j] + num;
    }
  }

  std::pair<csc*, csc*> tmp_pair(P, A);

  std::vector<std::pair<std::pair<csc*, csc*>, boost::shared_ptr<osqp_solver> > >::iterator it;
  for(it = sqp_map.begin();it != sqp_map.end();it++){
    bool match = true;

    if(it->first.first->m != tmp_pair.first->m ||
       it->first.first->n != tmp_pair.first->n ||
       it->first.first->nzmax != tmp_pair.first->nzmax) continue;
    if(it->first.second->m != tmp_pair.second->m ||
       it->first.second->n != tmp_pair.second->n ||
       it->first.second->nzmax != tmp_pair.second->nzmax) continue;

    for(size_t i = 0; i < state_len+1; i++){
      if(it->first.first->p[i] != tmp_pair.first->p[i]){
        match = false;
        break;
      }
    }
    if(!match) continue;

    for(size_t i=0; i < it->first.first->nzmax; i++){
      if(it->first.first->i[i] != tmp_pair.first->i[i]){
        match = false;
        break;
      }
    }
    if(!match) continue;

    for(size_t i = 0; i < state_len+1; i++){
      if(it->first.second->p[i] != tmp_pair.second->p[i]){
        match = false;
        break;
      }
    }
    if(!match) continue;

    for(size_t i=0; i < it->first.second->nzmax; i++){
      if(it->first.second->i[i] != tmp_pair.second->i[i]){
        match = false;
        break;
      }
    }
    if(!match) continue;

    if(match)break;
  }
  if (it != sqp_map.end()) solver = it->second;

  if (!solver){
    solver = boost::shared_ptr<osqp_solver>(new osqp_solver(state_len,inequality_len,P,A,verbose));
    if(!solver->initialized){
      *ret_status = -100.0;
      return ret;
    }
    sqp_map.push_back(std::make_pair(tmp_pair, solver));
  }else{
    csc_spfree(P);
    csc_spfree(A);
  }

  return solver->solve(ret,
                       eval_weight_matrix,
                       eval_coeff_vector,
                       inequality_matrix,
                       inequality_min_vector,
                       inequality_max_vector,
                       verbose,
                       ret_status);
}


extern "C" {
  double* solve_osqp_qp (double* ret,
                         double* eval_weight_matrix, double* eval_coeff_vector,
                         double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                         int state_len, int inequality_len, int verbose, double* ret_status,
                         int eval_weight_matrix_sparce_given, double* eval_weight_matrix_sparce,
                         int inequality_matrix_sparce_given, double* inequality_matrix_sparce) {
    solve_osqp_common(ret,
                      eval_weight_matrix, eval_coeff_vector,
                      inequality_matrix, inequality_min_vector, inequality_max_vector,
                      state_len, inequality_len, verbose, ret_status,
                      eval_weight_matrix_sparce_given, eval_weight_matrix_sparce,
                      inequality_matrix_sparce_given, inequality_matrix_sparce);
  };

  double* solve_osqp_sqp_with_hotstart (double* ret,
                                        double* eval_weight_matrix, double* eval_coeff_vector,
                                        double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                                        int state_len, int inequality_len, int verbose, double* ret_status,
                                        int eval_weight_matrix_sparce_given, double* eval_weight_matrix_sparce,
                                        int inequality_matrix_sparce_given, double* inequality_matrix_sparce) {
    solve_osqp_hotstart(ret,
                      eval_weight_matrix, eval_coeff_vector,
                      inequality_matrix, inequality_min_vector, inequality_max_vector,
                        state_len, inequality_len, verbose, ret_status,
                        eval_weight_matrix_sparce_given, eval_weight_matrix_sparce,
                        inequality_matrix_sparce_given, inequality_matrix_sparce);
  };

}
