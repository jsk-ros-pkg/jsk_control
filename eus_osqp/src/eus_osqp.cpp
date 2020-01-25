#include <osqp.h>

// Solve QP
// ref. example of OSQP:
// https://github.com/oxfordcontrol/osqp/blob/master/examples/osqp_demo.c
// https://osqp.org/docs/examples/setup-and-solve.html
double* solve_osqp_common (double* ret,
                                 double* eval_weight_matrix, double* eval_coeff_vector,
                                 double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                                 int state_len, int inequality_len, int verbose, double* ret_status) {
  // Load problem data
  c_float *P_x = new c_float[state_len*state_len];
  c_int P_nnz = state_len*state_len;
  c_int *P_i = new c_int[state_len*state_len];
  c_int *P_p = new c_int[state_len+1];
  c_float *q = new c_float[state_len];
  c_float *A_x = new c_float[state_len*inequality_len];
  c_int A_nnz = state_len*inequality_len;
  c_int *A_i = new c_int[state_len*inequality_len];
  c_int *A_p = new c_int[state_len+1];
  c_float *l = new c_float[inequality_len];
  c_float *u = new c_float[inequality_len];

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
  for (c_int i = 0; i < state_len; i++) {
    q[i] = eval_coeff_vector[i];
  }
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
  settings->rho = 1e-6;
  settings->alpha = 0.1;
  settings->max_iter = 100000;
  settings->verbose = verbose;

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

extern "C" {
  double* solve_osqp_qp (double* ret,
                         double* eval_weight_matrix, double* eval_coeff_vector,
                         double* inequality_matrix, double* inequality_min_vector, double* inequality_max_vector,
                         int state_len, int inequality_len, int verbose, double* ret_status) {
    solve_osqp_common(ret,
                      eval_weight_matrix, eval_coeff_vector,
                      inequality_matrix, inequality_min_vector, inequality_max_vector,
                      state_len, inequality_len, verbose, ret_status);
  };
}
