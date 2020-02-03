#include <cdd/setoper.h>
#include <cdd/cdd.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define digits1 5
#define digits2 3

double* A_eq_cash = NULL;
double* b_eq_cash = NULL;
double* A_ineq_cash = NULL;
double* b_ineq_cash = NULL;
double* V_cash = NULL;
double* R_nonneg_cash = NULL;
double* R_free_cash = NULL;

int cddlib_initialize (void){
  dd_set_global_constants();  /* First, this must be called to use cddlib. */
  return 0;
}

int cddlib_finalize (void){
  dd_free_global_constants();  /* At the end, this must be called. */
  return 0;
}

int max(int a, int b) {
  if (a > b)
    return a;
  else
    return b;
}

int min(int a, int b) {
  if (a < b)
    return a;
  else
    return b;
}

void set_value (mytype x, double value, int num, int m){
  if (value != 0){
    unsigned long int denom = pow(10, m);
    int numer = value * denom;

    int over = min(m,max(0,((int)(log10(fabs(numer))) - num)));
    denom /= pow(10,over);
    numer /= pow(10,over);

    while(denom>1 && numer!=0){
      if((numer / 10)*10 != numer) break;
      else{
        denom /= 10;
        numer /= 10;
      }
    }
    dd_set_si2(x, numer, denom);
  }else{
    dd_set_si2(x, 0, 1);
  }
  return;
}

/*
  A_eq   x + b_eq    = 0
  A_ineq x + b_ineq >= 0
 */
int cddlib_H_to_V (double* A_eq, double* b_eq, double* A_ineq, double* b_ineq, int d, int m_eq, int m_ineq, int* out_n, int* out_s_nonneq, int* out_s_free, int verbose)
{
  free(V_cash);
  V_cash = NULL;
  free(R_nonneg_cash);
  R_nonneg_cash = NULL;
  free(R_free_cash);
  R_free_cash = NULL;

  dd_PolyhedraPtr poly;
  dd_MatrixPtr A, G;
  dd_ErrorType err;


  double maxabs = 0;
  for (size_t i = 0; i < m_eq; i++){
    if (fabs(b_eq[i]) > maxabs) maxabs = fabs(b_eq[i]);
    for (size_t j = 0; j < d; j++){
      if (fabs(A_eq[i*d+j]) > maxabs) maxabs = fabs(A_eq[i*d+j]);
    }
  }
  for (size_t i = 0; i < m_ineq; i++){
    if (fabs(b_ineq[i]) > maxabs) maxabs = fabs(b_ineq[i]);
    for (size_t j = 0; j < d; j++){
      if (fabs(A_ineq[i*d+j]) > maxabs) maxabs = fabs(A_ineq[i*d+j]);
    }
  }
  int denom = (maxabs!=0)? max(- ((int)(log10(fabs(maxabs))) - digits1), 0) : 0;


  ddf_MatrixPtr Af;
  Af=ddf_CreateMatrix(m_eq + m_ineq,d+1);

  for (size_t i = 0; i < m_eq; i++){
    set_addelem(Af->linset,i+1);

    ddf_set_d(Af->matrix[i][0], b_eq[i]);
    for (size_t j = 0; j < d; j++){
      ddf_set_d(Af->matrix[i][j+1], A_eq[i*d+j]);
    }
  }
  for (size_t i = 0; i < m_ineq; i++){
    ddf_set_d(Af->matrix[m_eq+i][0], b_ineq[i]);
    for (size_t j = 0; j < d; j++){
      ddf_set_d(Af->matrix[m_eq+i][j+1], A_ineq[i*d+j]);
    }
  }

  Af->representation=dd_Inequality;

  ddf_rowindex newpos;
  ddf_rowset impl_linset,redset;
  ddf_ErrorType errf;
  if (verbose){
    printf("\nbefore canonicalize:\n");
    ddf_WriteMatrix(stdout,Af);  printf("\n");
  }

  ddf_MatrixCanonicalize(&Af, &impl_linset, &redset, &newpos, &errf);
  if (verbose){
    printf("\nafter canonicalize:\n");
    ddf_WriteMatrix(stdout,Af);  printf("\n");
  }

  A=dd_CreateMatrix(Af->rowsize,Af->colsize);
  for (size_t i = 0; i < Af->rowsize; i++){
    if(set_member(i+1,Af->linset)) set_addelem(A->linset,i+1);

    for (size_t j = 0; j < Af->colsize; j++){
      set_value(A->matrix[i][j], ddf_get_d(Af->matrix[i][j]), digits2,denom);
    }
  }
  A->representation=dd_Inequality;

  poly=dd_DDMatrix2Poly(A, &err);  /* compute the second (generator) representation */

  if (err==dd_NoError){
    G=dd_CopyGenerators(poly);
    if (verbose){
      printf("\nInput is H-representation:\n");
      dd_WriteMatrix(stdout,A);  printf("\n");
      dd_WriteMatrix(stdout,G);
    }
    int n = 0;
    int s_nonneg = 0;
    int s_free = 0;
    for (size_t i = 0; i < G->rowsize; i++){
      if (dd_get_d(G->matrix[i][0]) ==0){
        if (set_member(i+1,G->linset)) s_free++;
        else s_nonneg++;
      }else n++;
    }
    *out_n = n;
    *out_s_nonneq = s_nonneg;
    *out_s_free = s_free;
    int _n = 0;
    int _s_nonneg = 0;
    int _s_free = 0;
    V_cash = (double*)malloc(sizeof(double)*n*d);
    R_nonneg_cash = (double*)malloc(sizeof(double)*s_nonneg*d);
    R_free_cash = (double*)malloc(sizeof(double)*s_free*d);
    for (size_t i = 0; i < G->rowsize; i++){
      if (dd_get_d(G->matrix[i][0])==0){
        if (set_member(i+1,G->linset)){
          for (size_t j = 0; j < d; j++){
            R_free_cash[s_free*j+_s_free] = dd_get_d(G->matrix[i][1+j]);
          }
          _s_free++;
        }else{
          for (size_t j = 0; j < d; j++){
            R_nonneg_cash[s_nonneg*j+_s_nonneg] = dd_get_d(G->matrix[i][1+j]);
          }
          _s_nonneg++;
        }
      }else{
        for (size_t j = 0; j < d; j++){
          V_cash[n*j+_n] = dd_get_d(G->matrix[i][1+j]);
        }
        _n++;
      }
    }
    dd_FreeMatrix(A);
    dd_FreeMatrix(G);
    dd_FreePolyhedra(poly);
    return 0;
  }else{
    if (verbose){
      printf("\nInput is H-representation:\n");
      dd_WriteMatrix(stdout,A);
    }
    dd_WriteErrorMessages(stdout,err);
    dd_FreeMatrix(A);
    dd_FreePolyhedra(poly);
  }
  return -1;
}

int cddlib_get_V (double* V, double* R_nonneg, double* R_free, int d, int n, int s_nonneg, int s_free)
{
  for(size_t i=0; i < d*n; i++){
    V[i] = V_cash[i];
  }
  for(size_t i=0; i < d*s_nonneg; i++){
    R_nonneg[i] = R_nonneg_cash[i];
  }
  for(size_t i=0; i < d*s_free; i++){
    R_free[i] = R_free_cash[i];
  }
  return 0;
}

int cddlib_V_to_H (double* V, double* R_nonneg, double* R_free, int d, int n, int s_nonneg, int s_free, int* out_m_eq, int* out_m_ineq, int verbose)
{
  free(A_eq_cash);
  A_eq_cash = NULL;
  free(b_eq_cash);
  b_eq_cash = NULL;
  free(A_ineq_cash);
  A_ineq_cash = NULL;
  free(b_ineq_cash);
  b_ineq_cash = NULL;

  dd_PolyhedraPtr poly;
  dd_MatrixPtr A, G;
  dd_ErrorType err;

  double maxabs = 0;
  for (size_t i = 0; i < n; i++){
    for (size_t j = 0; j < d; j++){
      if (fabs(V[j*n+i]) > maxabs) maxabs = fabs(V[j*n+i]);
    }
  }
  for (size_t i = 0; i < s_nonneg; i++){
    for (size_t j = 0; j < d; j++){
      if (fabs(R_nonneg[j*s_nonneg+i]) > maxabs) maxabs = fabs(R_nonneg[j*s_nonneg+i]);
    }
  }
  for (size_t i = 0; i < s_free; i++){
    for (size_t j = 0; j < d; j++){
      if (fabs(R_free[j*s_free+i]) > maxabs) maxabs = fabs(R_free[j*s_free+i]);
    }
  }
  int denom = (maxabs!=0)? max(- ((int)(log10(fabs(maxabs))) - digits1), 0) : 0;

  ddf_MatrixPtr Gf;
  Gf=ddf_CreateMatrix(n+s_nonneg+s_free,d+1);

  for (size_t i = 0; i < n; i++){
    ddf_set_si(Gf->matrix[i][0],1);
    for (size_t j = 0; j < d; j++){
      ddf_set_d(Gf->matrix[i][j+1],V[j*n+i]);
    }
  }
  for (size_t i = 0; i < s_nonneg; i++){
    ddf_set_si(Gf->matrix[n+i][0],0);
    for (size_t j = 0; j < d; j++){
      ddf_set_d(Gf->matrix[n+i][j+1],R_nonneg[j*s_nonneg+i]);
    }
  }
  for (size_t i = 0; i < s_free; i++){
    set_addelem(Gf->linset,n+s_nonneg+i+1);
    ddf_set_si(Gf->matrix[n+s_nonneg+i][0],0);
    for (size_t j = 0; j < d; j++){
      ddf_set_d(Gf->matrix[n+s_nonneg+i][j+1],R_free[j*s_free+i]);
    }
  }

  Gf->representation=dd_Generator;

  ddf_rowindex newpos;
  ddf_rowset impl_linset,redset;
  ddf_ErrorType errf;
  if (verbose){
    printf("\nbefore canonicalize:\n");
    ddf_WriteMatrix(stdout,Gf);  printf("\n");
  }

  ddf_MatrixCanonicalize(&Gf, &impl_linset, &redset, &newpos, &errf);
  if (verbose){
    printf("\nafter canonicalize:\n");
    ddf_WriteMatrix(stdout,Gf);  printf("\n");
  }

  G=dd_CreateMatrix(Gf->rowsize,Gf->colsize);
  for (size_t i = 0; i < Gf->rowsize; i++){
    if(set_member(i+1,Gf->linset)) set_addelem(G->linset,i+1);

    dd_set_si(G->matrix[i][0], ddf_get_d(Gf->matrix[i][0]));
    for (size_t j = 0; j < Gf->colsize-1; j++){
      double value = ddf_get_d(Gf->matrix[i][j+1]);
      set_value(G->matrix[i][j+1], ddf_get_d(Gf->matrix[i][j+1]), digits2, denom);
    }
  }

  G->representation=dd_Generator;


  poly=dd_DDMatrix2Poly(G, &err);  /* compute the second (generator) representation */

  if (err==dd_NoError){
    A=dd_CopyInequalities(poly);
    if (verbose){
      printf("\nInput is V-representation:\n");
      dd_WriteMatrix(stdout,G);  printf("\n");
      dd_WriteMatrix(stdout,A);
    }
    int m_eq = 0;
    int m_ineq = 0;
    for (size_t i = 0; i < A->rowsize; i++){
      if (set_member(i+1,A->linset)) m_eq++;
      else m_ineq++;
    }
    *out_m_eq = m_eq;
    *out_m_ineq = m_ineq;
    int _m_eq = 0;
    int _m_ineq = 0;
    A_eq_cash = (double*)malloc(sizeof(double)*m_eq*d);
    b_eq_cash = (double*)malloc(sizeof(double)*m_eq);
    A_ineq_cash = (double*)malloc(sizeof(double)*m_ineq*d);
    b_ineq_cash = (double*)malloc(sizeof(double)*m_ineq);
    for (size_t i = 0; i < A->rowsize; i++){
      if (set_member(i+1,A->linset)){
        for (size_t j = 0; j < d; j++){
          A_eq_cash[d*_m_eq+j] = dd_get_d(A->matrix[i][1+j]);
        }
        b_eq_cash[_m_eq] = dd_get_d(A->matrix[i][0]);
        _m_eq++;
      }else{
        for (size_t j = 0; j < d; j++){
          A_ineq_cash[d*_m_ineq+j] = dd_get_d(A->matrix[i][1+j]);
        }
        b_ineq_cash[_m_ineq] = dd_get_d(A->matrix[i][0]);
        _m_ineq++;
      }
    }

    dd_FreeMatrix(A);
    dd_FreeMatrix(G);
    dd_FreePolyhedra(poly);
    return 0;
  }else{
    if (verbose){
      printf("\nInput is V-representation:\n");
      dd_WriteMatrix(stdout,G);
    }
    dd_WriteErrorMessages(stdout,err);
    dd_FreeMatrix(G);
    dd_FreePolyhedra(poly);
  }
  return -1;
}

int cddlib_get_H (double* A_eq, double* b_eq, double* A_ineq, double* b_ineq, int d , int m_eq, int m_ineq)
{
  for(size_t i=0; i < d*m_eq; i++){
    A_eq[i] = A_eq_cash[i];
  }
  for(size_t i=0; i < m_eq; i++){
    b_eq[i] = b_eq_cash[i];
  }
  for(size_t i=0; i < d*m_ineq; i++){
    A_ineq[i] = A_ineq_cash[i];
  }
  for(size_t i=0; i < m_ineq; i++){
    b_ineq[i] = b_ineq_cash[i];
  }
  return 0;
}
