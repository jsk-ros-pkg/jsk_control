#include <iostream>
#include <stdlib.h>
#include <nlopt.h>

#include "nlopt_solver.h"
//#include "eus_function.cpp"

NLoptSolver::NLoptSolver(double* x,
		const double* x_min,
		const double* x_max,
		int (*f)(double*,double*), int (*df)(double*,double*),
		int (*g)(double*,double*), int (*dg)(double*,double*),
		int (*h)(double*,double*), int (*dh)(double*,double*),
		int m_x, int m_g, int m_h,
		double ftol, double xtol, double eqthre, int max_eval, double max_time,
		Optimization::NLopt::Algorithm algorithm)
	:
		x(x), m_x(m_x), m_g(m_g), m_h(m_h), iteration(0), n_f(0), n_df(0), n_g(0), n_dg(0), n_h(0), n_dh(0) {
	this->f = f ;
	this->df = df;
	this->g = g ;
	this->dg = dg;
	this->h = h ;
	this->dh = dh;
	//
	this->fbuf = (double*)malloc(sizeof(double)*1) ;
	this->fbuf[0] = 0 ;
	this->dfbuf = (double*)malloc(sizeof(double)*1*m_x) ;
	for ( uint i=0 ; i<1*m_x ; i++ ) this->dfbuf[i] = 0 ;
	this->gbuf = (double*)malloc(sizeof(double)*m_g) ;
	for ( uint i=0 ; i<m_g ; i++ ) this->gbuf[i] = 0 ;
	this->dgbuf = (double*)malloc(sizeof(double)*m_g*m_x) ;
	for ( uint i=0 ; i<m_x*m_g ; i++ ) this->dgbuf[i] = 0 ;
	this->hbuf = (double*)malloc(sizeof(double)*m_h) ;
	for ( uint i=0 ; i<m_h ; i++ ) this->hbuf[i] = 0 ;
	this->dhbuf = (double*)malloc(sizeof(double)*m_h*m_x) ;
	for ( uint i=0 ; i<m_x*m_h ; i++ ) this->dhbuf[i] = 0 ;
	//
	// ラグランジュ法を用いて全てのアルゴリズムで制約条件を扱えるようにする
	if(algorithm == Optimization::NLopt::ISRES || algorithm == Optimization::NLopt::SLSQP)
	{
	  core_solver = nlopt_create(NLOPT_LD_CCSAQ, m_x); // dummy
		if(algorithm == Optimization::NLopt::ISRES)
		{
			solver = nlopt_create(NLOPT_GN_ISRES, m_x);
			nlopt_set_maxeval(solver, 1e6);
			nlopt_set_maxtime(solver, 24*60*60);
		}
		else
		{
			solver = nlopt_create(NLOPT_LD_SLSQP, m_x);
			nlopt_set_ftol_rel(solver, ftol);
			nlopt_set_xtol_rel(solver, xtol);
			//nlopt_set_maxeval(solver, 1e3);
		}
	}
	else if(algorithm == Optimization::NLopt::G_DIRECT || algorithm == Optimization::NLopt::G_DIRECT_L || algorithm == Optimization::NLopt::CCSA)
	{
	  // nlopt_opt core_solver;
		if(algorithm == Optimization::NLopt::CCSA)
		{
			core_solver = nlopt_create(NLOPT_LD_CCSAQ, m_x);
			nlopt_set_ftol_rel(core_solver, ftol);
			nlopt_set_xtol_rel(core_solver, xtol);
			nlopt_set_maxeval(core_solver, 1e6);
		}
		else
		{
			if(algorithm == Optimization::NLopt::G_DIRECT)
			{
				core_solver = nlopt_create(NLOPT_GN_ORIG_DIRECT, m_x);
			}
			else
			{
				core_solver = nlopt_create(NLOPT_GN_ORIG_DIRECT_L, m_x);
			}
			nlopt_set_maxeval(core_solver, 1e6);
			nlopt_set_maxtime(core_solver, 24*60*60);
			nlopt_set_ftol_rel(core_solver, ftol);
			nlopt_set_xtol_rel(core_solver, xtol);
		}
		solver = nlopt_create(NLOPT_AUGLAG_EQ, m_x);
		nlopt_set_local_optimizer(solver, core_solver);
		nlopt_set_ftol_rel(solver, ftol);
		nlopt_set_xtol_rel(solver, xtol);
		nlopt_set_maxeval(solver, 1e6);
		// nlopt_destroy(core_solver);
	}
	else if(algorithm == Optimization::NLopt::DIRECT || algorithm == Optimization::NLopt::DIRECT_L || algorithm == Optimization::NLopt::CRS || algorithm == Optimization::NLopt::STOGO || algorithm == Optimization::NLopt::L_BFGS || algorithm == Optimization::NLopt::TN || algorithm == Optimization::NLopt::SL_VM)
	{
	  // nlopt_opt core_solver;
		if(algorithm == Optimization::NLopt::DIRECT || algorithm == Optimization::NLopt::DIRECT_L || algorithm == Optimization::NLopt::CRS || algorithm == Optimization::NLopt::STOGO)
		{
			if(algorithm == Optimization::NLopt::DIRECT)
			{
				core_solver = nlopt_create(NLOPT_GN_DIRECT, m_x);
			}
			else if(algorithm == Optimization::NLopt::DIRECT_L)
			{
				core_solver = nlopt_create(NLOPT_GN_DIRECT_L, m_x);
			}
			else if(algorithm == Optimization::NLopt::CRS)
			{
				core_solver = nlopt_create(NLOPT_GN_CRS2_LM, m_x);
			}
			else
			{
				core_solver = nlopt_create(NLOPT_GD_STOGO, m_x);
			}
			nlopt_set_maxeval(core_solver, 1e6);
			nlopt_set_maxtime(core_solver, 24*60*60);
			nlopt_set_ftol_rel(core_solver, ftol);
			nlopt_set_xtol_rel(core_solver, xtol);
		}
		else
		{
			if(algorithm == Optimization::NLopt::L_BFGS)
			{
				core_solver = nlopt_create(NLOPT_LD_LBFGS, m_x);
			}
			else if(algorithm == Optimization::NLopt::TN)
			{
				core_solver = nlopt_create(NLOPT_LD_TNEWTON_PRECOND_RESTART, m_x);
			}
			else
			{
				core_solver = nlopt_create(NLOPT_LD_VAR2, m_x);
			}
			nlopt_set_ftol_rel(core_solver, ftol);
			nlopt_set_xtol_rel(core_solver, xtol);
			//nlopt_set_maxeval(core_solver, 1e3);
		}
		solver = nlopt_create(NLOPT_AUGLAG, m_x);
		nlopt_set_local_optimizer(solver, core_solver);
		nlopt_set_ftol_rel(solver, ftol);
		nlopt_set_xtol_rel(solver, xtol);
		//nlopt_set_maxeval(solver, 1e3);
		//		nlopt_destroy(core_solver);
	} else if ( algorithm == Optimization::NLopt::COBYLA ||
			algorithm == Optimization::NLopt::BOBYQA ||
			algorithm == Optimization::NLopt::NEWUOA ||
			algorithm == Optimization::NLopt::PRAXIS ||
			algorithm == Optimization::NLopt::NelderMeadSimplex ||
			algorithm == Optimization::NLopt::Sbplx ){
	  core_solver = nlopt_create(NLOPT_LD_CCSAQ, m_x); // dummy
		if(algorithm == Optimization::NLopt::COBYLA)
		{
			solver = nlopt_create(NLOPT_LN_COBYLA, m_x);
			nlopt_set_ftol_rel(solver, ftol);
			nlopt_set_xtol_rel(solver, xtol);
		}
		else if(algorithm == Optimization::NLopt::BOBYQA)
		{
			solver = nlopt_create(NLOPT_LN_BOBYQA, m_x);
			nlopt_set_ftol_rel(solver, ftol);
			nlopt_set_xtol_rel(solver, xtol);
		}
		else if(algorithm == Optimization::NLopt::NEWUOA)
		{
			solver = nlopt_create(NLOPT_LN_NEWUOA, m_x);
			nlopt_set_ftol_rel(solver, ftol);
			nlopt_set_xtol_rel(solver, xtol);
		}
		else if(algorithm == Optimization::NLopt::PRAXIS)
		{
			solver = nlopt_create(NLOPT_LN_PRAXIS, m_x);
			nlopt_set_ftol_rel(solver, ftol);
			nlopt_set_xtol_rel(solver, xtol);
		}
		else if(algorithm == Optimization::NLopt::NelderMeadSimplex)
		{
			solver = nlopt_create( NLOPT_LN_NELDERMEAD, m_x);
			nlopt_set_ftol_rel(solver, ftol);
			nlopt_set_xtol_rel(solver, xtol);
		}
		else if(algorithm == Optimization::NLopt::Sbplx)
		{
			solver = nlopt_create(NLOPT_LN_SBPLX, m_x);
			nlopt_set_ftol_rel(solver, ftol);
			nlopt_set_xtol_rel(solver, xtol);
		}
	}
	//
	if ( max_eval > 0 ) nlopt_set_maxeval(solver,max_eval) ;
	if ( max_time > 0 ) nlopt_set_maxtime(solver,max_time) ;
	// 定義域の設定
	nlopt_set_lower_bounds(solver, x_min);
	nlopt_set_upper_bounds(solver, x_max);
	// 評価関数の設定
	nlopt_set_min_objective(solver, NLoptSolver::ObjectiveFunctionWrapper, this);
	// 等式制約条件の設定
	// 数値的安定性のためにある程度の誤差を許容する
	double equality_constraint_tolerance[m_g];
	std::fill(equality_constraint_tolerance, equality_constraint_tolerance + m_g, eqthre);
	nlopt_add_equality_mconstraint(solver, m_g, NLoptSolver::EqualityConstraintWrapper, this, equality_constraint_tolerance);
	// 不等式制約条件の設定
	// 数値的安定性のためにある程度の誤差を許容する
	double inequality_constraint_tolerance[m_h];
	std::fill(inequality_constraint_tolerance, inequality_constraint_tolerance + m_h, eqthre);
	nlopt_add_inequality_mconstraint(solver, m_h, NLoptSolver::InequalityConstraintWrapper, this, inequality_constraint_tolerance);
	int major, minor, bugfix;
	nlopt_version(&major, &minor, &bugfix);
}

NLoptSolver::~NLoptSolver()
{
	nlopt_destroy(solver);
	if ( core_solver ) {
	  nlopt_destroy(core_solver);
	  core_solver = NULL;
	}
}

double NLoptSolver::ObjectiveFunctionWrapper(unsigned int n, const double* x, double* df, void* self)
{
	// 変数に代入する
	NLoptSolver* self_=reinterpret_cast<NLoptSolver*>(self);
	my_copy((double*)x,self_->x,self_->m_x) ; //self_->x = (double*)x ;
	// 評価関数の計算
	double f=self_->ObjectiveFunctionCost();
	// 各イテレーションの結果を出力する
//	OutputLog(self_->log_file, self_->frequency, std::setw(6) << std::right << self_->iteration << " " << std::scientific << std::setw(13) << f);
	// 勾配ベクトルの計算
	if(df != NULL)
	{
		my_copy(df,self_->dfbuf,1 * self_->m_x) ;
		self_->ObjectiveFunctionGradient(self_->dfbuf);
		my_copy(self_->dfbuf, df,1 * self_->m_x) ;
		// 勾配ベクトルのノルムを出力する
//		OutputIterationLog(self_->log_file, self_->frequency, std::setw(13) << df_.norm());
	}
	else
	{
		// 勾配ベクトルがない時は-を出力する
//		OutputIterationLog(self_->log_file, self_->frequency, "      -      ");
	}
	// イテレーション数のカウント
	self_->iteration++;
	return f;
}

// 評価関数
double NLoptSolver::ObjectiveFunctionCost()
{
	this->n_f++;
	// 評価関数の和
	if ( this->f ) (*(this->f))(this->x,this->fbuf) ;
	return this->fbuf[0];
}

// 評価関数の勾配
void NLoptSolver::ObjectiveFunctionGradient(double* dfbuf)
{
	this->n_df++;
	// 評価関数の和の勾配ベクトル

//	dfbuf[0] = 0 ;
	if ( this->df ) (*(this->df))(this->x,dfbuf) ;
}

void NLoptSolver::EqualityConstraintWrapper(unsigned int m, double* g, unsigned int n, const double* x, double* dg, void* self)
{
	// 変数に代入する
	NLoptSolver* self_=reinterpret_cast<NLoptSolver*>(self);

	my_copy((double*)x,self_->x,self_->m_x) ; //self_->x = (double*)x ;
	// 不等式制約条件
	my_copy(g,self_->gbuf,self_->m_g) ;
	self_->EqualityConstraintCost(self_->gbuf);
	my_copy(self_->gbuf,g,self_->m_g) ;
	// 勾配計算
	if(dg != NULL)
	{
		my_copy(dg,self_->dgbuf,self_->m_x * self_->m_g) ;
		self_->EqualityConstraintGradient(self_->dgbuf);
		my_copy(self_->dgbuf,dg,self_->m_x * self_->m_g) ;
	}
}

// 等式制約条件
void NLoptSolver::EqualityConstraintCost(double* gbuf)
{
	this->n_g++;
	if ( this->g && this->m_g>0 ) (*(this->g))(this->x,gbuf) ;
	//this->gbuf = gbuf ;
	// 等式制約条件を縦に並べる
}

// 等式制約条件の勾配
void NLoptSolver::EqualityConstraintGradient(double* dgbuf)
{
	this->n_dg++;
	if ( this->dg && this->m_g>0 ) (*(this->dg))(this->x,dgbuf) ;
}

void NLoptSolver::InequalityConstraintWrapper(unsigned int m, double* h, unsigned int n, const double* x, double* dh, void* self)
{
	// 変数に代入する
	NLoptSolver* self_=reinterpret_cast<NLoptSolver*>(self);
	my_copy((double*)x,self_->x,self_->m_x) ; //self_->x = (double*)x ;
	// 不等式制約条件
	my_copy(h,self_->hbuf,self_->m_h) ;
	self_->InequalityConstraintCost(self_->hbuf);
	my_copy(self_->hbuf,h,self_->m_h) ;
	// 勾配行列計算
	if(dh != NULL)
	{
		my_copy(dh,self_->dhbuf,self_->m_x * self_->m_h) ;
		self_->InequalityConstraintGradient(self_->dhbuf);
		my_copy(self_->dhbuf,dh,self_->m_x * self_->m_h) ;
	}
	// 各イテレーションの結果を出力する
	//OutputIterationLog(self_->log_file, self_->frequency, std::setw(14) << h_.maxCoeff() << std::fixed << std::endl);
}

// 不等式制約条件
void NLoptSolver::InequalityConstraintCost(double* hbuf)
{
	this->n_h++;
	if ( this->h && this->m_h>0 ) (*(this->h))(this->x,hbuf) ;
	//this->hbuf = hbuf ;
}

// 不等式制約条件の勾配
void NLoptSolver::InequalityConstraintGradient(double* dhbuf)
{
	this->n_dh++;
	if ( this->dh && this->m_h>0) (*(this->dh))(this->x,dhbuf) ;
}

// 最適化計算
int NLoptSolver::Optimize()
{
	// 最適化計算
	double cost;
	nlopt_result result=nlopt_optimize(this->solver, this->x, &cost);
	//this->output_result(result) ;
	this->fbuf[0] = cost;
	return result ;
}
