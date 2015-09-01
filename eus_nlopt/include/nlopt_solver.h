//#include "eus_function.cpp"
#include <sstream>
#include "my_param.h"

class NLoptSolver
{
public:
	NLoptSolver(double* x,
			const double* x_min,
			const double* x_max,
			int (*f)(double*,double*), int (*df)(double*,double*),
			int (*g)(double*,double*), int (*dg)(double*,double*),
			int (*h)(double*,double*), int (*dh)(double*,double*),
			int m_x, int m_g, int m_h,
			double ftol, double xtol, double eqthre, int max_eval, double max_time,
			Optimization::NLopt::Algorithm algorithm);
	~NLoptSolver();
	int Optimize();
	/*!
	 * @brief 評価関数の和を計算する
	 * @return 評価値の和
	 */
	double ObjectiveFunctionCost();
	//! 等式制約条件
	/*!
	 * @brief 等式制約条件を縦に並べたベクトルを計算する
	 * @param[out] g 等式制約条件を縦に並べたベクトル
	 */
	void EqualityConstraintCost(double* g);
	//! 不等式制約条件
	/*!
	 * @brief 不等式制約条件を縦に並べたベクトルを計算する
	 * @param[out] h 不等式制約条件を縦に並べたベクトル
	 */
	void InequalityConstraintCost(double* h);
	//
	double *fbuf, *gbuf, *hbuf, *dfbuf, *dgbuf, *dhbuf, *x ;
	//
	//
	// util functions vv
	static void my_log(std::string comment) {
		std::cout << comment << std::endl ;
	}

	static void my_log(std::string comment, double val){
		std::ostringstream os;
		os << comment ;
		os << " " << val;
		std::cout << os.str() << std::endl ;
	}

	static void my_log(std::string comment, double* val, uint cnt){
		std::ostringstream os;
		os << comment ;
		if ( val && cnt > 0 ){
			for ( uint i=0; i<cnt ; i++ ){
				os << " " << val[i] ;
			}
		}
		std::cout << os.str() << std::endl ;
	}

	static void my_copy(double* in, double* out, int size){
		for ( uint i=0 ; i<size ; i++ ){
			out[i] = in[i] ;
		}
	}

	void output_result(int result){
		if(result == NLOPT_SUCCESS)
		{
			my_log("Succeed.");
		}
		else if(result == NLOPT_FTOL_REACHED)
		{
			my_log("Succeed: Relative tolerance on function value was reached. ");
		}
		else if(result == NLOPT_XTOL_REACHED)
		{
			my_log("Succeed: Relative tolerance on optimization parameters was reached. ");
		}
		else if(result == NLOPT_MAXEVAL_REACHED)
		{
			my_log("Succeed: Maximum number of function evaluations was reached. ");
		}
		else if(result == NLOPT_MAXTIME_REACHED)
		{
			my_log("Succeed: Maximum optimization time was reached. ");
		}
		else if(result == NLOPT_FAILURE)
		{
			my_log("Fail.");
		}
		else if(result == NLOPT_INVALID_ARGS)
		{
			my_log("Fail: Invalid arguments.");
		}
		else if(result == NLOPT_OUT_OF_MEMORY)
		{
			my_log("Fail: Run out of memory.");
		}
		else if(result == NLOPT_ROUNDOFF_LIMITED)
		{
			my_log("Fail: Roundoff errors limited progress.");
		}

		//
		// if ( this->f ) (*(this->f))(this->x,fbuf) ;
		// if ( this->g ) (*(this->g))(this->x,gbuf) ;
		// if ( this->h ) (*(this->h))(this->x,hbuf) ;
		//
		my_log("Number of iteration.:	", this->iteration - 1);
		this->ObjectiveFunctionCost();
		my_log("object function: ", this->fbuf[0]) ;
		my_log("  | where     x: ", this->x, this->m_x) ;
		this->EqualityConstraintCost(this->gbuf) ;
		my_log("  |   eq constt: ", this->gbuf, this->m_g) ;
		this->InequalityConstraintCost(this->hbuf) ;
		my_log("  |  neq constt: ", this->hbuf, this->m_h) ;
	}

	void stop(){
		nlopt_force_stop(this->solver) ;
	}

private:
		nlopt_opt solver;
		nlopt_opt core_solver;
		// double* x;
		int (*f)(double*,double*), (*df)(double*,double*);
		//! 等式制約条件
		int (*g)(double*,double*), (*dg)(double*,double*);
		//! 不等式制約条件
		int (*h)(double*,double*), (*dh)(double*,double*);
		unsigned int m_x ;
		//! 等式制約条件の次元
		unsigned int m_g;
		//! 不等式制約条件の次元
		unsigned int m_h;
		//! 最適化計算の出力頻度
		unsigned int frequency;
		//! 最適化計算のイテレーション数
		unsigned int iteration;
		//! 評価関数の計算回数
		unsigned int n_f;
		//! 評価関数の勾配の計算回数
		unsigned int n_df;
		//! 等式制約条件の計算回数
		unsigned int n_g;
		//! 等式制約条件の勾配の計算回数
		unsigned int n_dg;
		//! 不等式制約条件の計算回数
		unsigned int n_h;
		//! 不等式制約条件の勾配の計算回数
		unsigned int n_dh;
		//! 評価関数のラッパー
		/*!
		 * @brief 評価関数のラッピング関数
		 * @param n 変数の次元
		 * @param x 変数
		 * @param df 勾配ベクトル
		 * @param self thisポインタ
		 * @return 評価値
		 */

		static double ObjectiveFunctionWrapper(unsigned int n, const double* x, double* df, void* self);
		//! 評価関数
		//! 評価関数の勾配
		/*!
		 * @brief 評価関数の和の勾配を計算する
		 * @param[out] df 評価関数の和の勾配ベクトル
		 */
		void ObjectiveFunctionGradient(double* df);
		//! 等式制約条件のラッパー
		/*!
		 * @brief 等式制約条件のラッピング関数
		 * @param m 等式制約条件の次元
		 * @param g 等式制約条件
		 * @param n 変数の次元
		 * @param x 変数
		 * @param dg 勾配ベクトル
		 * @param self thisポインタ
		 */
		static void EqualityConstraintWrapper(unsigned int m, double* g, unsigned int n, const double* x, double* dg, void* self);
		//! 等式制約条件の勾配
		/*!
		 * @brief 等式制約条件を縦に並べたベクトルの勾配を計算する
		 * @param[out] dg 制約条件を縦に並べたベクトルの勾配
		 */
		void EqualityConstraintGradient(double* dg);
		//! 不等式制約条件のラッパー
		/*!
		 * @brief 不等式制約条件のラッピング関数
		 * @param m 不等式制約条件の次元
		 * @param h 不等式制約条件
		 * @param n 変数の次元
		 * @param x 変数
		 * @param dh 勾配ベクトル
		 * @param self thisポインタ
		 */
		static void InequalityConstraintWrapper(unsigned int m, double* h, unsigned int n, const double* x, double* dh, void* self);
		//! 不等式制約条件の勾配
		/*!
		 * @brief 不等式制約条件を縦に並べたベクトルの勾配を計算する
		 * @param[out] dh 不等式制約条件を縦に並べたベクトルの勾配
		 */
		void InequalityConstraintGradient(double* dh);

};
