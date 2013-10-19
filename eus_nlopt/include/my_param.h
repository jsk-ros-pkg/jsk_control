namespace Optimization
{
	/*!
	 * @namespace Ipopt
	 * @brief Ipoptソルバオプションの列挙型の名前空間
	 */
	namespace Ipopt
	{
		/*!
		 * @enum InteriorPointMethodImplementation
		 * @brief 内点法の実装
		 */
		enum InteriorPointMethodImplementation
		{
			IPOPT=0, /* Ipoptの通常実装 */
			MPC, /* Mehrotra predictor-corrector method(S. Mehrotra, On the implementation of a primal-dual interior point method, SIAM J. Optimization 2(1992)575–601.)による実装．拡張目的関数の勾配方向の決定に一部の制約条件のみを考慮するため，LPや凸QPでは高速に解ける． */
			NUM_INTERIOR_POINT_METHOD_IMPLEMENTATIONS
		};
		/*!
		 * @enum BarrierParameterUpdateStrategy
		 * @brief 内点法でバリア関数の更新則
		 */
		enum BarrierParameterUpdateStrategy
		{
			SUMT=0, /* Sequential Unconstrained Minimization Technique(Fiacco, A.V. and G.P. McCormick, Nonlinear Programming: Sequential Unconstrained Minimization Techniques, John Wiley & Sons, New York, 1968)．徐々にバリア関数の係数を0にする．収束速度は遅いが非線形性の強い問題には適している． */
			LOQO, /* LOQO(Vanderbei, R. (1999), ‘LOQO: An interior point code for quadratic programming’,Optimization Methods and Software 12, 451–484. 411)の実装．次の評価点が主双対問題の許容解空間の中心パス上に乗るようにバリア関数の係数を計算する．収束速度が早く，特に線形な制約条件の問題に適している． */
			GSS, /* Golden section search(Kiefer, J. (1953), “Sequential minimax search for a maximum”, Proceedings of the American Mathematical Society 4 (3): 502–506)によって拡張目的関数が最小になるようなバリア関数の係数を計算する．収束速度が早く，拡張目的関数が凸な問題に適している． */
			NUM_BARRIER_PARAMETER_UPDATE_STRATEGIES
		};
	}
	/*!
	 * @namespace NLopt
	 * @brief NLoptソルバオプションの列挙型の名前空間
	 */
	namespace NLopt
	{
		/*!
		 * @enum Algorithm
		 * @brief NLoptに実装されているアルゴリズム
		 */
		enum Algorithm
		{
			/* 大域最適化問題のアルゴリズム */
			DIRECT=0, /* Didiving rectangle algorithm(D. R. Jones, C. D. Perttunen, and B. E. Stuckmann, "Lipschitzian optimization without the lipschitz constant," J. Optimization Theory and Applications, vol. 79, p. 157, 1993)．評価値が低い部分空間の分割を進めることで大域的最適解を探索する．非凸，多数の局所最適解を持つような問題を解くことができる． */
			G_DIRECT=1, /* DIRECTのGablonskyによる実装． */
			DIRECT_L=2, /* DIRECT locally biased(J. M. Gablonsky and C. T. Kelley, "A locally-biased form of the DIRECT algorithm," J. Global Optimization, vol. 21 (1), p. 27-37, 2001)．より局所的な分割を重視したDIRECT法．低次元で少数の局所最適解しか持たない問題に適する． */
			G_DIRECT_L=3, /* DIRECT-lのGablonskyによる実装． */
			CRS=4, /* Controlled random search with local mutation(P. Kaelo and M. M. Ali, "Some variants of the controlled random search algorithm for global optimization," J. Optim. Theory Appl. 130 (2), 253-264, 2006)．探索空間にランダムサンプリングして，シンプレックス法と線形近似を用いて探索する． */
			STOGO=5, /* StoGo(Madsen, K, Zertchaninov, S. and Zilinskas, A. Global Optimization using Branch-and-Bound, Submitted to the Journal of Global Optimization, 1998)．分枝限定法の内部で準ニュートンを用いて探索する． */
			ISRES=6, /* Improved Stochastic ranking evolution strategy(Thomas Philip Runarsson and Xin Yao, "Search biases in constrained evolutionary optimization," IEEE Trans. on Systems, Man, and Cybernetics Part C: Applications and Reviews, vol. 35 (no. 2), pp. 233-243, 2005)．遺伝的アルゴリズムに制約条件をペナルティ関数として追加する． */
			/* 勾配を用いた局所探索 */
			CCSA=7, /* Conservative convec separate approximation(Krister Svanberg, "A class of globally convergent optimization methods based on conservative convex separable approximations," SIAM J. Optim. 12 (2), p. 555-573, 2002)．評価関数と制約条件の勾配から保守側にローカルな二次関数近似をして探索する．規模が大きく(10^4~5)，密な問題でも解くことができる． */
			SLSQP=8, /* Sequential least squares quadratic programming(Dieter Kraft, "A software package for sequential quadratic programming", Technical Report DFVLR-FB 88-28, Institut für Dynamik der Flugsysteme, Oberpfaffenhofen, July 1988)．評価関数を二次近似し，制約条件を一次近似することで得られる二次計画問題を解いて探索方向を決定する．小中規模(10^2~3)問題には特に有効なアルゴリズム． */
			L_BFGS=9, /* Limited memory Broyden Fletcher Goldfarb Shanno(J. Nocedal, "Updating quasi-Newton matrices with limited storage," Math. Comput. 35, 773-782, 1980)．ヘッセ行列の近似に省メモリ版BFGS公式を用いた準ニュートン法．大規模(>10^3)の問題には適している．最適解近傍を初期値としない場合は大域収束性は補償されない． */
			TN=10, /* Truncated Newton(R. S. Dembo and T. Steihaug, "Truncated Newton algorithms for large-scale optimization," Math. Programming 26, p. 190-212, 1982)．勾配方向を共役勾配法で近似的に求めるニュートン法．収束は遅いが各ステップの計算は速い．保持するメモリが少なく大規模な問題でも解くことができる． */
			SL_VM=11, /* Shifted limited memory variable metric(J. Vlcek and L. Luksan, "Shifted limited-memory variable metric methods for large-scale unconstrained minimization," J. Computational Appl. Math. 186, p. 365-390, (2006)．ヘッセ行列の省メモリな近似を用いた修正ニュートン法．大域収束性を補償している．大規模な問題でも解くことができる． */
			// 勾配を使わないローカル探索 べーた
			COBYLA=12,
			BOBYQA=13,
			NEWUOA=14,
			PRAXIS=15,
			NelderMeadSimplex=16,
			Sbplx=17,
			NUM_ALGORITHMS=18
		};
	}
	/*!
	 * @enum Algorithm
	 * @brief 最適化計算のアルゴリズム(最初の3bitで使うソルバの設定)
	 */
	enum Algorithm
	{
		/* NLopt */
		/* 大域最適化問題のアルゴリズム */
		DIRECT		=0b0000000,
		G_DIRECT	=0b0001000,
		DIRECT_L	=0b0010000,
		G_DIRECT_L	=0b0011000,
		CRS		=0b0100000,
		STOGO		=0b0101000,
		ISRES		=0b0110000,
		/* 勾配を用いた局所探索 */
		CCSA		=0b0111000,
		SLSQP		=0b1000000,
		L_BFGS		=0b1001000,
		TN		=0b1010000,
		SL_VM		=0b1011000,
		/* Ipopt */
		IP_MPC		=0b1001001, /* InteriorPointMethodImplementation = MPC */
		IP_SUMT		=0b0000001, /* InteriorPointMethodImplementation = IPOPT, BarrierParameterUpdateStrategy = SUMT, hessian_approximation = false */
		IP_SUMT_BFGS	=0b1000001, /* InteriorPointMethodImplementation = IPOPT, BarrierParameterUpdateStrategy = SUMT, hessian_approximation = true (BFGS) */
		IP_LOQO		=0b0010001, /* InteriorPointMethodImplementation = IPOPT, BarrierParameterUpdateStrategy = LOQO, hessian_approximation = false */
		IP_LOQO_BFGS	=0b1010001, /* InteriorPointMethodImplementation = IPOPT, BarrierParameterUpdateStrategy = LOQO, hessian_approximation = true (BFGS) */
		IP_GSS		=0b0100001, /* InteriorPointMethodImplementation = IPOPT, BarrierParameterUpdateStrategy = GSS, hessian_approximation = false */
		IP_GSS_BFGS	=0b1100001 /* InteriorPointMethodImplementation = IPOPT, BarrierParameterUpdateStrategy = GSS, hessian_approximation = true (BFGS) */
	};
}
