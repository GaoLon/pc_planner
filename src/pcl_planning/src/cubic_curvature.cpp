#include "cubic_curvature.h"
// #include "lbfgs.hpp"

namespace cubic_curvature {
	// Simpson integrate
	template <typename F>
	static double Simpson(F const& f, double a, double b, int n)
	{
		if (n == 0)
		{
			return 0;
		}

		const double h = (b - a) / n;
		double s = f(a) + f(b);
		for (int i = 1; i < n; i += 2)s += 4 * f(a + i * h);
		for (int i = 2; i < n - 1; i += 2)s += 2 * f(a + i * h);
		return s * h / 3.0;
	}

	// integrate function for gsl
	static double fx(double x, void * params) {
		double* alpha = (double *)params;
		double a = alpha[0];
		double b = alpha[1];
		double c = alpha[2];
		double kappa = alpha[3];
		double theta = alpha[4];
		double f = theta + kappa * x + a * x*x / 2 + b * pow(x, 3) / 3 + c * pow(x, 4) / 4;

		return cos(f);
	}

	// integrate function for gsl
	static double fy(double x, void * params) {
		double* alpha = (double *)params;
		double a = alpha[0];
		double b = alpha[1];
		double c = alpha[2];
		double kappa = alpha[3];
		double theta = alpha[4];
		double f = theta + kappa * x + a * x*x / 2 + b * pow(x, 3) / 3 + c * pow(x, 4) / 4;

		return sin(f);
	}

	// forward Integrate
	Vector4d forward(Vector4d init, Vector4d x)
	{
		//cout << "forward brgin" << endl;
		Vector4d y = Vector4d::Zero();
		double result, error;
		double alpha[5];
		alpha[0] = x(0); alpha[1] = x(1); alpha[2] = x(2); alpha[3] = init(3); alpha[4] = init(2);
		gsl_function F;
		auto fun = [init, x](const double& s) {return cos(init(2) + init(3)*s + x(0)*s*s / 2 + x(1)*pow(s, 3) / 3 + x(2)*pow(s, 4) / 4); };
		auto fun2 = [init, x](const double& s) {return sin(init(2) + init(3)*s + x(0)*s*s / 2 + x(1)*pow(s, 3) / 3 + x(2)*pow(s, 4) / 4); };
		y(2) = init(2) + init(3)*x(3) + x(0)*x(3)*x(3) / 2 + x(1)*pow(x(3), 3) / 3 + x(2)*pow(x(3), 4) / 4;
		y(3) = init(3) + x(0)*x(3) + x(1)*x(3)*x(3) + x(2)*pow(x(3), 3);
		//y(0) = Simpson(fun, 0, x(3), (int)(x(3) / 0.001)) + init(0);
		//y(1) = Simpson(fun2, 0, x(3), (int)(x(3) / 0.001)) + init(1);
		gsl_integration_workspace * wx
			= gsl_integration_workspace_alloc(1000);
		gsl_integration_workspace * wy
			= gsl_integration_workspace_alloc(1000);
		
		// integrate x
		F.function = &fx;
		F.params = (void*)alpha;
		int state = gsl_integration_qags(&F, 0, x(3), 0, 1e-7, 1e-7,  \
			wx, &result, &error);
		if (state==GSL_ESING)
		{
			y(0) = std::numeric_limits<double>::infinity();
			y(1) = 0;
			gsl_integration_workspace_free(wx);
			gsl_integration_workspace_free(wy);
			return y;
		}
		y(0) = result + init(0);
		
		// integrate y
		F.function = &fy;
		state = gsl_integration_qags(&F, 0, x(3), 0, 1e-7, 1e-7,  \
			wx, &result, &error);
		if (state==GSL_ESING)
		{
			y(0) = std::numeric_limits<double>::infinity();
			y(1) = 0;
			gsl_integration_workspace_free(wy);
			gsl_integration_workspace_free(wy);
			return y;
		}
		y(1) = result + init(1);
		gsl_integration_workspace_free(wx);
		gsl_integration_workspace_free(wy);

		return y;
	}

	// get cubic curvature
	Vector4d get_cubic_curvature(Vector4d init, Vector4d ends)
	{
		// cout << "ends=" << ends << endl;
		// initiation
		double kappa0 = init[3];
		double xf = ends[0];
		double yf = ends[1];
		double thetaf = ends[2];
		double kappaf = ends[3];
		double d = hypot(xf, yf);
		double delta_th = abs(thetaf);
		double s0 = d * (delta_th * delta_th / 5 + 1);
		//double a0 = 6 * thetaf / s0 ^ 2 - 2 * kappa0 / s0 + 4 * kappaf / s0;
		double a0 = 6.0 * thetaf / (s0 * s0) - 2.0 * kappaf / s0 - 4 * kappa0 / s0;   //anothor start point
		double b0 = 3.0 / (s0 * s0) * (kappa0 + kappaf) - 6 * thetaf / (s0*s0*s0);
		double c0 = 0.0;
		Vector4d x(a0, b0, c0, s0);
		double h = 1e-4;

		// begin iteration
		for (int i = 0; i < 20; i++)
		{
			// judge convergence
			Vector4d fpk = forward(init, x);
			if (fpk[0]==std::numeric_limits<double>::infinity())
			{
				cout<<"GSL ERROR!!!"<<endl;
				return Vector4d::Zero();
			}
			//cout << "x" << x << endl;
			//cout << "fpk" << fpk << endl;
			double theta_error = abs(fpk(2) - thetaf);
			double xy_error = hypot(fpk(0) - xf, fpk(1) - yf);
			double kappa_error = abs(fpk(3) - kappaf);
			if (theta_error < 0.1&& xy_error < 0.001 && kappa_error < 0.005)
			{
				//cout << "get solution in iter=" << i - 1 << endl;
				return x;
			}

			// Newton step
			/// get hessian matrix
			Matrix4d hessian = Matrix4d::Zero();
			Matrix4d H = Matrix4d::Identity()*h;
			for (int i = 0; i < 4; i++)
			{
				hessian.col(i) = Matrix<double, 4, 1>(forward(init, x + Vector4d(H.col(i))) - fpk) / h;
			}
			// check hessian
			FullPivLU<Matrix4d> lu(hessian);
			if (lu.isInvertible())
			{
				x = Vector4d(lu.solve(Matrix<double, 4, 1>(ends - fpk))) + x;
			}
			else 
			{
				//cout << "error! hessian matrix isn't invertibal!" << endl;
				return Vector4d::Zero();
			}
			//x = Vector4d(hessian.lu().solve(Matrix<double, 4, 1>(ends - fpk))) + x;
			//x = Vector4d(hessian.inverse()*Matrix<double, 4, 1>(ends - fpk)) + x;
			for (auto j = 0; j < 4; j++)
			{
				if (abs(x[j]) > 1e7)
				{
					//cout << "error! cubic curvature failed!" << endl;
					return Vector4d::Zero();
				}
			}
		}
		
		return Vector4d::Zero();
	}

	//class objective_function
	//{
	//private:
	//	double *m_x;
	//	Vector4d init;
	//	Vector4d ends;

	//public:
	//	objective_function(Vector4d _init,Vector4d _ends) : m_x(NULL),init(_init),ends(_ends)
	//	{
	//	}

	//	~objective_function()
	//	{
	//		if (m_x != NULL)
	//		{
	//			free(m_x);
	//			m_x = NULL;
	//		}
	//	}

	//	int run(int N)
	//	{
	//		double fx;
	//		double *m_x = (double *)malloc(sizeof(double) * N);

	//		/* Initialize the variables. */
	//		double kappa0 = init[3];
	//		double xf = ends[0];
	//		double yf = ends[1];
	//		double thetaf = ends[2];
	//		double kappaf = ends[3];
	//		double d = hypot(xf, yf);
	//		double delta_th = abs(thetaf);
	//		double s0 = d * (delta_th * delta_th / 5 + 1) + 0.4*delta_th;
	//		//double a0 = 6 * thetaf / s0 ^ 2 - 2 * kappa0 / s0 + 4 * kappaf / s0;
	//		double a0 = 6.0 * thetaf / (s0 * s0) - 2.0 * kappaf / s0 - 4 * kappa0 / s0;   //anothor start point
	//		double b0 = 3.0 / (s0 * s0) * (kappa0 + kappaf) - 6 * thetaf / (s0*s0*s0);
	//		double c0 = 0.0;
	//		m_x[0] = a0; m_x[1] = b0; m_x[2] = c0; m_x[3] = s0;

	//		/*
	//			Start the L-BFGS optimization; this will invoke the callback functions
	//			evaluate() and progress() when necessary.
	//		 */
	//		lbfgs::lbfgs_parameter_t lbfgs_params;
	//		lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
	//		lbfgs_params.mem_size = 10;
	//		int ret = lbfgs::lbfgs_optimize(N, m_x, &fx, _evaluate, NULL, _progress, this, &lbfgs_params);

	//		/* Report the result. */
	//		printf("L-BFGS optimization terminated with status code = %d\n", ret);
	//		printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, m_x[0], m_x[1]);

	//		return ret;
	//	}

	//private:
	//	static double _evaluate(void *instance,
	//		const double *x,
	//		double *g,
	//		const int n)
	//	{
	//		return reinterpret_cast<objective_function *>(instance)->evaluate(x, g, n);
	//	}

	//	double evaluate(const double *x,
	//		double *g,
	//		const int n)
	//	{
	//		double fx = 0.0;
	//		Vector4d c(x[0], x[1], x[2], x[3]);
	//		Vector4d fpk = forward(init, c);
	//		Vector4d err = fpk - ends;
	//		fx = 1e4*(err(0)*err(0) + err(1)*err(1)) + err(2)*err(2) + 400 * err(3)*err(3);
	//		g[0] = 

	//		return fx;
	//	}

	//	static int _progress(void *instance,
	//		const double *x,
	//		const double *g,
	//		const double fx,
	//		const double xnorm,
	//		const double gnorm,
	//		const double step,
	//		int n,
	//		int k,
	//		int ls)
	//	{
	//		return reinterpret_cast<objective_function *>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
	//	}

	//	int progress(const double *x,
	//		const double *g,
	//		const double fx,
	//		const double xnorm,
	//		const double gnorm,
	//		const double step,
	//		int n,
	//		int k,
	//		int ls)
	//	{
	//		printf("Iteration %d:\n", k);
	//		printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
	//		printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
	//		printf("\n");
	//		return 0;
	//	}
	//};
}

