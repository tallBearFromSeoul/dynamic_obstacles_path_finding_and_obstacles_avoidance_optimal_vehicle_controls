#include <cppad/ipopt/solve.hpp>

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Matrix<float, 4, 2> Mat42f;

class Vehicle;

inline float rad_to_deg(float __rad) {
	return 180.f*__rad/3.145927;
}

inline double deg_to_rad(double _deg) {
	return 3.145927*_deg/180;
}

inline float steer_to_beta(float __rad) {
	return atan(tan(__rad)*0.5f);
}

namespace {
	using CppAD::AD;
	int N;
	double P = 100;
	double Q = 1.0;
	double R = 0.5;
	double S = 200.0;
	// assuming 60 Hz
	double dt = 0.03333333;
	//double dt = 0.01666666;//0.025;
	double lr = 1.738;
	//double x0U = 20.;
	double x2U = 12.;
	float PI = 3.145927;
	double x3U = deg_to_rad(125);//PI*0.6f;
	double u0U = 0.7;
	double u1U = 1.5;
	Vec4f state_init;
	std::vector<NodePtr> zref;
	std::vector<ObsPtr> obstacles;
	int n_Sobs, n_Dobs;

	class FG_eval {
		private:
		public:
			
			FG_eval(int _time_horizon, float _dt, float _lr, const Vec4f &_state_init, std::vector<NodePtr> *_zref, std::vector<ObsPtr> &_obstacles) {
				N=_time_horizon;
				dt = _dt;
				lr = _lr;
				state_init = _state_init;
				zref = *_zref;
				
				n_Sobs = 0;
				n_Dobs = 0;
				obstacles = _obstacles;
				for (ObsPtr &__obs : _obstacles) {
				switch(__obs->type()) {
					case(Objet::Type::STATIC):
						n_Sobs++;
						break;
					case(Objet::Type::DYNAMIC):
						n_Dobs++;
						break;
					}
				}
			};
			// derived class part of constructor
			typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
			// Evaluation of the objective f(x) and constraindt g(x)
			void operator()(ADvector &fg, const ADvector &x) {
				fg[0]	= 0;
				fg[0] += CppAD::pow(zref[0]->val(0)-x[0],2)*P + CppAD::pow(zref[0]->val(1)-x[1], 2)*P + CppAD::pow(x[0], 2) + CppAD::pow(x[1], 2);
				for (int i=1; i<N; i++) {
					int px = i*6;
					int pu = i*4;
					int pz = i < zref.size() ? i : zref.size()-1;
					fg[0] += CppAD::pow(zref[pz]->val(0)-x[px],2)*P + CppAD::pow(zref[pz]->val(1)-x[px+1], 2)*P + CppAD::pow(x[pu], 2)*Q + CppAD::pow(x[pu+1], 2)*R;
				}
				fg[0] += CppAD::pow(zref.back()->val(0)-x[6*N],2)*S + CppAD::pow(zref.back()->val(1)-x[6*N+1],2)*S;

				// x 0 1 2 3 6 7 8 9 12 13 14 15
				// u 4 5 10 11 16 17
				for (int i=1; i<N+1; i++) {
					int pf = (i-1)*4+1;
					int px = i*6;
					int ppx = (i-1)*6;
					int pu = px-2;
					
					AD<double> alpha_1 = x[ppx+3] + x[pu];
					AD<double> k1_0 = x[ppx+2]*CppAD::cos(alpha_1);
					AD<double> k1_1 = x[ppx+2]*CppAD::sin(alpha_1);
					AD<double> k1_2 = x[pu+1];
					AD<double> k1_3 = x[ppx+2]*CppAD::sin(x[pu])/lr;
					
					AD<double> alpha_2 = x[ppx+3] + x[pu] + k1_3*dt;
					AD<double> k2_0 = (x[ppx+2]+k1_2*dt)*CppAD::cos(alpha_2);
					AD<double> k2_1 = (x[ppx+2]+k1_2*dt)*CppAD::sin(alpha_2);
					AD<double> k2_2 = x[pu+1];
					AD<double> k2_3 = (x[ppx+2]+k1_2*dt)*CppAD::sin(x[pu])/lr;
					
					AD<double> alpha_3 = x[ppx+3] + x[pu] + k2_3*dt;
					AD<double> k3_0 = (x[ppx+2]+k2_2*dt)*CppAD::cos(alpha_3);
					AD<double> k3_1 = (x[ppx+2]+k2_2*dt)*CppAD::sin(alpha_3);
					AD<double> k3_2 = x[pu+1];
					AD<double> k3_3 = (x[ppx+2]+k2_2*dt)*CppAD::sin(x[pu])/lr;

					AD<double> alpha_4 = x[ppx+3] + x[pu] + k3_3*dt;
					AD<double> k4_0 = (x[ppx+2]+k3_2*dt)*CppAD::cos(alpha_4);
					AD<double> k4_1 = (x[ppx+2]+k3_2*dt)*CppAD::sin(alpha_4);
					AD<double> k4_2 = x[pu+1];
					AD<double> k4_3 = (x[ppx+2]+k3_2*dt)*CppAD::sin(x[pu])/lr;

					AD<double> K_0 = k1_0/6 + k2_0/3 + k3_0/3 + k4_0/6;
					AD<double> K_1 = k1_1/6 + k2_1/3 + k3_1/3 + k4_1/6;
					AD<double> K_2 = k1_2/6 + k2_2/3 + k3_2/3 + k4_2/6;
					AD<double> K_3 = k1_3/6 + k2_3/3 + k3_3/3 + k4_3/6;
					fg[pf] = x[px] - x[ppx] - K_0*dt;
					fg[pf+1] = x[px+1] - x[ppx+1] - K_1*dt;
					fg[pf+2] = x[px+2] - x[ppx+2] - K_2*dt;
					fg[pf+3] = x[px+3] - x[ppx+3] - K_3*dt;

					// Euler Discretization (not accurate)
					/*
					AD<double> alpha = x[ppx+3] + x[pu];
					fg[pf] = x[px] - x[ppx] - x[ppx+2]*CppAD::cos(alpha)*dt;
					fg[pf+1] = x[px+1] - x[ppx+1] - x[ppx+2]*CppAD::sin(alpha)*dt;
					fg[pf+2] = x[px+2] - x[ppx+2] - x[pu+1]*dt;
					fg[pf+3] = x[px+3] - (x[ppx+2]/lr)*CppAD::sin(x[pu])*dt;
					*/
				}
				/*
				for (int j=0; j<n_Sobs+n_Dobs; j++) {
					for (int i=0; i<N+1; i++) {
						int px = i*6;
						AD<double> dx = x[px]-obstacles[j]->pos(0);
						AD<double> dy = x[px+1]+obstacles[j]->pos(1);
						AD<double> val = CppAD::abs(CppAD::abs(dx)-obstacles[j]->rad()) + CppAD::abs(CppAD::abs(dy)-obstacles[j]->rad());
						//fg[0] += 1.0/((double(i)+1.0)*val/double(N));
					}
				}
				*/
				for (int j=0; j<n_Sobs+n_Dobs; j++) {
					for (int i=0; i<N+1; i++) {
						int px = i*6;
						// this is correct
						int pf = 4*N+1 + j*(N+1) + i;
						AD<double> obs_x = obstacles[j]->pos(0);
						AD<double> obs_y = obstacles[j]->pos(1);
						AD<double> dx = x[px]-obs_x;
						AD<double> dy = x[px+1]-obs_y;
						//fg[pf] = CppAD::abs(dx)-obstacles[j]->rad() + CppAD::abs(dy)-obstacles[j]->rad();
						fg[pf] = CppAD::pow(dx, 2) + CppAD::pow(dy, 2);
					}
				}
		}
	};
}

typedef Eigen::Matrix2f Mat2f;
typedef Eigen::MatrixXf MatXf;

class Optimizer {
	private:
		int _N;
		float _ts;
		float _lr;
		MatXf _pred_states;
		MatXf _inputs;

	public:
		Optimizer(int _N, float _ts, float _lr) : _N(_N), _ts(_ts), _lr(_lr) {};
		MatXf pred_states() {return _pred_states;};
		MatXf inputs() {return _inputs;};
		Vec2f input_opt() {return _inputs.col(0);};

		bool optimize(const Vec4f &_state_init, std::vector<NodePtr> *_path, std::vector<ObsPtr> &_obstacles) {
			bool ok = true;
			typedef CPPAD_TESTVECTOR(double) Dvector;
			int _n_Sobs = 0;
			int _n_Dobs = 0;
			for (ObsPtr &__obs : _obstacles) {
				switch(__obs->type()) {
					case(Objet::Type::STATIC):
						_n_Sobs++;
						break;
					case(Objet::Type::DYNAMIC):
						_n_Dobs++;			
						break;
				}
			}
			size_t ng = 4*(_N) + (_n_Sobs+_n_Dobs)*(_N+1);
			size_t nx = 2*_N + 4*(_N+1);
			Dvector xi(nx), xl(nx), xu(nx);
			for (int i=0; i<_N+1; i++) {
				int p = i*6;
				xi[p] = 0.0;
				xl[p] = -150;
				xu[p] = 150;
				xi[p+1] = 0.0;
				xl[p+1] = -150;
				xu[p+1] = 150;
				xi[p+2] = 0.0;
				xl[p+2] = 0.0;
				xu[p+2] = x2U;
				xi[p+3] = 0.0;
				xl[p+3] = -x3U;
				xu[p+3] = x3U;
				if (i==_N)
					break;
				xi[p+4] = 0.0;
				xl[p+4] = -u0U;
				xu[p+4] = u0U;
				xi[p+5] = 0.0;
				xl[p+5] = -2*u1U;
				xu[p+5] = u1U;
			}
			for (int i=0; i<4; i++) {
				xi[i] = _state_init(i);
				xl[i] = _state_init(i);
				xu[i] = _state_init(i);
			}
			Dvector gl(ng), gu(ng);
			for (int i=0; i<_N; i++) {
				int p = i*4;
				gl[p] = 0.0;
				gu[p] = 0.0;
				gl[p+1] = 0.0;
				gu[p+1] = 0.0;
				gl[p+2] = 0.0;
				gu[p+2] = 0.0;
				gl[p+3] = 0.0;
				gu[p+3] = 0.0;
			}
			for (int j=0; j<_n_Sobs+_n_Dobs; j++) {
				for (int i=0; i<_N+1; i++) {
					int p = 4*_N + j*(_N+1) + i;
					gl[p] = CppAD::pow(_obstacles[j]->rad(),2);
					//gl[p] = 0.0;
					gu[p] = 1e13;
				}
			}
			std::string options;
			// Use sparse matrices for calculation of Jacobians and Hessians
			// with forward mode for Jacobian (seems to be faster for this case).
			options += "Sparse  true        forward\n";
			// turn off any printing
			options += "Integer print_level 5\n";
			options += "String  sb        yes\n";
			// maximum number of iterations
			options += "Integer max_iter    50\n";
			// approximate accuracy in first order necessary conditions;
			// see Mathematical Programming, Volume 106, Number 1,
			// Pages 25-57, Equation (6)
			options += "Numeric tol         1e-4\n";
			//1e-6
			FG_eval fg_eval(_N, _ts, _lr, _state_init, _path, _obstacles);
			CppAD::ipopt::solve_result<Dvector> solution;
			CppAD::ipopt::solve<Dvector, FG_eval>(options, xi, xl, xu, gl, gu, fg_eval, solution);
			ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
			_pred_states = MatXf::Zero(4, _N+1);
			_inputs = MatXf::Zero(2, _N);
			for (int i=0; i<_N; i++) {
				int p = i*6;
				_pred_states.col(i)(0) = solution.x[p];
				_pred_states.col(i)(1) = solution.x[p+1];
				_pred_states.col(i)(2) = solution.x[p+2];
				_pred_states.col(i)(3) = solution.x[p+3];
				_inputs.col(i)(0) = solution.x[p+4];
				_inputs.col(i)(1) = solution.x[p+5];
			}
			_pred_states.col(_N)(0) = solution.x[_N*6];
			_pred_states.col(_N)(1) = solution.x[_N*6+1];
			_pred_states.col(_N)(2) = solution.x[_N*6+2];
			_pred_states.col(_N)(3) = solution.x[_N*6+3];
			return ok;
		}
};

