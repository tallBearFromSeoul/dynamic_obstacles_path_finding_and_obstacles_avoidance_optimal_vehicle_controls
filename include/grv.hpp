#ifndef GRV_HPP
#define GRV_HPP
#include <random>
#include <Eigen/Dense>

typedef Eigen::VectorXf VecXf;
typedef Eigen::MatrixXf MatXf;

class GRV {
	private:
		std::mt19937 _gen;
		std::uniform_real_distribution<float> _uni_dis;
		int _n;
		float PI = 3.145927f;
		float _K;
		float _D;

		VecXf _mu;
		MatXf _sigma;
		MatXf _sigma_inv;
	public:
		void set_mu(const VecXf &__mu) {
			_mu=__mu;
		}
		void set_sigma(const MatXf &__sigma) {
			_sigma = __sigma;
		}
		VecXf mu() {return _mu;}
		MatXf sigma() {return _sigma;}

		GRV() {}
		// mu : mean | sigma : variance
		GRV(const VecXf &__mu, const MatXf &__sigma) : _mu(__mu), _sigma(__sigma) {
			_sigma_inv = _sigma.inverse();
			_n = __mu.rows();
			_D = float(_n);
			_K = 1.f/(pow(sqrt(2*PI),_D)*sqrt(_sigma.determinant()));
		};
		float operator()(const VecXf &__y) {
			return _K*exp(-0.5f*(__y-_mu).transpose()*_sigma_inv*(__y-_mu));
		}
		VecXf sample() {
			VecXf x(_n);
			//VecXf x = VecXf::NullaryExpr(_n,1,[&](){return _uni_dis(_gen);});
			VecXf sum(_n);
			sum.setZero();
			int num_iter = 30;
			for (int i = 0; i < num_iter; i++) {
				x.setRandom();
				//x = VecXf::NullaryExpr(_n,1,[&](){return _uni_dis(_gen);});
				x = 0.5 * (x + VecXf::Ones(_n));
				sum = sum + x;
			}
			sum = sum - (static_cast<double>(num_iter) / 2) * VecXf::Ones(_n);
			x = sum / (std::sqrt(static_cast<double>(num_iter) / 12));

			// Find the eigen vectors of the covariance matrix
			Eigen::SelfAdjointEigenSolver<MatXf> eigen_solver(_sigma);
			MatXf eigenvectors = eigen_solver.eigenvectors().real();

			// Find the eigenvalues of the covariance matrix
			MatXf eigenvalues = eigen_solver.eigenvalues().real().asDiagonal();
			
			// Find the transformation matrix
			Eigen::SelfAdjointEigenSolver<MatXf> es(eigenvalues);
			MatXf sqrt_eigenvalues = es.operatorSqrt();
			MatXf Q = eigenvectors * sqrt_eigenvalues;

			return Q * x + _mu;
		}

};

#endif
