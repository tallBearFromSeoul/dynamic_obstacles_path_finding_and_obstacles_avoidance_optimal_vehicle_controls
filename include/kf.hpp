#ifndef KF_HPP
#define KF_HPP
#include "grv.hpp"
#include <iostream>
#include <string>
#include <unordered_map>
#include <Eigen/Dense>
#include <fstream>

class KF {
private:
	float dt = 0.03333333f;
	Vec4f _mu;
	Mat4f _cov;
	Mat4f _A;
	Mat4f _H;
	Vec4f _xm0;
	Mat4f _Pm0;
	std::unordered_map<int, Vec4f> _xm;
	std::unordered_map<int, Mat4f> _Pm;
	GRV *_process_noise;
public:
	Vec4f xm(int __obs_id) {return _xm[__obs_id];}
	Mat4f Pm(int __obs_id) {return _Pm[__obs_id];}
	std::unordered_map<int, Vec4f> *xm() {return &_xm;}
	std::ofstream kf_output;

	KF(std::string _s) {
		_mu = Vec4f::Zero();
		_cov << 0.015f,0.f,0.f,0.f,
						0.f,0.015f,0.f,0.f,
						0.f,0.f,0.001f,0.f,
						0.f,0.f,0.f,0.001f;
		_A << 1.f,0.f,dt,0.f,
					0.f,1.f,0.f,dt,
					0.f,0.f,1.f,0.f,
					0.f,0.f,0.f,1.f;
		_H = Mat4f::Identity();
		_xm0 = Vec4f::Zero();
		_Pm0 << 1.f,0.f,0.f,0.f,
						0.f,1.f,0.f,0.f,
						0.f,0.f,1.f,0.f,
						0.f,0.f,0.f,1.f;
		_process_noise = new GRV(_mu, _cov);
		kf_output = std::ofstream("../sample_output/"+_s+".csv");
		kf_output << "obs_id,true_value,meas_value,corr_value,diff(%)\n";
	};

	//There are two types of algorithms :
	// - using Kalman Gain method (with and without joseph's form)
	// - using recursion
	// implement both and compare.
	//

	// we will initially assume the obstacle to follow the linear system dynamics equations such that : 
	//  - obstacles do not accelerate and maintain constant velocity decided during instantiation.
	//  - they do not change direction of velocity as well.
	// Under these assumptions or settings, we can define the system dynamics of an obstacle as :
	//   dx/dt = v_x
	//   dy/dt = v_y
	// 	 x(k+1) = x(k) + v_x*dt;
	// 	 y(k+1) = y(k) + v_y*dt;
	// 	 or,
	// 	 A = dX/dt = {{1,0,dt,0},
	// 	 					    {0,1,0,dt},
	// 	 					    {0,0,1,0},
	// 	 					    {0,0,0,1}}
	//   measurement variable z without noise is : 
	//   z = {x,y,v_x,v_y}T
	//   z = H*X
	//   H = {{1,0,0,0},
	//        {0,1,0,0},
	//        {0,0,1,0},
	//        {0,0,0,1}}
	// With Gaussian noises v and w present, 
	// X(k) = A*x(k-1) + v(k-1)
	// z(k) = H*x(k) + w(k)
	// where,
	// v(k) -> sensor noise
	// w(k) -> process noise

	// Kalman Gain method
	std::unordered_map<int, Vec4f> *update_kalman_gain(const std::vector<ObsPtr> &__obstacles_in_range, Li_Radar *__lidar, bool __joseph) {
		for (const ObsPtr &__obs : __obstacles_in_range) {
			update_kalman_gain_helper(__obs, __lidar, __joseph);
		}
		return &_xm;
	}
	// computationally more efficient version
	// but numerically less stable
	// since the dimension is low (4) for this case so computation is really fast
	// So we are using Joseph's version for more accuracy
	void update_kalman_gain_helper(const ObsPtr &__obs, Li_Radar *__lidar, bool __joseph) {
		int obs_id = __obs->id();
		Vec4f v, xp, w, zbar;
		Mat4f Pp, K;
		v = __obs->sensor_noise();
		w = _process_noise->sample();
		zbar = _H*(_A*__obs->state() + v) + w;
		if (_xm.count(obs_id)) {
			xp = _A*_xm[obs_id];
			Pp = _A*_Pm[obs_id]*_A.transpose() + __lidar->cov();
		} else {
			xp = _A*_xm0;
			Pp = _A*_Pm0*_A.transpose() + __lidar->cov();
		}
		K = Pp*_H.transpose() * (_H*Pp*_H.transpose() + _cov).inverse();
		_xm[obs_id] = xp + K*(zbar - _H*xp);
		Mat4f I = Mat4f::Identity();
		if (__joseph)
			_Pm[obs_id] = (I - K*_H) * Pp * (I - K*_H).transpose() + K*__lidar->cov()*K.transpose();
		else
			_Pm[obs_id] = (I - K*_H) * Pp;
		kf_output << obs_id<<","<<__obs->state().transpose()<<","<<zbar.transpose()<<","<<_xm[obs_id].transpose()<<","<<((_xm[obs_id] - __obs->state()).array()*100.f / __obs->state().array()).transpose()<<"\n";
	}

	// Recursion method
	std::unordered_map<int, Vec4f> *update_recursive(const std::vector<ObsPtr> &__obstacles_in_range, Li_Radar *__lidar) {
		for (const ObsPtr &__obs : __obstacles_in_range) {
			update_recursive_helper(__obs, __lidar);
		}
		return &_xm;
	}
	void update_recursive_helper(const ObsPtr &__obs, Li_Radar *__lidar) {
		int obs_id = __obs->id();
		Vec4f v, xp, w, zbar;
		Mat4f Pp;
		v = __obs->sensor_noise();
		w = _process_noise->sample();
		zbar = _H*(_A*__obs->state() + v) + w;
		if (_xm.count(obs_id)) {
			xp = _A*_xm[obs_id];
			Pp = _A*_Pm[obs_id]*_A.transpose() + __lidar->cov();
		} else {
			xp = _A*_xm0;
			Pp = _A*_Pm0*_A.transpose() + __lidar->cov();
		}
		_xm[obs_id] = xp + Pp*_H.transpose() * (_H*Pp*_H.transpose() + _cov).inverse() * (zbar - _H*xp);
		_Pm[obs_id] = Pp - Pp*_H.transpose() * (_H*Pp*_H.transpose() + _cov).inverse() * _H*Pp;

		kf_output << obs_id<<","<<__obs->state().transpose()<<","<<zbar.transpose()<<","<<_xm[obs_id].transpose()<<","<<((_xm[obs_id] - __obs->state()).array()*100.f / __obs->state().array()).transpose()<<"\n";
	}
};
#endif
