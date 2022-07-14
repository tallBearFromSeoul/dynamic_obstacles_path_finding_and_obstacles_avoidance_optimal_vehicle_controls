#include "grv.hpp"
#include <random>
#include <vector>
#include <cmath>
#include <iostream>

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Matrix2f Mat2f;
typedef Eigen::Matrix4f Mat4f;


class Objet {
	public:
		enum Shape {
			CIRCLE, RECTANGLE
		};
		enum Type {
			STATIC, DYNAMIC//, HYBRID
		};
		enum Behaviour {
			NONE, CONTROL, HORZ, VERT, DIAG, NEGVERT, NEGDIAG
		};
	protected:
		Shape _shape;
		Type _type;
		Behaviour _behav;
	public:
		Shape shape() {return _shape;};
		Type type() {return _type;};
		Objet(Shape __shape, Type __type, Behaviour __behav) : _shape(__shape), _type(__type), _behav(__behav) {};
		//virtual void update();
};


class Vehicle : public Objet {
	private:
		float _dt, _lf, _lr;

		float _x;
		float _y;
		float _v; 
		float _head;

	public:
		Vehicle(Shape __shape, Type __type, float __ts, float __lf, float __lr, const Vec4f &__z0, Behaviour __behav) : Objet(__shape, __type, __behav), _dt(__ts), _lf(__lf), _lr(__lr) {
			set_state(__z0);
		};
		
		Vec4f state() {
			return {_x, _y, _v, _head};
		}
		
		void set_state(const Vec4f &_state) {
			_x = _state(0);
			_y = _state(1);
			_v = _state(2);
			_head = _state(3);
		}

		float d_x(float __v, float __head, float __beta) {
			return __v*cos(__head+__beta);
		}
		float d_y(float __v, float __head, float __beta) {
			return __v*sin(__head+__beta);
		}
		float d_v(float __a) {
			return __a;
		}
		float d_head(float __v, float __beta) {
			return (__v/_lr) * sin(__beta);
		}
		Vec4f d_state(float __v, float __head, float __beta, float __a) {
			return Vec4f(d_x(__v,__head,__beta), d_y(__v,__head,__beta), d_v(__a), d_head(__v,__beta));
		}
		
		Vec4f update(const Vec2f &__u) {
			Vec4f k1 = d_state(_v, _head, __u(0), __u(1));
			Vec4f k2 = d_state(_v+(k1(2)/2)*_dt, _head+(k1(3)/2)*_dt, __u(0), __u(1));
			Vec4f k3 = d_state(_v+(k2(2)/2)*_dt, _head+(k2(3)/2)*_dt, __u(0), __u(1));
			Vec4f k4 = d_state(_v+k3(2)*_dt, _head+k3(3)*_dt, __u(0), __u(1));
			Vec4f res = (k1/6 + k2/3 + k3/3 + k4/6)*_dt;
			_x += res(0);
			_y += res(1);
			_v += res(2);
			_head += res(3);
			return state();
			/*
			float _beta = __u(0);
			float alpha = _head + _beta;
			// Euler Discretization not good approximation
			_x += _v * cos(alpha) * _dt;
			_y += _v * sin(alpha) * _dt;
			_head += (_v/_lr) * sin(_beta) * _dt;
			_v += __u(1) * _dt;
			return state();
			*/
		}
};


typedef Eigen::Matrix<float, 1, 2, Eigen::RowMajor> RowVec2f;

std::random_device _rd;
class Obs : public Objet {
	private:
		static int MAXID;
		int _id;
		std::mt19937 _gen;
		std::uniform_real_distribution<float> _uni_dis_x;
		std::uniform_real_distribution<float> _uni_dis_y;
		std::uniform_real_distribution<float> _uni_dis_vel;
		std::uniform_real_distribution<float> _uni_dis_p;

		float _rad;
		float _dt = 0.03333333f;
		RowVec2f _pos;
		RowVec2f _E;
		RowVec2f _vel;
		Vec4f _noise;
		
	public:
		void update_noise(const Vec4f &__noise) {_noise = __noise;}
		int id() {return _id;}
		float rad() {return _rad;}
		float pos(int i) {return _pos(i);}
		float vel(int i) {return _vel(i);}
		RowVec2f pos() {return _pos;}
		RowVec2f vel() {return _vel;}
		Vec4f sensor_noise() {return _noise;}
		Vec4f state() {return Vec4f(_pos(0), _pos(1), _vel(0), _vel(1));}

		Obs(float *max_bounds) : Objet(Shape::CIRCLE, Type::DYNAMIC, Behaviour::NONE) {
			_id = MAXID++;
			// https://www.researchgate.net/post/Kalman_filter_how_do_I_choose_initial_P_0
			// from this website, decided to initialize the covariance matrix to be at max 0.01.
			float x_l = max_bounds[0];
			float x_u = max_bounds[1];
			float y_l = max_bounds[2];
			float y_u = max_bounds[3];
			float v_l = max_bounds[4];
			float v_u = max_bounds[5];

			_rad = 1.f;
			_gen = std::mt19937(_rd());
			_uni_dis_x = std::uniform_real_distribution<float>(x_l, x_u);
			_uni_dis_y = std::uniform_real_distribution<float>(y_l, y_u);
			_uni_dis_vel = std::uniform_real_distribution<float>(v_l, v_u);
			_uni_dis_p = std::uniform_real_distribution<float>(0.f, 1.f);

			_pos(0) = _uni_dis_x(_gen);
			_pos(1) = _uni_dis_y(_gen);
			_vel = RowVec2f::NullaryExpr(1,2,[&](){
					if (_uni_dis_p(_gen) > 0.5f)
						return _uni_dis_vel(_gen);
					else
						return -_uni_dis_vel(_gen);
			});
			//_vel = RowVec2f::NullaryExpr(1,2,[&](){return _norm_dis_vel(_gen);});
		}

		void update() {
			if (_type != Type::DYNAMIC)
				return;
			switch (_behav) {
				case (Behaviour::NONE):
					_pos(0) += _vel(0)*_dt;
					_pos(1) += _vel(1)*_dt;
					break;
				case (Behaviour::CONTROL):
					break;
				case (Behaviour::HORZ):
					_pos(0) += 0.01;
					_pos(1) += 0.015;
					break;
				case (Behaviour::VERT):
					_pos(1) += 0.012;
					break;
				case (Behaviour::DIAG):
					_pos(0) += 0.018;
					_pos(1) -= 0.03;
					break;
				case (Behaviour::NEGVERT):
					_pos(1) -= 0.012;
					break;
				case (Behaviour::NEGDIAG):
					_pos(0) -= 0.04;
					_pos(1) -= 0.04;

			}
		}

		bool collision_free(const RowVec2f &__v, float _col_thresh) {
			switch(_shape) {
				case Shape::CIRCLE:
					if ((__v-_pos).norm() <= _rad+_col_thresh) {
						return false;
					} else {
						return true;
					}
				case Shape::RECTANGLE:
					return false;
			}
		}

};

typedef std::shared_ptr<Objet> ObjetPtr;
typedef std::shared_ptr<Obs> ObsPtr;
typedef std::shared_ptr<Vehicle> VehiclePtr;

class Li_Radar {
	private:
		float _range;
		Vec4f _mu;
		Mat4f _cov;
		GRV *_sensor_noise;

	public:
		Mat4f cov() {return _cov;}

		Li_Radar(float __range) : _range(__range) {
			_mu = Vec4f::Zero();
			//_cov = Mat4f::Zero();
			_cov << 0.04f,0.f,0.f,0.f,
							0.f,0.04f,0.f,0.f,
							0.f,0.f,0.001f,0.f,
							0.f,0.f,0.f,0.001f;
			_sensor_noise = new GRV(_mu, _cov);
		}

		void scan(const Vec2f &__pos, std::vector<ObsPtr> *__all_obstacles, std::vector<ObsPtr> &obstacles_in_range__) {
			RowVec2f pos(__pos(0), __pos(1));
			scan(pos, __all_obstacles, obstacles_in_range__);
		}

		void scan(const RowVec2f &__pos, std::vector<ObsPtr> *__all_obstacles, std::vector<ObsPtr> &obstacles_in_range__) {
			for (const ObsPtr &__obs : *__all_obstacles) {
				if (in_range(__pos, __obs)) {
					Vec4f v = _sensor_noise->sample();
					__obs->update_noise(v);
					obstacles_in_range__.push_back(__obs);
				}
			}
			//std::cout<<"out of #"<<__all_obstacles->size()<<" amount of obstacles, there are #"<<obstacles_in_range__.size()<<" of obstacles in range of "<<_range<<"\n";
		}
		
		bool in_range(const RowVec2f &__pos, const ObsPtr &__obs) {
			float norm = (__pos-__obs->pos()).norm();
			return norm < _range;
		}
};

class World {
	private:
		Vehicle *_vehicle;
		std::vector<ObsPtr> _obstacles;

	public:
		Vehicle *vehicle() {return _vehicle;}
		std::vector<ObsPtr> *obstacles() {return &_obstacles;}

		World() {}
		World(Vehicle *__vehicle, const std::vector<ObsPtr> &__obstacles) : _vehicle(__vehicle), _obstacles(__obstacles) {}

		void add_to_world(Vehicle *__vehicle) {
			_vehicle = __vehicle;
		}
		void add_to_world(const ObsPtr &__obs) {
			_obstacles.push_back(__obs);
		}
		void update() {
			//if (_vehicle != nullptr)
			//	_vehicle->update();
			for (const ObsPtr &__obs : _obstacles) {
				__obs->update();
			}
		}
};


class ObjetFactory {
	private:
	public:
		ObjetFactory() {}

		void createNObstacles(int __N, float *__max_bounds, std::vector<ObsPtr> *out__) {
			for (int i=0; i<__N; i++) {
				out__->push_back(std::make_shared<Obs>(__max_bounds));
			}
		}
		void createVehicle(Objet::Shape __shape, float __ts, float __lr, const Vec4f &__state_init, std::vector<VehiclePtr> &__out) {
			__out.push_back(std::make_shared<Vehicle>(__shape, Objet::Type::DYNAMIC, __ts, __lr, __lr, __state_init, Objet::Behaviour::CONTROL));
		}
};


