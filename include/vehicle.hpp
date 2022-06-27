#include <random>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector4f Vec4f;

class Vehicle {
	private:
		float _ts, _lf, _lr;

		float _x;
		float _y;
		float _v; 
		float _head;

	public:
		Vehicle(float __ts, float __lf, float __lr, const Vec4f &__z0) : _ts(__ts), _lf(__lf), _lr(__lr) {
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

		Vec4f update(const Vec2f &__z) {
			float _beta = __z(0);
			float alpha = _head + _beta;
			_x += _v * cos(alpha) * _ts;
			_y += _v * sin(alpha) * _ts;
			_head += (_v/_lr) * sin(_beta) * _ts;
			_v += __z(1) * _ts;
			return state();
		}
};


class Objet {
	public:
		enum Shape {
			CIRCLE, RECTANGLE
		};
		enum Type {
			STATIC, DYNAMIC//, HYBRID
		};	
	protected:
		Shape _shape;
		Type _type;
	public:
		Shape shape() {return _shape;};
		Type type() {return _type;};
		Objet(Shape __shape, Type __type) : _shape(__shape), _type(__type) {};
};

typedef Eigen::Matrix<float, 1, 2, Eigen::RowMajor> RowVec2f;

std::random_device _rd;
class Obs : public Objet {
	private:
		float _rad;
		RowVec2f _pos;
		std::mt19937 _gen;
		std::uniform_real_distribution<float> _uni_dis;
		std::normal_distribution<float> _norm_dis;

	public:
		float rad() {return _rad;};
		float pos(int i) {return _pos(i);};
		RowVec2f pos() {return _pos;};

		Obs(Shape __shape, Type __type, const RowVec2f &__pos, float __rad) : _pos(__pos), _rad(__rad), Objet(__shape, __type) {
			_gen = std::mt19937(_rd());
			_uni_dis = std::uniform_real_distribution<float>(-0.2,0.2);
		_norm_dis = std::normal_distribution<float>(0.f, 0.05f);
		};
		
		void update() {
			if (_type != Type::DYNAMIC)
				return;
			RowVec2f uni = RowVec2f::NullaryExpr(1,2,[&](){return _uni_dis(_gen);});
			RowVec2f norm = RowVec2f::NullaryExpr(1,2,[&](){return _norm_dis(_gen);});
			_pos += uni + norm;
			return;	
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

typedef std::shared_ptr<Obs> ObsPtr;
