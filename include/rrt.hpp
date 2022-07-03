#include <random>
#include "graph.hpp"
#include "kdtree.hpp"
#include "vehicle.hpp"

typedef Eigen::Matrix2f Mat2f;

class RRT : public kdTree, public Graph {
	public:
		enum Status{
			NONE, TRAPPED, ADVANCED, REACHED, NO_NEIGHBORS
		};
	private:
	protected:
		const float PI = 3.145927;
		const float DEST_THRESH = 0.5f;
		const float COL_THRESH = 0.1f; 
		float SPEED_LIMIT = 1.1f;
		float STEER_LIMIT = 45.f;//40.f

		std::random_device _rd;
		std::mt19937 _gen;
		std::uniform_real_distribution<float> _uni_dis;
		std::normal_distribution<float> _norm_dis;
		std::vector<ObsPtr> *_obstacles;
		RowVec2f _v_init, _v_dest, _v_diff, _v_mid;
		Status _build_status=Status::NONE;
	public:
		Status build_status() {return _build_status;};
		RRT(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles) : _v_init(__v_init), _v_dest(__v_dest), kdTree(__v_init.cols()), _obstacles(__obstacles) {
			_kd_root = std::make_shared<TreeNode>(__v_init, 0);
			_v_diff = (_v_dest - _v_init) / 2.f;
			_v_mid = (_v_dest + _v_init) / 2.f;
			_gen = std::mt19937(_rd());
			_uni_dis = std::uniform_real_distribution<float>(-1.0,1.0);
			_norm_dis = std::normal_distribution<float>(0.f, 0.25f);
		};
		
		RRT(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles, int __k) : _v_init(__v_init), _v_dest(__v_dest), kdTree(__v_init.cols()), _obstacles(__obstacles) {
			_kd_root = std::make_shared<TreeNode>(__v_init, 0);
			_v_diff = (_v_dest - _v_init) / 2.f;
			_v_mid = (_v_dest + _v_init) / 2.f;
			_gen = std::mt19937(_rd());
			_uni_dis = std::uniform_real_distribution<float>(-1.0,1.0);
			_norm_dis = std::normal_distribution<float>(0.f, 0.25f);
			_build_status = build(__k);
		};
		virtual Status build(int __k) {
			Status status, latest_status;
			NodePtr last_node = nullptr;
			for (int i=0; i<__k; i++) {
				RowVec2f v_rand = random_config(last_node);
				status = extend(v_rand, last_node, latest_status);
				latest_status = status;
				if (status==Status::REACHED) {
					return status;
				}
			}
			return status;
		}
		
		Status extend(const RowVec2f &__v_rand, NodePtr &__last_node, Status latest_status) {
			NodePtr n_near, n_new;
			n_near = nearest(__v_rand);
			if (latest_status == Status::TRAPPED) {
				n_new = new_biased_config(n_near->val());
			} else {
				n_new = new_config(n_near->val());
			}
			if (!collision_free(n_new)) {
				return Status::TRAPPED;
			}
			__last_node = n_new;
			insert(_kd_root, n_new);
			if (_n_nodes == 0)
				add_edge(_kd_root, n_new);
			else
				add_edge(n_near, n_new);
			if ((n_new->val()-_v_dest).norm()<DEST_THRESH) {
				NodePtr n_dest = std::make_shared<Node>(_v_dest);
				add_edge(n_new, n_dest);
				dfs(_kd_root, n_dest);
				return Status::REACHED;
			} else {
				return Status::ADVANCED;
			}
		}
		
		bool collision_free(const NodePtr &__n) {
			bool res = true;
			RowVec2f __v = __n->val();
			for (int i=0; i<_obstacles->size(); i++) {
				res &= _obstacles->at(i)->collision_free(__v, COL_THRESH);
			}
			return res;
		}
		
		RowVec2f random_config(const NodePtr &__last_node) {
			RowVec2f res = RowVec2f::NullaryExpr(1,2,[&](){return _uni_dis(_gen);});
			if (__last_node != nullptr)
				res += __last_node->val();
			return res;
		}

		NodePtr new_biased_config(const RowVec2f &__v_near) {
			RowVec2f diff_normal = _v_dest-__v_near;
			diff_normal.normalize();
			float theta = (1-_norm_dis(_gen)) * STEER_LIMIT*1.5;
			Vec2f rotated = rotate(deg_to_rad(theta))*diff_normal.transpose();
			RowVec2f new_v = __v_near + SPEED_LIMIT*rotated.transpose();
			return std::make_shared<Node>(new_v);
		}

		NodePtr new_config(const RowVec2f &__v_near) {	
			RowVec2f diff_normal = _v_dest-__v_near;
			diff_normal.normalize();
			float theta = _uni_dis(_gen) * STEER_LIMIT;
			Vec2f rotated = rotate(deg_to_rad(theta))*diff_normal.transpose();
			RowVec2f new_v = __v_near + SPEED_LIMIT*rotated.transpose();
			return std::make_shared<Node>(new_v);
		}
		
		Mat2f rotate(float __theta) {
			Mat2f rot_mat {{cos(__theta), -sin(__theta)}, {sin(__theta), cos(__theta)}};
			return rot_mat;
		}

		float deg_to_rad(float __deg) {
			return PI*__deg/180.f;
		}
};


class RRTStar : public RRT {
	protected:
			const float NEIGH_THRESH = 2.0f;
		const int REBUILD_ITER_MAX = 20;
		std::unordered_map<int, float> _nid2cost_map;
		NodePtr n_dest;
	public:
		RRTStar(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles, int __k) : RRT(__v_init, __v_dest, __obstacles) {
			_build_status = build(__k);};
		
		Status build(int __k) {
			Status status, latest_status;
			NodePtr last_node = nullptr;
			_nid2cost_map[_kd_root->id()] = 0.f;
			for (int i=0; i<__k; i++) {
				RowVec2f v_rand = random_config(last_node);
				status = extend(v_rand, last_node, latest_status,  _kd_root, false);
				latest_status = status;
				if (status==RRT::Status::REACHED) {
					return status;
				}
			}
			return status;
		}
	
		Status update(const Vec2f &__cur_p) {
			Status status, latest_status;
			NodePtr n_cur, n_near, last_node;
			n_cur = std::make_shared<Node>(__cur_p);
			last_node = n_cur;
			for (int i=0; i<REBUILD_ITER_MAX; i++) {
				RowVec2f v_rand = random_config(last_node);
				status = extend(v_rand, last_node, latest_status, n_cur, true);
				latest_status = status;
				if (status == RRT::Status::REACHED) {
					return status;
				}
			}
			n_near = nearest(__cur_p);
			std::vector<NodePtr> nbs = neighbors(n_near, NEIGH_THRESH);
			nbs.insert(nbs.begin(), n_near);
			std::sort(nbs.begin(),nbs.end(),[this](const NodePtr &a, const NodePtr &b) -> bool {return _nid2cost_map[a->id()] < _nid2cost_map[b->id()];});
			dfs(n_cur, nbs, n_dest);
			if (!_path_found) {
				for (const NodePtr &__nb : nbs) {
					last_node = n_cur;	
					for (int i=0; i<REBUILD_ITER_MAX; i++) {
						RowVec2f v_rand = random_config(last_node);
						status = extend(v_rand, last_node, latest_status, __nb, true);
						latest_status = status;
						if (status == RRT::Status::REACHED) {
							return status;
						}
					}	
				}
			}
			return status;
		}

		Status extend(const RowVec2f &__v_rand, NodePtr &__last_node, Status latest_status, const NodePtr &__n_init, bool __rebuild) {
			NodePtr n_near, n_new;
			n_near = nearest(__v_rand);
			if (latest_status == Status::TRAPPED) {
				n_new = RRT::new_biased_config(n_near->val());
			} else {
				n_new = RRT::new_config(n_near->val());
			}
			if (!collision_free(n_new)) {
				return Status::TRAPPED;
			}
			std::vector<NodePtr> nbs = neighbors(n_new, NEIGH_THRESH);
			if (nbs.size() == 0)
				return Status::NO_NEIGHBORS;
			NodePtr n_min = choose_parent(nbs, n_near, n_new);
			__last_node = n_min;
			
			insert(_kd_root, n_new);
			add_edge(n_min, n_new);
			_nid2cost_map[n_new->id()] = _nid2cost_map[n_min->id()] + (n_new->val()-n_min->val()).norm();
			rewire(nbs, n_min, n_new);
			if ((n_min->val()-_v_dest).norm()<DEST_THRESH) {
				n_dest = std::make_shared<Node>(_v_dest);
				add_edge(n_min, n_dest);
				_nid2cost_map[n_dest->id()] = _nid2cost_map[n_min->id()] + (_v_dest-n_min->val()).norm();
				dfs(__n_init, n_dest);
				if (_path_found)
					return Status::REACHED;
				/*
				if (__rebuild) {
					std::vector<NodePtr> nbs = neighbors(__n_init, NEIGH_THRESH);
					for (const NodePtr &__nb : nbs) {
						dfs(__nb, n_dest);
						if (_path_found)
							return Status::REACHED;
					}
				}
				*/
			} else {
				return Status::ADVANCED;
			}
		}

		void rewire(const std::vector<NodePtr> &__nbs, const NodePtr &__n_min, const NodePtr &__n_new) {
			for (const NodePtr &__nb : __nbs) {
				NodePtr n_p = RRT::new_config(__n_new->val());
				if (!collision_free(n_p)) {
					continue;
				}
				if ((n_p->val()-__n_new->val()).norm() + _nid2cost_map[__n_new->id()] < _nid2cost_map[__nb->id()]) {
					reconnect(__n_new, __nb);
				}
			}
			
		}

		NodePtr choose_parent(const std::vector<NodePtr> &__nbs, const NodePtr &__n_near, const NodePtr &__n_new) {
			float c_min = (__n_new->val()-__n_near->val()).norm() + _nid2cost_map[__n_near->id()];
			NodePtr z_min = __n_near;
			for (const NodePtr &__nb : __nbs) {
				NodePtr n_p = RRT::new_config(__nb->val());
				if (!collision_free(n_p)) {
					continue;
				}
				float c_p = (n_p->val()-__nb->val()).norm() + _nid2cost_map[__nb->id()];
				if (c_p < _nid2cost_map[__n_new->id()] && c_p < c_min) {
					c_min = c_p;
					z_min = __nb;
				}
			}
			return z_min;
		}

};


