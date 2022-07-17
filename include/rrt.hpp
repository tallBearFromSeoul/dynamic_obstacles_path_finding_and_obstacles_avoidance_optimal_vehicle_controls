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
 		const float INIT_OFFSET = 0.f;
 		const float DEST_OFFSET = 0.f;
 		float SPEED_LIMIT = 1.0f;
 		float STEER_LIMIT = 35.f;//40.f

 		std::random_device _rd;
 		std::mt19937 _gen;
 		std::uniform_real_distribution<float> _uni_dis, _uni_dis_x, _uni_dis_y;
 		std::vector<ObsPtr> *_obstacles;
 		RowVec2f _v_init, _v_dest, _v_diff, _v_mid;
 		Status _build_status=Status::NONE;
	public:
		Status build_status() {return _build_status;};
		RRT(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles) : _v_init(__v_init), _v_dest(__v_dest), kdTree(__v_init.cols()), _obstacles(__obstacles), DEST_OFFSET(std::max(__v_dest(0),__v_dest(1))/3.f) {
			_kd_root = std::make_shared<TreeNode>(__v_init, 0);
			_v_diff = (_v_dest - _v_init) / 2.f;
 			_v_mid = (_v_dest + _v_init) / 2.f;
 			_gen = std::mt19937(_rd());
 			_uni_dis = std::uniform_real_distribution<float>(-1.0,1.0);
 			_uni_dis_x = std::uniform_real_distribution<float>(std::min(_v_init(0),_v_dest(0))-INIT_OFFSET, std::max(_v_init(0),_v_dest(0))+DEST_OFFSET);
 			_uni_dis_y = std::uniform_real_distribution<float>(std::min(_v_init(1),_v_dest(1))-INIT_OFFSET, std::max(_v_init(1),_v_dest(1))+DEST_OFFSET);
 		};

 		RRT(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles, int __k) : _v_init(__v_init), _v_dest(__v_dest), kdTree(__v_init.cols()), _obstacles(__obstacles) {
			_kd_root = std::make_shared<TreeNode>(__v_init, 0);
			_v_diff = (_v_dest - _v_init) / 2.f;
 			_v_mid = (_v_dest + _v_init) / 2.f;
 			_gen = std::mt19937(_rd());
 			_uni_dis = std::uniform_real_distribution<float>(-1.0,1.0);
 			_uni_dis_x = std::uniform_real_distribution<float>(std::min(_v_init(0),_v_dest(0))-INIT_OFFSET, std::max(_v_init(0),_v_dest(0))+DEST_OFFSET);
 			_uni_dis_y = std::uniform_real_distribution<float>(std::min(_v_init(1),_v_dest(1))-INIT_OFFSET, std::max(_v_init(1),_v_dest(1))+DEST_OFFSET);
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
 			Vec2f __v = __n->val().transpose();
 			return collision_free(__v);
 		}
 		bool collision_free(const Vec2f &__v) {
 			bool res = true;
 			for (int i=0; i<_obstacles->size(); i++) {
 				res &= _obstacles->at(i)->collision_free(__v, COL_THRESH);
 			}
			return res;
		}
		
		RowVec2f random_config() {
			RowVec2f res = RowVec2f::Zero();
			res(0) = _uni_dis_x(_gen);
			res(1) = _uni_dis_y(_gen);
			return res;
		}
		RowVec2f random_config(const Vec2f &__cur_p) {
			RowVec2f res = RowVec2f::Zero();
			res(0) = std::uniform_real_distribution<float>{std::min(__cur_p(0),_v_dest(0))-1.f, std::max(__cur_p(0),_v_dest(0))+1.f}(_gen);
			res(1) = std::uniform_real_distribution<float>{std::min(__cur_p(1),_v_dest(1))-1.f, std::max(__cur_p(1),_v_dest(1))+1.f}(_gen);
			return res;
		}
		RowVec2f random_config(const NodePtr &__last_node) {
			RowVec2f res = RowVec2f::NullaryExpr(1,2,[&](){return _uni_dis(_gen);});
			//if (__last_node != nullptr)
			//	res += __last_node->val();
			return res;
		}
 		NodePtr new_biased_config(const RowVec2f &__v_near) {
 			RowVec2f diff_normal = _v_dest-__v_near;
 			diff_normal.normalize();
 			float theta = (1-_uni_dis(_gen)) * STEER_LIMIT*1.5;
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
		NodePtr new_config(const RowVec2f &__src, const RowVec2f &__dst) {
			RowVec2f diff_normal = __dst-__src;
			diff_normal.normalize();
			float theta = _uni_dis(_gen) * STEER_LIMIT;
			Vec2f rotated = rotate(deg_to_rad(theta))*diff_normal.transpose();
			RowVec2f new_v = __src + SPEED_LIMIT*rotated.transpose();
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
	private:
	protected:
		const float NEIGH_THRESH = 1.f;
		const int REBUILD_ITER_MAX = 20;
		std::unordered_map<int, float> _nid2cost_map;
		NodePtr n_dest;
	public:
		RRTStar(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles) : RRT(__v_init, __v_dest, __obstacles) {}
		RRTStar(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles, int __k) : RRT(__v_init, __v_dest, __obstacles) {
			_build_status = build(__k);
		}
		virtual Status build(int __k) {
			Status latest_status = Status::NONE;
			NodePtr n_init = _kd_root;
			_nid2cost_map[_kd_root->id()] = 0.f;
			for (int i=0; i<__k; i++) {
				// Uniform Random Sampling of a vertex in the range of _v_init, and _v_dest.
				RowVec2f v_rand = random_config();
				// Obtain nearest node from the vertex through KD Tree.
				NodePtr n_near, n_new;
				n_near = nearest(v_rand);
				// Generate a new node from the nearest node 
				// where the new node will be stochastically rotated
				// and placed towards _v_dest
				// while conforming to the steer limit.
				n_new = new_config(n_near->val());
				// temporary initial cost assignment to n_near
				float cost = (n_new->val()-n_near->val()).norm();
				_nid2cost_map.insert({n_new->id(), _nid2cost_map.at(n_near->id()) + cost});
				// Collision check of the new node
				// if the check fails then just skip the iteration;
				if (!collision_free(n_new)) {
					n_new.reset();
					latest_status = Status::TRAPPED;
					continue;
				}
				// Obtain neighboring nodes withing the Ball of radius NEIGH_THRESH=1.f, 
				// from the node n_new using KD Tree.
				std::vector<NodePtr> nbs = neighbors(n_new, NEIGH_THRESH);
				
				NodePtr n_min;
				// Compute the n_min node,
				// which is the node that will be rewired such that
				// it will become the parent of the n_new node.
				if (nbs.size() == 0) {
					// If there is no neighboring nodes within the Ball of radius NEIGH_THRESH=1.f found,
					// the nearest node from the n_new node will be used as n_min.
					n_min = nearest(n_new->val());
				} else {
					// Compute the n_min node,
					// which is the node with the minimum cost out of the neighbors.
					n_min = choose_parent(nbs, n_near, n_new, &_nid2cost_map);
				}
				// Insert the n_new node into the KD Tree and the graph,
				// such that we can include the node into our search next iterations.
				insert(_kd_root, n_new);
				add_edge(n_min, n_new);
				// Update the cost of the n_new node,
				// which will just be the Sum of costs of n_min node 
				// and the norm of the difference of n_min and n_new vertices.
				_nid2cost_map[n_new->id()] = _nid2cost_map[n_min->id()] + (n_new->val()-n_min->val()).norm();
				// Perform the rewiring 
				rewire(nbs, n_min, n_new, this, &_nid2cost_map);
				
				if ((n_min->val()-_v_dest).norm()<DEST_THRESH) {
					n_dest = std::make_shared<Node>(_v_dest);
					add_edge(n_min, n_dest);
					_nid2cost_map[n_dest->id()] = _nid2cost_map[n_min->id()] + (_v_dest-n_min->val()).norm();
					dfs(n_init, n_dest);
					if (_path_found)
						return Status::REACHED;
				} 
				latest_status = Status::ADVANCED;
				//status = extend(v_rand, last_node, latest_status, _kd_root, false);
			}
			return latest_status;
		}
		void rewire(const std::vector<NodePtr> &__nbs, const NodePtr &__n_min, const NodePtr &__n_new, Graph *graph, std::unordered_map<int, float> *__nid2cost_map) {
			for (const NodePtr &__nb : __nbs) {
				NodePtr n_p = RRT::new_config(__n_new->val());
				if (!collision_free(n_p)) {
					continue;
				}
				if ((n_p->val()-__n_new->val()).norm() + __nid2cost_map->at(__n_new->id()) < __nid2cost_map->at(__nb->id())) {
					float cost = __nid2cost_map->at(__n_new->id());
					__nid2cost_map->insert({__nb->id(), (n_p->val()-__n_new->val()).norm() + cost});
					graph->reconnect(__n_new, __nb);
				}
			}
			
		}
		NodePtr choose_parent(const std::vector<NodePtr> &__nbs, const NodePtr &__n_near, const NodePtr &__n_new, std::unordered_map<int, float> *__nid2cost_map) {
			float c_min = (__n_new->val()-__n_near->val()).norm() + __nid2cost_map->at(__n_near->id());
			NodePtr z_min = __n_near;
			for (const NodePtr &__nb : __nbs) {
				NodePtr n_p = RRT::new_config(__nb->val());
				if (!collision_free(n_p)) {
					continue;
				}
				float c_p = (n_p->val()-__nb->val()).norm() + __nid2cost_map->at(__nb->id());
				if (c_p < __nid2cost_map->at(__n_new->id()) && c_p < c_min) {
					c_min = c_p;
					z_min = __nb;
				}
			}
			__nid2cost_map->insert({z_min->id(), c_min});
			return z_min;
		}
		Status update(const Vec2f &__cur_p) {
			Status latest_status;
			NodePtr n_cur, n_near;
			n_cur = std::make_shared<Node>(__cur_p);
			_nid2cost_map[n_cur->id()] = 0.f;
			add_node(n_cur);
			std::vector<NodePtr> nbs = neighbors(n_cur, NEIGH_THRESH*2.f);
			if (nbs.size() == 0) {
				n_near = nearest(__cur_p);
				nbs = {n_near};
			}
			dfs(n_cur, nbs, n_dest);
			if (_path_found)
				return Status::REACHED;
			for (int i=0; i<REBUILD_ITER_MAX; i++) {
					RowVec2f v_rand = random_config(__cur_p);
					NodePtr n_near, n_new;
					n_near = nearest(v_rand);
					n_new = new_config(n_near->val());
					float cost = (n_new->val()-n_near->val()).norm();
					_nid2cost_map.insert({n_new->id(), _nid2cost_map.at(n_near->id()) + cost});
					if (!collision_free(n_new)) {
						n_new.reset();
						latest_status = Status::TRAPPED;
						continue;
					}
					std::vector<NodePtr> nbs = neighbors(n_new, NEIGH_THRESH);
					NodePtr n_min;
					if (nbs.size() == 0) {
						n_min = nearest(n_new->val());
					} else {
						n_min = choose_parent(nbs, n_near, n_new, &_nid2cost_map);
					}
					insert(_kd_root, n_new);
					add_edge(n_min, n_new);
					_nid2cost_map[n_new->id()] = _nid2cost_map[n_min->id()] + (n_new->val()-n_min->val()).norm();
					rewire(nbs, n_min, n_new, this, &_nid2cost_map);
					//status = extend(v_rand, last_node, latest_status, n_cur, true);
					if (latest_status == Status::REACHED) {
						return latest_status;
					}
					latest_status = Status::ADVANCED;
				}
			
			nbs = neighbors(n_cur, NEIGH_THRESH);
			if (nbs.size() == 0) {
				n_near = nearest(__cur_p);
				nbs = {n_near};
			}
			dfs(n_cur, nbs, n_dest);
			insert(_kd_root, n_cur);
			if (_path_found)
				return Status::REACHED;
			return latest_status;
		}
};
class BRRTStar : public RRTStar {
	private:
	protected:
		NodePtr n_dest_B = nullptr;
		Graph *_graph_B = nullptr;
		kdTree *_kd_tree_B = nullptr;
		TreeNodePtr _kd_root_B = nullptr;
		std::unordered_map<int, float> _nid2cost_map_B;
		std::pair<std::vector<NodePtr>, float> _b_path;
	public:
		std::unordered_map<int, std::vector<int>> *graphB() {return _graph_B->graph();}
		std::unordered_map<int, int> gid2nid_mapB() {return _graph_B->gid2nid_map();}
		std::unordered_map<int, NodePtr> nid2n_mapB() {return _graph_B->nid2n_map();}
		BRRTStar(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles) : RRTStar(__v_init, __v_dest, __obstacles) {}
		BRRTStar(const RowVec2f &__v_init, const RowVec2f &__v_dest, std::vector<ObsPtr> *__obstacles, int __k) : RRTStar(__v_init, __v_dest, __obstacles) {
			_kd_root_B = std::make_shared<TreeNode>(__v_dest, 0);
			_kd_tree_B = new kdTree(__v_init.cols(), _kd_root_B);
			_graph_B = new Graph(_kd_root_B);
			n_dest = _kd_root_B;
			n_dest_B = _kd_root;
			std::vector<NodePtr> temp = {};
			_b_path = std::make_pair(temp, std::numeric_limits<float>::max()); 
			_build_status = build(__k);
		}
		Status build(int __k) {
			Status latest_status = Status::NONE;
			NodePtr n_init = _kd_root;
			_nid2cost_map[_kd_root->id()] = 0.f;
			_nid2cost_map_B[_kd_root_B->id()] = 0.f;
			Graph *graphA__ = this;
			Graph *graphB__ = _graph_B;
			kdTree *kdtreeA__ = this;
			kdTree *kdtreeB__ = _kd_tree_B;
			RowVec2f *vdestA__ = &_v_dest;
			RowVec2f *vdestB__ = &_v_init;
			TreeNodePtr kdrootA__ = _kd_root;
			TreeNodePtr kdrootB__ = _kd_root_B;
			std::unordered_map<int, float> *costmapA__ = &_nid2cost_map;
			std::unordered_map<int, float> *costmapB__ = &_nid2cost_map_B;
			for (int i=0; i<__k; i++) {
				// Uniform Random Sampling of a vertex in the range of _v_init, and _v_dest.
				RowVec2f v_rand = random_config();
				// Obtain nearest node from the vertex through KD Tree.
				NodePtr n_near, n_new;
				n_near = kdtreeA__->nearest(v_rand);
				// Generate a new node from the nearest node 
				// where the new node will be stochastically rotated
				// and placed towards _v_dest
				// while conforming to the steer limit.
				n_new = RRT::new_config(n_near->val(), *vdestA__);
				// temporary initial cost assignment to n_near
				float cost = (n_new->val()-n_near->val()).norm();
				costmapA__->insert({n_new->id(), costmapA__->at(n_near->id()) + cost});
				// Collision check of the new node
				// if the check fails then just skip the iteration;
				if (!collision_free(n_new)) {
					n_new.reset();
					latest_status = Status::TRAPPED;
					continue;
				}
				// Obtain neighboring nodes withing the Ball of radius NEIGH_THRESH=1.f, 
				// from the node n_new using KD Tree.
				std::vector<NodePtr> nbs = kdtreeA__->neighbors(n_new, NEIGH_THRESH);
				NodePtr n_min;
				// Compute the n_min node,
				// which is the node that will be rewired such that
				// it will become the parent of the n_new node.
				if (nbs.size() == 0) {
					// If there is no neighboring nodes within the Ball of radius NEIGH_THRESH=1.f found,
					// the nearest node from the n_new node will be used as n_min.
					n_min = kdtreeA__->nearest(n_new->val());
				} else {
					// Compute the n_min node,
					// which is the node with the minimum cost out of the neighbors.
					n_min = choose_parent(nbs, n_near, n_new, costmapA__);
				}
				// Insert the n_new node into the KD Tree and the graph,
				// such that we can include the node into our search next iterations.
				kdtreeA__->insert(kdrootA__, n_new);
				graphA__->add_edge(n_min, n_new);
				
				// Update the cost of the n_new node,
				// which will just be the Sum of costs of n_min node 
				// and the norm of the difference of n_min and n_new vertices.
				cost = (n_new->val()-n_min->val()).norm();
				costmapA__->insert({n_new->id(), costmapA__->at(n_min->id()) + cost});
				// Perform the rewiring 
				rewire(nbs, n_min, n_new, graphA__, costmapA__);
		
				NodePtr n_connect;
				n_connect = kdtreeB__->nearest(n_new->val());
				std::pair<std::vector<NodePtr>, float> c_path = connect(n_new, n_connect, kdrootB__, kdtreeB__, graphA__, graphB__, costmapA__, costmapB__);	
				if (c_path.second < _b_path.second) {
					_b_path = c_path;
					_path = c_path.first;
				}
				if (_b_path.second != std::numeric_limits<float>::max())
					latest_status = Status::REACHED;
				std::swap(graphA__,graphB__);
				std::swap(kdtreeA__,kdtreeB__);
				std::swap(costmapA__,costmapB__);
				std::swap(vdestA__,vdestB__);
				std::swap(kdrootA__,kdrootB__);
			}
			return latest_status;
		}
		
		std::pair<std::vector<NodePtr>, float> connect(const NodePtr &__n_new, const NodePtr &__n_conn, const TreeNodePtr &__kd_root_B, kdTree *__kd_treeB, Graph *__graphA, Graph *__graphB, std::unordered_map<int, float> *__nid2cost_mapA, std::unordered_map<int, float> *__nid2cost_mapB) {
			NodePtr n_new_B = new_config(__n_conn->val(), __n_new->val());
			float norm = (n_new_B->val()-__n_conn->val()).norm();
			__nid2cost_mapB->insert({n_new_B->id(), __nid2cost_mapB->at(__n_conn->id()) + norm});
			if (!collision_free(n_new_B)) {
				n_new_B.reset();
				std::vector<NodePtr> temp = {};
				return std::make_pair(temp, std::numeric_limits<float>::max());
			}
			std::vector<NodePtr> nbs_B = __kd_treeB->neighbors(n_new_B, NEIGH_THRESH);
			NodePtr n_min_B;
			if (nbs_B.size() == 0) {
				n_min_B = __kd_treeB->nearest(n_new_B->val());
			} else {
				n_min_B = choose_parent(nbs_B, __n_conn, n_new_B, __nid2cost_mapB);
			}
			__kd_treeB->insert(__kd_root_B, n_new_B);
			__graphB->add_edge(n_min_B, n_new_B);
			__nid2cost_mapB->insert({n_new_B->id(), __nid2cost_mapB->at(n_min_B->id()) + (n_new_B->val()-n_min_B->val()).norm()});
			rewire(nbs_B, n_min_B, n_new_B, __graphB, __nid2cost_mapB);
			return dfs_connect(__n_new, n_new_B, __graphA, __graphB, __nid2cost_mapA, __nid2cost_mapB);
		}
		std::vector<NodePtr> update(const Vec2f &__cur_p) {
			Status latest_status;
			NodePtr n_cur, n_near;
			n_cur = std::make_shared<Node>(__cur_p);
			_nid2cost_map_B[n_cur->id()] = 0.f;
			_graph_B->add_node(n_cur);
			std::vector<NodePtr> nbs_B = _kd_tree_B->neighbors(n_cur, NEIGH_THRESH);
			if (nbs_B.size() == 0) {
				NodePtr n_near = _kd_tree_B->nearest(n_cur->val());
				nbs_B = {n_near};
			}
			return dfs_connect(n_cur, nbs_B, this, _graph_B, _kd_tree_B, &_nid2cost_map, &_nid2cost_map_B);
		}
		std::pair<std::vector<NodePtr>, float> dfs_connect(const NodePtr &__n_new, const NodePtr &__n_new_B, Graph *__graphA, Graph *__graphB, std::unordered_map<int, float> *__nid2cost_mapA, std::unordered_map<int, float> *__nid2cost_mapB) {
			std::vector<NodePtr> path_A = __graphA->dfs(_kd_root, __n_new);
			std::vector<NodePtr> path_B = __graphB->dfs(n_dest, __n_new_B);
			std::reverse(path_B.begin(), path_B.end());
			path_A.insert(path_A.end(), path_B.begin(), path_B.end());
			float path_sum = __nid2cost_mapA->at(__n_new->id()) + __nid2cost_mapB->at(__n_new_B->id());
			return std::make_pair(path_A, path_sum);
		}
	
		std::vector<NodePtr> dfs_connect(const NodePtr &__n_cur, const std::vector<NodePtr> &__nbs_B, Graph *__graphA, Graph *__graphB, kdTree *__kdtreeB, std::unordered_map<int, float> *__nid2cost_mapA, std::unordered_map<int, float> *__nid2cost_mapB) {
			float c_min = std::numeric_limits<float>::max();
			NodePtr best_nb_B;
			for (const NodePtr &__nb : __nbs_B) {
				float cost = __nid2cost_mapB->at(__nb->id());
				if (cost < c_min) {
					c_min = cost;
					best_nb_B = __nb;
				}
			}
			std::vector<NodePtr> path_B;
			if (__graphB->dfs_path_B(n_dest, best_nb_B)) {
				__graphB->path_B(path_B);
				std::reverse(path_B.begin(), path_B.end());
			}
			return path_B;
		}
};	
