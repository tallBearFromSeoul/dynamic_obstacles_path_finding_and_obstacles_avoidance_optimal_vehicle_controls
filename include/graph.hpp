#include "node.hpp"
#include <unordered_map>
#include <unordered_set>

class Graph {
	protected:
		std::unordered_map<int, std::vector<int>> _adj_list;
		std::unordered_map<int, int> _nid2gid_map;
		std::unordered_map<int, int> _gid2nid_map;
		std::unordered_map<int, NodePtr> _nid2n_map;
		std::unordered_map<int, int> _gid2pgid_map;
		std::vector<NodePtr> _path;
		std::vector<NodePtr> _path_B;
		bool _path_found = false;
		bool _path_B_found = false;
		int _n_nodes=0;
		int _n_edges=0;

	public:
		void path_B(std::vector<NodePtr> &out__) {
			out__ = _path_B;
		}
		std::unordered_map<int, int> gid2nid_map() {return _gid2nid_map;}
		std::unordered_map<int, NodePtr> nid2n_map() {return _nid2n_map;}
		std::unordered_map<int, std::vector<int>> *graph() {return &_adj_list;}
		virtual std::vector<NodePtr> *path() {return &_path;}

		Graph() {};
		Graph(const NodePtr &__root) {
			add_node(__root);
		}

		void add_node(const NodePtr &__n) {
			_gid2nid_map[_n_nodes] = __n->id();
			_nid2gid_map[__n->id()] = _n_nodes;
			_nid2n_map[__n->id()] = __n;
			_adj_list[_n_nodes] = {};
			_n_nodes++;
		};

		void add_edge(const NodePtr &__src, const NodePtr &__dst) {
			if (!_nid2gid_map.count(__src->id())) {
				add_node(__src);
			} 
			if (!_nid2gid_map.count(__dst->id())) {
				add_node(__dst);
			}
			__src->set_branch();
			_adj_list[_nid2gid_map[__src->id()]].push_back(_nid2gid_map[__dst->id()]);
			_gid2pgid_map[_nid2gid_map[__dst->id()]] = _nid2gid_map[__src->id()];
			_n_edges++;
		}

		void reconnect(const NodePtr &__n_new, const NodePtr &__nb) {
			// parent or the node pointing to __nb will now point to __n_new now;
			int __pgid = _gid2pgid_map[_nid2gid_map[__nb->id()]];
			
			_gid2pgid_map[_nid2gid_map[__nb->id()]] = _nid2gid_map[__n_new->id()];
			_adj_list[__pgid].erase(std::remove(_adj_list[__pgid].begin(), _adj_list[__pgid].end(), _nid2gid_map[__nb->id()]), _adj_list[__pgid].end());
			if (_adj_list[__pgid].size() == 0) {
				_adj_list.erase(__pgid);
			}
			add_edge(__n_new, __nb);
		}

		bool dfs_path_B(const NodePtr &__root_B, const NodePtr &__best_leaf_B) {
			_path_B.clear();
			_path_B_found = false;
			std::vector<NodePtr> path_;
			dfs_path_B_helper(__root_B, __best_leaf_B, path_);
			if (_path_B_found)
				return true;
			return false;
		}

		void dfs_path_B_helper(const NodePtr &__src, const NodePtr &__dst, std::vector<NodePtr> path__) {
		if (_path_B_found) 
			return;
		if (__src == nullptr)
			return;
		int s_id = _nid2gid_map[__src->id()];
		path__.push_back(__src);
		if (__src->id() == __dst->id()) {
			_path_B = path__;
			_path_B_found = true;
			return;
		}
		for (int gid_e : _adj_list[s_id]) {
			dfs_path_B_helper(_nid2n_map[_gid2nid_map[gid_e]], __dst, path__);
			}
		}

		void dfs(const NodePtr &__n_cur, const NodePtr &__n_near, const NodePtr &__dst) {
			dfs(__n_near, __dst);
			if (_path_found) {
				add_edge(__n_cur, __n_near);
				_path.insert(_path.begin(), __n_cur);
				return;
			}
		}

		void dfs(const NodePtr &__n_cur, const std::vector<NodePtr> &__nbs, const NodePtr &__dst) {
			for (const NodePtr &__nb : __nbs) {
				dfs(__nb, __dst);
				if (_path_found) {
					add_edge(__n_cur, __nb);
					_path.insert(_path.begin(), __n_cur);
					return;
				}
			}
		}
		
		std::vector<NodePtr> dfs(const NodePtr &__src, const NodePtr &__dst) {
			_path_found = false;
			_path.clear();
			std::vector<NodePtr> __path;
			dfs_helper(__src, __dst, __path);
			return _path;
		}

		void dfs_helper(const NodePtr &__src, const NodePtr &__dst, std::vector<NodePtr> __path) {
			if (_path_found) 
				return;
			if (__src == nullptr)
				return;
			int s_id = _nid2gid_map[__src->id()];
			__path.push_back(__src);
			if (__src == __dst) {
				_path = __path;
				_path_found = true;
				return;
			}
			for (int e_id : _adj_list[s_id]) {
				dfs_helper(_nid2n_map[_gid2nid_map[e_id]], __dst, __path);
			}
		}

};

