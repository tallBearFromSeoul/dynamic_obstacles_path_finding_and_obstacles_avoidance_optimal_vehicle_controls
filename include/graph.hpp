#include "node.hpp"
#include <unordered_map>
#include <unordered_set>

class Graph {
	protected:
		std::vector<std::vector<int>> _adj_list;
		std::unordered_map<int, int> _nid2gid_map;
		std::unordered_map<int, int> _gid2nid_map;
		std::unordered_map<int, NodePtr> _nid2n_map;
		std::unordered_map<int, int> _nid2pnid_map;
		std::vector<NodePtr> _path;
		std::vector<bool> _visited;
		bool _path_found = false;
		int _n_nodes=0;
		int _n_edges=0;

	public:
		std::unordered_map<int, int> gid2nid_map() {return _gid2nid_map;};
		std::unordered_map<int, NodePtr> nid2n_map() {return _nid2n_map;};
		std::vector<std::vector<int>> graph() {return _adj_list;};
		std::vector<NodePtr> path() {return _path;};

		Graph() {};
		
		void add_node(const NodePtr &__n) {
			_gid2nid_map[_n_nodes] = __n->id();
			__n->report();
			_nid2gid_map[__n->id()] = _n_nodes;
			_nid2n_map[__n->id()] = __n;
			_adj_list.resize(_n_nodes+1);
			_n_nodes++;
		};

		void add_edge(const NodePtr &__src, const NodePtr &__dst) {
			if (!_nid2gid_map.count(__src->id())) {
				add_node(__src);
			} 
			if (!_nid2gid_map.count(__dst->id())) {
				add_node(__dst);
			}
			/*
			std::cout<<"\nadd_edge()\n";
			std::cout<<"src gid : "<<_nid2gid_map[__src->id()]<<"\n";
			std::cout<<"dst gid : "<<_nid2gid_map[__dst->id()]<<"\n";
			*/
			_adj_list[_nid2gid_map[__src->id()]].push_back(_nid2gid_map[__dst->id()]);
			_nid2pnid_map[__dst->id()] = __src->id();
			_n_edges++;
		}

		void reconnect(const NodePtr &__n_new, const NodePtr &__nb) {
			// parent or the node pointing to __nb will now point to __n_new now;
			int __pnid = _nid2pnid_map[__nb->id()];
			int __pgid = _nid2gid_map[__pnid];
			_nid2pnid_map[__nb->id()] = __n_new->id();
			// removing edge from previous parent to __nb
			_adj_list[__pnid].erase(std::remove(_adj_list[__pnid].begin(), _adj_list[__pnid].end(), __nb->id()), _adj_list[__pnid].end());
			/*
			for (int i=0; i<_adj_list[__pgid].size(); i++) {
				if (_adj_list[__pgid][i] == __nb->id()) {
					_adj_list[__pgid].erase(_adj_list[__pgid].begin()+i);
				}
			}
			*/
			add_edge(__n_new, __nb);
		}

		void dfs(const NodePtr &__src, const NodePtr &__dst) {
			_path.clear();
			_visited = std::vector<bool>(_n_nodes, false);
			std::vector<NodePtr> __path;
			dfs_helper(__src, __dst, __path);
		}

		void dfs_helper(const NodePtr &__src, const NodePtr &__dst, std::vector<NodePtr> __path) {
			/*
			std::cout<<"\ndfs_helper():\n";
			__src->report();
			std::cout<<"graph id : "<<_nid2gid_map[__src->id()]<<"\n";
			__dst->report();
			std::cout<<"graph id : "<<_nid2gid_map[__dst->id()]<<"\n";
			*/
			if (_path_found)
				return;
			if (__src == nullptr)
				return;
			int s_id = _nid2gid_map[__src->id()];
			if (_visited[s_id])
				return;
			__path.push_back(__src);
			if (__src == __dst) {
				_path = __path;
				_path_found = true;
				return;
			}
			for (int e_id : _adj_list[s_id]) {
				dfs_helper(_nid2n_map[_gid2nid_map[e_id]], __dst, __path);
			}
			_visited[s_id] = true;
		}

		void print_path() {
			std::cout<<"print_path():\n";
			for (const NodePtr &_n : _path) {
				_n->report();
				std::cout<<"^ graph id : "<<_nid2gid_map[_n->id()]<<"\n";
			}
			std::cout<<"end print_path():\n";
		}
};

