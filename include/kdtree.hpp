#include "node.hpp"

class kdTree {
	protected:
		int _k;
		TreeNodePtr _kd_root = nullptr;
	public:

		kdTree(int __k) : _k(__k) {};
		kdTree(const std::vector<NodePtr> &__nodes) {
			build(__nodes);
		};
		
		void build(const std::vector<NodePtr> &__nodes) {
			_k = __nodes[0]->dims();
			for (int i=0; i<__nodes.size(); i++) {
				_kd_root = insert(_kd_root, __nodes[i]);
			}
		}

		TreeNodePtr insert(const TreeNodePtr &__root, const NodePtr &__node2ins) {
			return insert_helper(__root, __node2ins, 0);
		}

		TreeNodePtr insert_helper(const TreeNodePtr &__node, const NodePtr &__node2ins, int __depth) {
			if (__node == nullptr) {
				return std::make_shared<TreeNode>(__node2ins, __depth);
			}
			// current dimension
			int cd = __depth % _k;
			if (__node2ins->val(cd) < __node->val(cd)) {
				__node->set_left(insert_helper(__node->left(), __node2ins, __depth+1));

			} else {
				__node->set_right(insert_helper(__node->right(), __node2ins, __depth+1));
			}
			return __node;
		}

		float diff_dim(const TreeNodePtr &__n0, const TreeNodePtr &__n1, int __depth) {
			return __n0->val(__depth)-__n1->val(__depth);
		}		
		float sq_diff(const NodePtr &__n0, const TreeNodePtr &__n1) {
			return (__n0->val()-__n1->val()).squaredNorm();
		}

		template<typename Derived>
		NodePtr nearest(const Eigen::MatrixBase<Derived> &__val) {
			TreeNodePtr node = std::make_shared<TreeNode>(__val, 0);
			NodePtr n = nearest(node);
			node.reset();
			return n;
		}
		
		NodePtr nearest(const NodePtr &__node2comp) {
			TreeNodePtr node = std::make_shared<TreeNode>(__node2comp, 0);
			NodePtr n = nearest(node);
			node.reset();
			return n;
		}

		NodePtr nearest(const TreeNodePtr &__node2comp) {
			float cost = sq_diff(_kd_root, __node2comp);
			return nearest_helper(_kd_root, __node2comp, 0, _kd_root, cost);
		}

		NodePtr nearest_helper(const TreeNodePtr &__node, const TreeNodePtr &__node2comp, int __depth, const NodePtr &__best, float __best_cost) {
			if (__node == nullptr)
				return nullptr;
			float diff = sq_diff(__node, __node2comp);
			float dx = diff_dim(__node, __node2comp, __depth);
			float dx2 = dx*dx; 
			NodePtr best_l = __best;
			float best_cost_l = __best_cost;
			if (diff < __best_cost) {
				best_cost_l = diff;
				best_l = __node;
			}
			int next_dim = (__depth+1) % _k;
			TreeNodePtr first, second;
			if (dx > 0) {
				first = __node->left();
				second = __node->right();
			} else {
				first = __node->right();
				second = __node->left();
			}

			NodePtr next = nearest_helper(first, __node2comp, next_dim, best_l, best_cost_l);
			if (next != nullptr) {
				float diff_l = sq_diff(next, __node2comp);
				if (diff_l < best_cost_l) {
					best_cost_l = diff_l;
					best_l = next;
				}
			}
			if (dx2 < best_cost_l) {
				next = nearest_helper(second, __node2comp, next_dim, best_l, __best_cost);
				if (next != nullptr) {
					float diff_l = sq_diff(next, __node2comp);
					if (diff_l < best_cost_l) {
						best_cost_l = diff_l;
						best_l = next;
					}
				}
			}
			return best_l;
		}
		
		template<typename Derived>
		std::vector<NodePtr> neighbors(const Eigen::MatrixBase<Derived> &__val, float __rad) {
			TreeNodePtr node = std::make_shared<TreeNode>(__val, 0);
			std::vector<NodePtr> nbs = neighbors(node, __rad);
			node.reset();
			return nbs;
		}
		std::vector<NodePtr> neighbors(const NodePtr &__node2comp, float __rad) {
			TreeNodePtr node = std::make_shared<TreeNode>(__node2comp, 0);
			std::vector<NodePtr> nbs = neighbors(node, __rad);
			node.reset();
			return nbs;
		}
		std::vector<NodePtr> neighbors(const TreeNodePtr &__node2comp, float __rad) {
			return neighbors_helper(_kd_root, __node2comp, __rad, 0);
		}
		
		std::vector<NodePtr> neighbors_helper(const TreeNodePtr &__node, const TreeNodePtr &__node2comp, float __rad, int __depth) {
			if (__node == nullptr)
				return {};
			float r2 = __rad*__rad;
			float diff = sq_diff(__node, __node2comp);
			float dx = diff_dim(__node, __node2comp, __depth);
			float dx2 = dx*dx;
			
			std::vector<NodePtr> nbs, nbs2, nbs3;
			if (diff <= r2) {
				nbs.push_back(__node);
			}
			int next_dim = (__depth+1) % _k;
			TreeNodePtr first, second;
			if (dx > 0) {
				first = __node->left();
				second = __node->right();
			} else {
				first = __node->right();
				second = __node->left();
			}
			nbs2 = neighbors_helper(first, __node2comp, __rad, next_dim);
			nbs.insert(nbs.end(), nbs2.begin(), nbs2.end());
			if (dx2 < r2) {
				nbs3 = neighbors_helper(second, __node2comp, __rad, next_dim);
				nbs.insert(nbs.end(), nbs3.begin(), nbs3.end());
			}
			return nbs;
		}

		void dfs_report(const TreeNodePtr &__root) {
			if (__root == nullptr)
				return;
			__root->report();
			if (__root->left() != nullptr) 
				dfs_report(__root->left());
			if (__root->right() != nullptr)
				dfs_report(__root->right());
		}
		
		void report() {
			dfs_report(_kd_root);	
		}
};
