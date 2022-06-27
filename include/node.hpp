#ifndef NODE_HPP
#define NODE_HPP
#include <iostream>
#include <vector>
#include <Eigen/Dense>

typedef Eigen::Matrix<float, 1, Eigen::Dynamic, Eigen::RowMajor> RowVecXf;

class Node {
	protected:
		typedef std::shared_ptr<Node> NodePtr;
		//string key;
		RowVecXf _val;
		int _id;
		int _dims;
		int _n_nodes=0;
		int _n_edges=0;
		static int MAXID;
	public:
		//vector<NodePtr> chd;
		RowVecXf val() {return _val;};
		float val(int __idx) {return _val(__idx);};
		int id() {return _id;};
		int dims() {return _dims;};
		
		Node() {};
		Node(const NodePtr &__node) : _val(__node->val()) {_dims=_val.cols();};
		Node(const RowVecXf &__val) : _val(__val) {_id=MAXID++;_dims=_val.cols();};
		//Node(string _key, RowVecXf _val) : key(_kay), val(_val) {id=MAXID++;};

		virtual void report() {
			return;
			//std::cout<<"[Node id, val] : ["<<id()<<" , "<<val()<<"]\n";			
		}
};


class TreeNode : public Node {
	typedef std::shared_ptr<TreeNode> TreeNodePtr;
	private:
		TreeNodePtr _left = nullptr;
		TreeNodePtr _right = nullptr;
		int _depth;
	public:
		TreeNodePtr left() {return _left;};
		TreeNodePtr right() {return _right;};
		void set_left(const TreeNodePtr &__n) {_left=__n;};
		void set_right(const TreeNodePtr &__n) {_right=__n;};
		int depth() {return _depth;};

		TreeNode() {};
		TreeNode(const NodePtr &__node, int __depth) : Node(*__node), _depth(__depth) {};//std::cout<<"TreeNode created...\n\t[id, val, depth] : ["<<__node->id()<<" , "<<__node->val()<<" , "<<_depth<<"]\n";};
		TreeNode(const RowVecXf &__val, int __depth) : Node(__val), _depth(__depth) {};
		void report() {
			return;
			//std::cout<<"[Node id, val, depth] : ["<<id()<<" , "<<val()<<" , "<<depth()<<"]\n";			
		}
};

#endif
typedef std::shared_ptr<Node> NodePtr;
typedef std::shared_ptr<TreeNode> TreeNodePtr;
