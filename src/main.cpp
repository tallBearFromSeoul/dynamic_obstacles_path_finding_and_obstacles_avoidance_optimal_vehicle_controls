#include "rrt.hpp"
#include "optimizer.hpp"
#include "model.hpp"
#include "camera.hpp"
#include "vertices.hpp"
#include "display_utility.hpp"

int Node::MAXID = 0;

int main(int argc, char* argv[]) {
	int N = 30;
	int RRT_ITER_MAX = 1000;
	float ts = 0.03333333f;
	//float ts = 0.01666666f;//0.05f;
	float lr = 1.738f;
	float v_i = 0.f;
	float head_i = deg_to_rad(45);
	Vec4f state_init {0.f, 0.f, v_i, head_i};
	RowVec2f v_init {0.f, 0.f};
	RowVec2f v_dest {15.f, 15.f};
	RowVec2f op0 = {5,5};
	RowVec2f op1 = {10,3};
	RowVec2f op2 = {8,11};
	RowVec2f op3 = {3,10};
	std::vector<ObsPtr> obstacles;
	ObjetFactory *factory = new ObjetFactory();
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::STATIC, op0, 3.f, obstacles, Objet::Behaviour::NONE);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op1, 1.f, obstacles, Objet::Behaviour::VERT);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op2, 1.f, obstacles, Objet::Behaviour::HORZ);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op3, 1.f, obstacles, Objet::Behaviour::DIAG);

	int n_obs = obstacles.size();

	// Displaying 3D TESTING :
	/*
	while (true)
		dp->render_3d(state_init);
	*/
	// RRT TESTING : 
	/*
	//RRT *rrt = new RRT(v_init, v_dest, &obstacles, RRT_ITER_MAX);
	RRTStar *rrt = new RRTStar(v_init, v_dest, &obstacles, RRT_ITER_MAX);
	RRT::Status status = rrt->build_status();
	if (status != RRT::Status::REACHED) {
		std::cout<<"didn't reach ...\n";
	}
	std::vector<NodePtr> path_ = rrt->path();
	
	std::cout<<"path size : "<<path_.size()<<"\n";
	while(true) {
		dp->render(rrt, path_, obstacles);
	}
	*/
	
	RRTStar *rrt = new RRTStar(v_init, v_dest, &obstacles, RRT_ITER_MAX);
	while (rrt->build_status()!=RRT::Status::REACHED) {
		delete rrt;
		rrt = new RRTStar(v_init, v_dest, &obstacles, RRT_ITER_MAX);
	}
	std::vector<NodePtr> *path_ = rrt->path();
	Optimizer *opt = new Optimizer(N, ts, lr);
	std::vector<VehiclePtr> vehicles;
	factory->createVehicle(Objet::Shape::CIRCLE, ts, lr, state_init, vehicles);
	VehiclePtr vehicle = vehicles[0];
	//Vehicle *vehicle = new Vehicle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, ts, lr, lr, state_init);
	std::vector<Vec4f> trajectory {state_init};
	
	
	//Display_Pangolin *dp = new Display_Pangolin(1024, 1024, "display");
	//dp->render(rrt, path_, trajectory, state_init, opt->pred_states(),obstacles);

	Vec4f state = state_init;
	Vec2f pi = state.block(0,0,2,1);
	Vec2f pf {15, 15};
	while ((pf-pi).norm() > 0.5f) {
		//path_ =  rrt->path();
		if (path_->size() < N+1) {
			delete opt;
			N = path_->size()-1;
			opt = new Optimizer(N, ts, lr);
		}
		opt->optimize(state, path_, obstacles);
		Vec2f u_opt = opt->input_opt();
		for (ObsPtr &__obs : obstacles) {
			__obs->update();
		}
		state = vehicle->update(u_opt);	
		pi = state.block(0,0,2,1);
		trajectory.push_back(state);
		rrt->update(pi);
		path_ = rrt->path();
		/*
		RRT::Status update_status = rrt->update(pi);
		if (update_status == RRT::Status::ADVANCED) {
			path_ = rrt->path();
		} else {
			std::cerr<<"ERROR NO PATH\n";
			break;
		}
		*/
		/*
		delete rrt;
		rrt = new RRTStar(pi, v_dest, &obstacles, RRT_ITER_MAX);
		//rrt = new RRTStar(pi, v_dest, &obstacles, RRT_ITER_MAX, state(2), 40.f);
		float i=0;
		while (rrt->build_status()!=RRT::Status::REACHED) {
			delete rrt;
			rrt = new RRTStar(pi, v_dest, &obstacles, RRT_ITER_MAX);
			//rrt = new RRTStar(pi, v_dest, &obstacles, RRT_ITER_MAX, state(2), 40.f+(10.f*i));
			i++;
		}
		*/
		//dp->render(rrt, path_, trajectory, state, opt->pred_states(),obstacles);
	}
	while(true) {
		//dp->render(rrt, path_, trajectory, state, opt->pred_states(), obstacles);
	}
	
	return 0;
}

