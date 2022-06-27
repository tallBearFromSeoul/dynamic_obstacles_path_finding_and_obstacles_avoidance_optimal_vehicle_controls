#include "rrt.hpp"
#include "optimizer.hpp"
#include "display.hpp"

int Node::MAXID = 0;

int main(int argc, char* argv[]) {
	//get_started();
	int N = 30;
	float ts = 0.03333333f;
	//float ts = 0.01666666f;//0.05f;
	float lr = 1.738f;
	float v_i = 0.f;
	float head_i = deg_to_rad(45);
	Vec4f state_init {0.f, 0.f, v_i, head_i};
	RowVec2f v_init {0.f, 0.f};
	RowVec2f v_dest {15.f, 15.f};
	RowVec2f op0 = {5,5};
	RowVec2f op1 = {4,10};
	RowVec2f op2 = {9,9};
	ObsPtr obs0 = std::make_shared<Obs>(Objet::Shape::CIRCLE, Objet::Type::STATIC, op0, 2);
	ObsPtr obs1 = std::make_shared<Obs>(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op1, 1);
	ObsPtr obs2 = std::make_shared<Obs>(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op2, 1);
	std::vector<ObsPtr> obstacles {obs0, obs1, obs2};
	int n_obs = obstacles.size();

	/*
	RRTStar *rrt = new RRTStar(v_init, v_dest, &obstacles, 3000);
	*/
	RRT *rrt = new RRT(v_init, v_dest, &obstacles, 3000);
	while (rrt->build_status()!=RRT::Status::REACHED) {
		delete rrt;
		rrt = new RRT(v_init, v_dest, &obstacles, 3500);
	}
	
	std::vector<NodePtr> path_ = rrt->path();
	Optimizer *opt = new Optimizer(N, ts, lr);
	Vehicle *vehicle = new Vehicle(ts, lr, lr, state_init);

	std::vector<Vec4f> trajectory {state_init};
	//std::vector<Mat42f> ref;
	//std::vector<MatXf> paths;
	
	Display_Pangolin *dp = new Display_Pangolin(1024, 1024, "display");
	
	Vec4f state = state_init;
	Vec2f pi = state.block(0,0,2,1);
	Vec2f pf {15, 15};
	//for (int i=0; i<30; i++) {
	while ((pf-pi).norm() > 0.25f) {
		path_ =  rrt->path();
		if (path_.size() < N+1) {
			N = path_.size()-1;
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
		delete rrt;
		rrt = new RRT(pi, v_dest, &obstacles, 3000, state(2), 40.f);
		float i=0;
		while (rrt->build_status()!=RRT::Status::REACHED) {
			delete rrt;
			rrt = new RRT(pi, v_dest, &obstacles, 3500, state(2), 40.f+(10.f*i));
			//rrt->set_speed_lim(0.5f-(0.05f)*i);
			i++;
		}
		dp->render(rrt, path_, trajectory, state, opt->pred_states(),obstacles);
	}
	while(true) {
		dp->render(rrt, path_, trajectory, state, opt->pred_states(), obstacles);
	}
	/*
	Vehicle *vehicle = new Vehicle(ts, lr, lr, state_init);
	std::cout<<"Initial state is :\n";
	vehicle->report();
	std::cout<<"zref is :\n"<<zref<<"\n";;

	//std::vector<Obs> obstacles;
	std::cout<<"# of obstacles : "<<n_obs<<"\n";
	for (int i=0; i<n_obs; i++) {
		std::cout<<"obstacles["<<i<<"] [x, y, rad] : ["<<obstacles[i].pos(0)<<" , "<<obstacles[i].pos(1)<<" , "<<obstacles[i].rad<<"]\n";
	}
	NOptimizer *nopt = new NOptimizer(vehicle, obstacles, N, ts, lr, state_init, zref, false);
	int max_step = 2;
	Vehicle agent(ts, lr, lr, state_init);
	for (int i=0; i<max_step; i++) {
		
		if (nopt->optimize_N()) {
			break;
		}
		Vec4f new_state = agent.update(nopt->input_hist[0]);
		nopt->set_init(new_state);
	}
	nopt->print();
	*/
	return 0;
}

