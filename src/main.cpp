#include <iostream>
#include "rrt.hpp"
#include "optimizer.hpp"
#include "display.hpp"

int Node::MAXID = 0;
int Obs::MAXID = 0;

int main(int argc, char* argv[]) {
	int N = 50;
	int n_obs = 25;
	int RRT_ITER_MAX = 1500;
	float dt = 0.03333333f;
	//float dt = 0.01666666f;//0.05f;
	float lr = 1.738f;
	float v_i = 0.f;
	float head_i = deg_to_rad(45);	
	float scanner_range = 20.f;
	float obs_max_bounds[6] {1.5f,25.5f,1.5f,25.5f,0.6f,1.2f};

	RowVec2f v_init {0.f, 0.f};
	RowVec2f v_dest {25.f, 25.f};
	Vec4f state_init {0.f, 0.f, v_i, head_i};
	Vec4f state = state_init;
	Vec2f pi = state.block(0,0,2,1);
	Vec2f pf = v_dest.transpose();
	Vec2f u_opt;

	Display_Pangolin *dp = new Display_Pangolin(1920,1080,"vehicle motion planning in dynamic environment");

	World *world = new World();
	KF *kf_re = new KF("kalman_recursive");
	KF *kf_ga = new KF("kalman_gain");
	KF *kf_jo = new KF("kalman_joseph");
	Li_Radar *scanner = new Li_Radar(scanner_range);
	ObjetFactory *factory = new ObjetFactory();
	factory->createNObstacles(n_obs, obs_max_bounds, world->obstacles());

	BRRTStar *rrt = new BRRTStar(v_init, v_dest, world->obstacles(), RRT_ITER_MAX);
	for (int i=0; i<10; i++) {
		if (rrt->build_status()==RRT::Status::REACHED)
			break;
		delete rrt;
		std::cout<<"reinstantiating rrtstar\n";
		rrt = new BRRTStar(v_init, v_dest, world->obstacles(), RRT_ITER_MAX);
	}
	std::vector<NodePtr> path_ = *rrt->path();

	Optimizer *opt = new Optimizer(N, dt, lr);
	std::vector<VehiclePtr> vehicles;
	factory->createVehicle(Objet::Shape::CIRCLE, dt, lr, state_init, vehicles);
	VehiclePtr vehicle = vehicles[0];
	std::vector<Vec4f> trajectory {state_init};

	int prev = 0;
	std::unordered_map<int, Vec4f> *xm, *xm2, *xm3;
	std::vector<ObsPtr> obstacles_in_range;
	while (!pangolin::ShouldQuit()) {
		obstacles_in_range.clear();
		if (!rrt->collision_free(pi))
			break;
		if (path_.size() < N+1) {
			delete opt;
			prev = path_.size()-1;
			opt = new Optimizer(path_.size()-1, dt, lr);
		} else if (prev != N && path_.size() >= N+1) {
			delete opt;
			prev = N;
			opt = new Optimizer(N, dt, lr);
		}
		scanner->scan(pi, world->obstacles(), obstacles_in_range);
		xm = kf_re->update_recursive(obstacles_in_range, scanner);
		xm2 = kf_ga->update_kalman_gain(obstacles_in_range, scanner, false);
		xm3 = kf_jo->update_kalman_gain(obstacles_in_range, scanner, true);
		bool success = opt->optimize(state, &path_, obstacles_in_range, kf_jo);
		u_opt = opt->input_opt();
		if (!success) {
			//std::cout<<"not success.. input opt is : "<<u_opt<<"\n";
			//u_opt = Vec2f(0.f,-1.5f);
		}
		world->update();
		state = vehicle->update(u_opt);	
		pi = state.block(0,0,2,1);
		trajectory.push_back(state);
		// BRRT Star Testing
		path_ = rrt->update(pi);
		// RRT Star Testing 
		//rrt->update(pi);
		//path_ = *rrt->path();
		dp->render(success, N, rrt, &path_, trajectory, state, opt->pred_states(), world->obstacles(), &obstacles_in_range, scanner_range, kf_jo);
	}
	kf_re->kf_output.close();
	kf_ga->kf_output.close();
	kf_jo->kf_output.close();
	while (!pangolin::ShouldQuit()) {
		dp->render(false, N, rrt, &path_, trajectory, state, opt->pred_states(), world->obstacles(), &obstacles_in_range, scanner_range, kf_jo);
	}
	return 0;
}
