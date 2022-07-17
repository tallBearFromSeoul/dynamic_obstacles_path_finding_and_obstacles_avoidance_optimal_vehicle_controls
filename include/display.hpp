#include <unordered_map>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <Eigen/Dense>
#include "kf.hpp"

class Node;
class Obs;
class RRT;
typedef std::shared_ptr<Node> NodePtr;
typedef std::shared_ptr<Obs> ObsPtr;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix4f Mat4f;
typedef Eigen::MatrixXf MatXf;
typedef Eigen::Matrix<float, 4, 2> Mat42f;
class Display_Pangolin {
	private:
 		const std::string wind_name;
 		double w = 1920;
 		double h = 1080;
 		float dt = 0.03333333f;

 		double up_x = 0;
 		double up_y = 1;
 		double up_z = 0;
 		double p0 = 100;

 		double x0 = 5;
 		double y0 = 5;
 		double z0 = 4;
 		pangolin::OpenGlRenderState s_cam;

 		void setup() {
			pangolin::CreateWindowAndBind(wind_name, w, h);
			glEnable(GL_DEPTH_TEST);
			pangolin::GetBoundWindow()->RemoveCurrent();
			
			pangolin::BindToContext(wind_name);
			glEnable(GL_DEPTH_TEST);
			s_cam = pangolin::OpenGlRenderState(
					pangolin::ProjectionMatrix(w, h, p0, p0, w/2, h/2, 0.01, 10000),
					pangolin::ModelViewLookAt(x0,y0,z0,x0,y0,0,up_x,up_y,up_z));
			pangolin::Handler3D handler(s_cam);
			pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, w/h).SetHandler(&handler);
		};
	public:
		Display_Pangolin(const double &_w, const double &_h, const std::string &_s) : w(_w), h(_h), wind_name(_s) {setup();};
		Display_Pangolin(const double &_w, const double &_h, const double &_F, const std::string &_s) : w(_w), h(_h), wind_name(_s), p0(_F) {setup();};
		void draw_tree(RRT *__rrt) {
			std::unordered_map<int, std::vector<int>> graphA = *__rrt->graph();
			std::unordered_map<int, int> gid2nid_mapA = __rrt->gid2nid_map();
			std::unordered_map<int, NodePtr> nid2n_mapA = __rrt->nid2n_map();
			for (int i=0; i<graphA.size(); i++) {
				NodePtr src = nid2n_mapA[gid2nid_mapA[i]];
				for (int e_id : graphA[i]) {
					NodePtr dst = nid2n_mapA[gid2nid_mapA[e_id]];
					glColor3f(1.f,0.f,0.f);
					glPointSize(3.f);
					draw_edge(src, dst);
				}
 			}
 		}

 		template<typename T>
 		void render(bool __success, int __N, T *__rrt, const std::vector<NodePtr> *__path, const std::vector<Vec4f> &__trajectory, const Vec4f &__state, const MatXf &__pred_states, std::vector<ObsPtr> *__obstacles, std::vector<ObsPtr> *__obstacles_in_range, float __scanner_range, KF *__kf) {
 			glEnable(GL_DEPTH_TEST);
 			s_cam = pangolin::OpenGlRenderState(
 					pangolin::ProjectionMatrix(w, h, p0, p0, w/2, h/2, 0.01, 10000),
 					pangolin::ModelViewLookAt(x0,y0,z0,x0,y0,0,up_x,up_y,up_z));
 			pangolin::Handler3D handler(s_cam);
 			pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, w/h).SetHandler(&handler);
 			glClearColor(0.7f,0.7f,0.7f,0.5f);
 			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 			d_cam.Activate(s_cam);
 			draw_vehicle(__state);
 			if (__success)
 				draw_predictions(__pred_states);
 			draw_obstacles(*__obstacles);
 			draw_obstacles_state_estimates(__N, __kf, __obstacles_in_range);
 			draw_scanner_range(__state, __scanner_range);
 			draw_trajectory(__trajectory);
 			draw_path(__path);
 			draw_tree(__rrt);
 			pangolin::FinishFrame();
 		}

 		void render(RRTStar *__rrt, const std::vector<NodePtr> *__path, std::vector<ObsPtr> &_obstacles) {
 			glEnable(GL_DEPTH_TEST);
 			s_cam = pangolin::OpenGlRenderState(
 					pangolin::ProjectionMatrix(w, h, p0, p0, w/2, h/2, 0.01, 10000),
 					pangolin::ModelViewLookAt(x0,y0,z0,x0,y0,0,up_x,up_y,up_z));
 			pangolin::Handler3D handler(s_cam);
 			pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, w/h).SetHandler(&handler);
 			glClearColor(0.9f,0.9f,0.9f,0.5f);
 			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 			d_cam.Activate(s_cam);
 			draw_path(__path);
 			glColor3f(0.3f,0.3f,0.7f);
 			draw_tree(__rrt);
 			glColor3f(0.5f,0.2f,0.8f);
 			draw_obstacles(_obstacles);
 			pangolin::FinishFrame();
 		}

 		void draw_tree(BRRTStar *__rrt) {
 			std::unordered_map<int, std::vector<int>> graphA, graphB;
 			std::unordered_map<int, int> gid2nid_mapA, gid2nid_mapB;
			std::unordered_map<int, NodePtr> nid2n_mapA, nid2n_mapB; 
			graphA = *__rrt->graph();
			graphB = *__rrt->graphB();
			gid2nid_mapA = __rrt->gid2nid_map();
			gid2nid_mapB = __rrt->gid2nid_mapB();
			nid2n_mapA = __rrt->nid2n_map();
			nid2n_mapB = __rrt->nid2n_mapB();
			
			for (int i=0; i<graphA.size(); i++) {
				NodePtr src = nid2n_mapA[gid2nid_mapA[i]];
				for (int e_id : graphA[i]) {
					NodePtr dst = nid2n_mapA[gid2nid_mapA[e_id]];
					glColor3f(1.f,0.2f,0.2f);
					glLineWidth(2.f);
					draw_edge(src, dst);
				}
			}
			
			for (int i=0; i<graphB.size(); i++) {
				NodePtr src = nid2n_mapB[gid2nid_mapB[i]];
				for (int e_id : graphB[i]) {
					NodePtr dst = nid2n_mapB[gid2nid_mapB[e_id]];
					glColor3f(0.2f,1.f,1.f);
					glLineWidth(2.f);
					draw_edge(src, dst);
				}
 			}
 		}

 		void draw_point(const NodePtr &__n) {
 			glVertex3f(__n->val(0), __n->val(1), 0);
 		}

 		void draw_obstacles(std::vector<ObsPtr> &_obstacles) {
 			for (ObsPtr &_obs : _obstacles) {
 				glColor3f(1.0f,0.0f,0.0f);
 				glLineWidth(5.f);
 				draw_circle(_obs->pos(0), _obs->pos(1), 0.f, _obs->rad());
 			}
 		}
 		void draw_scanner_range(const Vec4f &__state, float __scanner_range) {
 			glColor3f(0.8f,0.3f,0.f);
 			glLineWidth(5.f);
 			draw_circle(__state(0),__state(1),0.f,__scanner_range);
 		}
 		void draw_predictions(const MatXf &_pred_states) {
 			for (int j=1; j<_pred_states.cols(); j++) {
 				float x1 = _pred_states.col(j-1)(0);
 				float y1 = _pred_states.col(j-1)(1);
 				float x2 = _pred_states.col(j)(0);
 				float y2 = _pred_states.col(j)(1);
 				glColor3f(0.f,0.7f,0.5f);
 				glLineWidth(6.f);
 				glBegin(GL_LINES);
 				glVertex3f(x1, y1, 0);
 				glVertex3f(x2, y2, 0);
 				glEnd();
 			}
 		}
		void draw_circle(float _x, float _y, float _z, float _r) {
 			glBegin(GL_LINE_LOOP);
 			for (int ii=0; ii < 32; ii++) {
				float theta = 2.0f * 3.1415926f * float(ii) / 32.f;
				float x = _r*cosf(theta);
				float y = _r*sinf(theta);
				glVertex3f(x+_x, y+_y, _z);
			}
			glEnd();
		}
 		void draw_trajectory(const std::vector<Vec4f> &_trajectory) {
 			for (int i=1; i<_trajectory.size(); i++) {
 				glColor3f(0.0f,1.0f,0.0f);
 				glLineWidth(4.f);
 				glBegin(GL_LINES);
 				glVertex3f(_trajectory[i-1](0), _trajectory[i-1](1), 0);
 				glVertex3f(_trajectory[i](0), _trajectory[i](1), 0);
 				glEnd();
 			}

 		}
 		void draw_obstacles_state_estimates(int __N, KF *__kf, std::vector<ObsPtr> *__obstacles_in_range) {
 			for (const ObsPtr &__obs : *__obstacles_in_range) {
 				int obs_id = __obs->id();
 				float rad = __obs->rad();
 				Vec4f state_est = __kf->xm(obs_id);
 				Mat4f cov = __kf->Pm(obs_id);
 				// since x_var == y_var
 				float var = cov(0,0);
 				float sd = sqrt(var);
 				glColor3f(0.f,0.f,1.f);
 				glLineWidth(3.f);
 				draw_circle(state_est(0), state_est(1), 0.f, rad);
 				glLineWidth(3.f);
 				glColor3f(0.5f,0.3f,0.3f);
 				draw_circle(state_est(0), state_est(1), 0.f, rad+sd);
 				/*
 				glColor3f(1.f,0.f,0.f);
 				glLineWidth(5.f);
 				glBegin(GL_LINES);
 				glVertex3f(state_est(0), state_est(1), 0.f);
 				glVertex3f(state_est(0)+state_est(2),state_est(1)+state_est(3), 0.f);
 				glEnd();
 				float x = state_est(0);
 				float y = state_est(1);
 				float vx = state_est(2);
 				float vy = state_est(3);
 				for (int i=0; i<__N; i++) {
 					glColor3f(1.0f,0.3f,0.3f);
 					glLineWidth(3.f);
 					x += vx*dt;
 					y += vy*dt;
 					draw_circle(x,y,0.f,rad);
 				}
 				*/
 			}
 		}

 		void draw_vehicle(const Vec4f &__state) {
 			float cx = __state(0);
 			float cy = __state(1);
			float c1x = __state(0)-0.25f;
			float c1y = __state(1)-0.25f;
 			float c2x = __state(0)+0.25f;
 			float c2y = __state(1)+0.25f;
 			glColor3f(0.f,0.7f,0.5f);
 			glLineWidth(5.f);
 			glBegin(GL_LINES);
 			glVertex3f(c1x, c1y, 0);
 			glVertex3f(c1x, c2y, 0);
			glVertex3f(c1x, c2y, 0);
			glVertex3f(c2x, c2y, 0);
			glVertex3f(c2x, c2y, 0);
			glVertex3f(c2x, c1y, 0);
			glVertex3f(c2x, c1y, 0);
			glVertex3f(c1x, c1y, 0);
			glEnd();
		};
 		void draw_path(const std::vector<NodePtr> *__path) {
 			for (int i=0; i<__path->size()-1; i++) {
 				glColor3f(1.f,1.f,0.f);
 				glLineWidth(2.f);
 				draw_edge(__path->at(i), __path->at(i+1));
 			}
 		}
		void draw_edge(const NodePtr &__src, const NodePtr &__dst) {
			glBegin(GL_LINES);
			draw_point(__src);
			draw_point(__dst);
			glEnd();	
		}
};
