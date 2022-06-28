#include <unordered_map>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/gldraw.h>
#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

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
		
		double up_x = 0;
		double up_y = 1;
		double up_z = 0;
		double p0 = 100;

		double z0 = 5;
		pangolin::OpenGlRenderState s_cam;
	
		void setup() {
			pangolin::CreateWindowAndBind(wind_name, w, h);
			glEnable(GL_DEPTH_TEST);
			pangolin::GetBoundWindow()->RemoveCurrent();
			
			pangolin::BindToContext(wind_name);
			glEnable(GL_DEPTH_TEST);
			s_cam = pangolin::OpenGlRenderState(
					pangolin::ProjectionMatrix(w, h, p0, p0, w/2, h/2, 0.01, 10000),
					pangolin::ModelViewLookAt(0,0,z0,0,0,0,up_x,up_y,up_z));
			pangolin::Handler3D handler(s_cam);
			pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, w/h).SetHandler(&handler);
		};

	public:
		Display_Pangolin(const double &_w, const double &_h, const std::string &_s) : w(_w), h(_h), wind_name(_s) {setup();};
		Display_Pangolin(const double &_w, const double &_h, const double &_F, const std::string &_s) : w(_w), h(_h), wind_name(_s), p0(_F) {setup();};

		void draw_tree(RRT *__rrt) {
			std::unordered_map<int, std::vector<int>> adj_list_ = *__rrt->graph();
			std::unordered_map<int, int> gid2nid_map_ = __rrt->gid2nid_map();
			std::unordered_map<int, NodePtr> nid2n_map_ = __rrt->nid2n_map();
			for (int i=0; i<adj_list_.size(); i++) {
				NodePtr src = nid2n_map_[gid2nid_map_[i]];
				for (int e_id : adj_list_[i]) {
					NodePtr dst = nid2n_map_[gid2nid_map_[e_id]];
					glColor3f(1.f,0.f,0.f);
					glPointSize(3.f);
					draw_edge(src, dst);
				}
			}
		}		
		void draw_point(const NodePtr &__n) {
			glVertex3f(__n->val(0), __n->val(1), 0);
		}
		void draw_obstacles(std::vector<ObsPtr> &_obstacles) {
			for (ObsPtr &_obs : _obstacles) {
				glColor3f(1.0f,0.2f,1.0f);
				draw_circle(_obs->pos(0), _obs->pos(1), 0.f, _obs->rad());
			}
		}
		void draw_predictions(const MatXf &_pred_states) {
			for (int j=1; j<_pred_states.cols(); j++) {
				float x1 = _pred_states.col(j-1)(0);
				float y1 = _pred_states.col(j-1)(1);
				float x2 = _pred_states.col(j)(0);
				float y2 = _pred_states.col(j)(1);
				glColor3f(1.f,0.3f,0.1f);
				glBegin(GL_LINES);
				glVertex3f(x1, y1, 0);
				glVertex3f(x2, y2, 0);
				glEnd();
			}
		}
		void render(RRT *__rrt, const std::vector<NodePtr> *__path, std::vector<ObsPtr> &_obstacles) {
			glEnable(GL_DEPTH_TEST);
			s_cam = pangolin::OpenGlRenderState(
					pangolin::ProjectionMatrix(w, h, p0, p0, w/2, h/2, 0.01, 10000),
					pangolin::ModelViewLookAt(0,0,z0,0,0,0,up_x,up_y,up_z));
			pangolin::Handler3D handler(s_cam);
			pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, w/h).SetHandler(&handler);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			d_cam.Activate(s_cam);
			draw_path(__path);
			glColor3f(0.3f,0.3f,0.7f);
			//draw_tree(__rrt);
			glColor3f(0.5f,0.2f,0.8f);
			draw_obstacles(_obstacles);
			pangolin::FinishFrame();
		}

		void render(RRT *__rrt, const std::vector<NodePtr> *__path, const std::vector<Vec4f> &_trajectory, const Vec4f &_state, const MatXf &_pred_states, std::vector<ObsPtr> &_obstacles) {
			glEnable(GL_DEPTH_TEST);
			s_cam = pangolin::OpenGlRenderState(
					pangolin::ProjectionMatrix(w, h, p0, p0, w/2, h/2, 0.01, 10000),
					pangolin::ModelViewLookAt(0,0,z0,0,0,0,up_x,up_y,up_z));
			pangolin::Handler3D handler(s_cam);
			pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, w/h).SetHandler(&handler);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			d_cam.Activate(s_cam);
			draw_path(__path);
			draw_tree(__rrt);
			draw_obstacles(_obstacles);
			draw_vehicle(_state);
			draw_predictions(_pred_states);
			draw_trajectory(_trajectory);
			glColor3f(0,0.5,0.5);
			pangolin::FinishFrame();
		}

		void render_3d(const Vec4f &__state) {
			//glEnable(GL_LIGHTING);
			//glEnable(GL_COLOR_MATERIAL);
			//glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glEnable(GL_DEPTH_TEST);
			s_cam = pangolin::OpenGlRenderState(
					pangolin::ProjectionMatrix(w, h, p0, p0, w/2, h/2, 0.01, 1000),
					pangolin::ModelViewLookAt(-0.5,0,1.0,0,0,1.0,0,0,1));
			pangolin::Handler3D handler(s_cam);
			pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, w/h).SetHandler(&handler);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			d_cam.Activate(s_cam);
			glColor3f(1.0f,0.f,0.f);
			draw_vehicle_3d(__state);
			pangolin::FinishFrame();

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
				glPointSize(3.f);
				glBegin(GL_LINES);
				glVertex3f(_trajectory[i-1](0), _trajectory[i-1](1), 0);
				glVertex3f(_trajectory[i](0), _trajectory[i](1), 0);
				glEnd();
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

		void draw_vehicle_3d(const Vec4f &__state) {
			float cx = __state(0);
			float cy = __state(1);
			float c1x = __state(0)-0.7f;
			float c1y = __state(1)-0.8f;
			float c2x = __state(0)+0.7f;
			float c2y = __state(1)+0.8f;
			glPointSize(15.f);
			glBegin(GL_LINES);
			glVertex3f(c1x, c1y, 0);
			glVertex3f(c1x, c2y, 0);
			glVertex3f(c1x, c2y, 0);
			glVertex3f(c2x, c2y, 0);
			glVertex3f(c2x, c2y, 0);
			glVertex3f(c2x, c1y, 0);
			glVertex3f(c2x, c1y, 0);
			glVertex3f(c1x, c1y, 0);
			
			glVertex3f(c1x, c1y, 1.9f);
			glVertex3f(c1x, c2y, 1.9f);
			glVertex3f(c1x, c2y, 1.9f);
			glVertex3f(c2x, c2y, 1.9f);
			glVertex3f(c2x, c2y, 1.9f);
			glVertex3f(c2x, c1y, 1.9f);
			glVertex3f(c2x, c1y, 1.9f);
			glVertex3f(c1x, c1y, 1.9f);

			glVertex3f(c1x, c1y, 0);
			glVertex3f(c1x, c1y, 1.9f);
			glVertex3f(c1x, c2y, 0);
			glVertex3f(c1x, c2y, 1.9f);
			glVertex3f(c2x, c2y, 0);
			glVertex3f(c2x, c2y, 1.9f);
			glVertex3f(c2x, c1y, 0);
			glVertex3f(c2x, c1y, 1.9f);
			glEnd();
		}

		void draw_path(const std::vector<NodePtr> *__path) {
			for (int i=0; i<__path->size()-1; i++) {
				glColor3f(0.3f,1.0f,0.f);
				glPointSize(15.f);
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

