#include "rrt.hpp"
#include "optimizer.hpp"
#include "model.hpp"
#include "camera.hpp"
#include "vertices.hpp"
#include "display_utility.hpp"

int Node::MAXID = 0;
void draw_edge(const NodePtr &__src, const NodePtr &__dst, Shader &shader, unsigned int horzBoxVAO, unsigned int horzBoxVBO) {
	float x0, y0, x1, y1;
	x0 = __src->val(0);
	y0 = __src->val(1);
	x1 = __dst->val(0);
	y1 = __dst->val(1);
	shader.use();
	glm::mat4 model = glm::mat4(1.0f);
	model = glm::translate(model, glm::vec3(x0,y0,1.5));
	//model = glm::rotate(model, head, glm::vec3(0,,1));
	model = glm::scale(model, glm::vec3((1.f+x1-x0),(1.f+y1-y1), 2.f));
	shader.setMat4("model", model);
	glBindVertexArray(horzBoxVAO);
	glDrawArrays(GL_TRIANGLES,0,18);
	//model = glm::rotate(model, glm::radians(90.f), glm::vec3(0,0,1));
	//shader.setMat4("model",model);
	glDepthMask( GL_TRUE ); 
	glDisable( GL_BLEND );
	glBindVertexArray(0);

}

void draw_tree(RRT *__rrt, Shader &shader, unsigned int horzBoxVAO, unsigned int horzBoxVBO) {
	std::unordered_map<int, std::vector<int>> adj_list_ = *__rrt->graph();
	std::unordered_map<int, int> gid2nid_map_ = __rrt->gid2nid_map();
	std::unordered_map<int, NodePtr> nid2n_map_ = __rrt->nid2n_map();
	for (int i=0; i<adj_list_.size(); i++) {
		NodePtr src = nid2n_map_[gid2nid_map_[i]];
		for (int e_id : adj_list_[i]) {
			NodePtr dst = nid2n_map_[gid2nid_map_[e_id]];
			//glColor3f(1.f,0.f,0.f);
			//glPointSize(3.f);
			draw_edge(src, dst, shader, horzBoxVAO, horzBoxVBO);
		}
	}
}

inline glm::mat3 rot_yaw(float __yaw) {
	float mat_val[9] = {1,0,0,
											0,cos(__yaw),-sin(__yaw),
											0,sin(__yaw),cos(__yaw)};
	glm::mat3 res;
	memcpy(glm::value_ptr(res), mat_val, sizeof(mat_val));
	return res;
}

inline glm::mat4 poseMat4f(const Vec4f &__state) {
	float __x = __state(0);
	float __y = __state(1);
	float __yaw = __state(3);
	float mat_val[16] = {1,0,0,__x,
											0,cos(__yaw),-sin(__yaw),__y,
											0,sin(__yaw),cos(__yaw),0,
											0,0,0,1};
}

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
	RowVec2f op0 = {5,-4};
	RowVec2f op1 = {10,3};
	RowVec2f op2 = {8,11};
	RowVec2f op3 = {3,10};
	RowVec2f op4 = {15,20};

	cubesphere.printSelf();
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	GLFWwindow* window = glfwCreateWindow(W, H, "LearnOpenGL", nullptr, nullptr);
	if (window == nullptr) {
		std::cout<<"Failed to create GLFW window"<<"\n";
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout<<"Failed to initialize GLAD\n";
		return -1;
	}
	stbi_set_flip_vertically_on_load(true);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

	unsigned int floorTexture = loadTexture("../data/textures/road.jpg");
	unsigned int cubeTexture = loadTexture("../data/textures/brick.png");
	unsigned int metalTexture = loadTexture("../data/textures/metal.png");

	// sphere VAO
	unsigned int sphereVAO, sphereVBO,sphereEBO;
	glGenVertexArrays(1, &sphereVAO);
	glGenBuffers(1, &sphereVBO);
	glGenBuffers(1, &sphereEBO);
	glBindVertexArray(sphereVAO);
	glBindBuffer(GL_ARRAY_BUFFER, sphereVBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphereEBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	int stride = 8*sizeof(float);//cubesphere.getInterleavedStride();
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
	glVertexAttribPointer(1, 3, GL_FLOAT, false, stride, (void*)(sizeof(float)*3));
	glVertexAttribPointer(2, 2, GL_FLOAT, false, stride, (void*)(sizeof(float)*6));
	glBufferData(GL_ARRAY_BUFFER, cubesphere.getInterleavedVertexSize(), cubesphere.getInterleavedVertices(), GL_STATIC_DRAW);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, cubesphere.getIndexSize(), cubesphere.getIndices(), GL_STATIC_DRAW);
	glBindVertexArray(0);

	// horzBox VAO
	unsigned int horzBoxVAO, horzBoxVBO;
	glGenVertexArrays(1, &horzBoxVAO);
	glGenBuffers(1, &horzBoxVBO);
	glBindVertexArray(horzBoxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, horzBoxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(horzBoxVertices), &horzBoxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glBindVertexArray(0);

	// vertBox VAO
	unsigned int vertBoxVAO, vertBoxVBO;
	glGenVertexArrays(1, &vertBoxVAO);
	glGenBuffers(1, &vertBoxVBO);
	glBindVertexArray(vertBoxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, vertBoxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertBoxVertices), &vertBoxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glBindVertexArray(0);

	// plane VAO
	unsigned int planeVAO, planeVBO;
	glGenVertexArrays(1, &planeVAO);
	glGenBuffers(1, &planeVBO);
	glBindVertexArray(planeVAO);
	glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), &planeVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3*sizeof(float)));
	glBindVertexArray(0);
	
	// cube VAO
	unsigned int cubeVAO, cubeVBO;
	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &cubeVBO);
	glBindVertexArray(cubeVAO);
	glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), &cubeVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	glBindVertexArray(0);

	Shader shader_mesh("../config/model_loading.vs","../config/model_loading.fs");
	Shader shader_single_color("../config/single_color.vs", "../config/single_color.fs");
	Shader shader_multi_color("../config/multi_color.vs", "../config/multi_color.fs");
	
	Shader shader_pred("../config/pred.vs","../config/pred.fs");
	Shader shader_obs_multi_color("../config/obs_multi_color.vs","../config/obs_multi_color.fs");
	Shader shader_trajectory("../config/traj.vs","../config/traj.fs");
	Shader shader_path("../config/path.vs","../config/path.fs");
	Shader shader_tree("../config/tree.vs","../config/tree.fs");
	//GLModel gl_model("../Material/bus_setia_negara_texturizer.blend");
	GLModel gl_model("../14-lowpolyfiatuno/LowPolyFiatUNO.obj");

	std::vector<ObsPtr> obstacles;
	ObjetFactory *factory = new ObjetFactory();
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op0, 1.f, obstacles, Objet::Behaviour::VERT);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op1, 1.f, obstacles, Objet::Behaviour::VERT);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op2, 1.f, obstacles, Objet::Behaviour::HORZ);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op3, 1.f, obstacles, Objet::Behaviour::DIAG);
	factory->createObstacle(Objet::Shape::CIRCLE, Objet::Type::DYNAMIC, op4, 1.f, obstacles, Objet::Behaviour::NEGDIAG);

	int n_obs = obstacles.size();

	RRTStar *rrt = new RRTStar(v_init, v_dest, &obstacles, RRT_ITER_MAX);
	while (rrt->build_status()!=RRT::Status::REACHED) {
		delete rrt;
		std::cout<<"reinstantiating rrtstar\n";
		rrt = new RRTStar(v_init, v_dest, &obstacles, RRT_ITER_MAX);
	}
	std::vector<NodePtr> *path_ = rrt->path();
	Optimizer *opt = new Optimizer(N, ts, lr);
	std::vector<VehiclePtr> vehicles;
	factory->createVehicle(Objet::Shape::CIRCLE, ts, lr, state_init, vehicles);
	VehiclePtr vehicle = vehicles[0];
	std::vector<Vec4f> trajectory {state_init};
	
	Vec4f state = state_init;
	Vec2f pi = state.block(0,0,2,1);
	Vec2f pf {45.f, 45.f};
	int prev = 0;
	while (!glfwWindowShouldClose(window) && (pf-pi).norm() > 0.5f) {
		float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		
		processInput(window);
		if (path_->size() < N+1) {
			delete opt;
			prev = path_->size()-1;
			opt = new Optimizer(path_->size()-1, ts, lr);
		} else if (prev != N && path_->size() >= N+1) {
			delete opt;
			prev = N;
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
		std::cout<<"before rrt update()\n";
		RRT::Status status_update = rrt->update(pi);
		std::cout<<"path size : "<<path_->size()<<"\n";
		std::cout<<"after rrt update()\n";
		path_ = rrt->path();
		std::cout<<"status update : "<<status_update<<"\n";

		processInput(window);
		glClearColor(0.7f, 0.7f, 0.7f, 0.5f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
		glm::mat4 model, view, projection;
		glm::vec4 lightPosition, lightAmbient, lightDiffuse, lightSpecular, materialAmbient, materialDiffuse, materialSpecular;
		model = glm::mat4(1.0f);
		if (fpv) {
			view = glm::lookAt(glm::vec3(state(0)-3,state(1)-3,5),glm::vec3(state(0),state(1),2.5),glm::vec3(0,0,1));
						 //glm::inverse(poseMat4f(state));//camera.GetViewMatrix();
		} else {
			view = camera.GetViewMatrix();
		}
		projection = glm::perspective(glm::radians(45.f), float(W)/float(H), 0.05f, 90.0f);	

		shader_tree.use();
		shader_tree.setMat4("model",model);
		shader_tree.setMat4("view",view);
		shader_tree.setMat4("projection",projection);

		shader_trajectory.use();
		shader_trajectory.setMat4("model",model);
		shader_trajectory.setMat4("view",view);
		shader_trajectory.setMat4("projection",projection);

		shader_path.use();
		shader_path.setMat4("model",model);
		shader_path.setMat4("view",view);
		shader_path.setMat4("projection",projection);

		shader_pred.use();
		shader_pred.setMat4("model",model);
		shader_pred.setMat4("view",view);
		shader_pred.setMat4("projection",projection);
		
		shader_obs_multi_color.use();
		shader_obs_multi_color.setMat4("model",model);
		shader_obs_multi_color.setMat4("view",view);
		shader_obs_multi_color.setMat4("projection",projection);
		
		lightPosition = glm::vec4(0, 0, 1, 0);
    lightAmbient = glm::vec4(0.3f, 0.3f, 0.3f, 1);
    lightDiffuse = glm::vec4(0.7f, 0.7f, 0.7f, 1);
    lightSpecular = glm::vec4(1.0f, 1.0f, 1.0f, 1);
    materialAmbient = glm::vec4(0.5f, 0.5f, 0.5f, 1);
    materialDiffuse = glm::vec4(0.7f, 0.7f, 0.7f, 1);
    materialSpecular = glm::vec4(0.4f, 0.4f, 0.4f, 1);
    float materialShininess  = 16;	

		shader_obs_multi_color.setVec4("lightPosition",lightPosition);
		shader_obs_multi_color.setVec4("lightAmbient",lightAmbient);
		shader_obs_multi_color.setVec4("lightDiffuse",lightDiffuse);
		shader_obs_multi_color.setVec4("lightSpecular",lightSpecular);
		shader_obs_multi_color.setVec4("materialAmbient",materialAmbient);
		shader_obs_multi_color.setVec4("materialDiffuse",materialDiffuse);
		shader_obs_multi_color.setVec4("materialSpecular",materialSpecular);
		shader_obs_multi_color.setFloat("materialShininess",materialShininess);
		//shader_obs_multi_color.setVec4("map0",glm::Vec4(0,0,0));
		shader_obs_multi_color.setBool("textureUsed",true);

		shader_single_color.use();
		shader_single_color.setMat4("model",model);
		shader_single_color.setMat4("view",view);
		shader_single_color.setMat4("projection",projection);

		shader_multi_color.use();
		shader_multi_color.setMat4("model",model);
		shader_multi_color.setMat4("view", view);
		shader_multi_color.setMat4("projection", projection);

		glStencilMask(0x00);
		// floor
		glBindVertexArray(planeVAO);
		glBindTexture(GL_TEXTURE_2D, floorTexture);
		glDrawArrays(GL_TRIANGLES, 0, 48);
		glBindVertexArray(0);
		glBindTexture(GL_TEXTURE_2D, 0);
		
		model = glm::mat4(1.0f);
		model = glm::translate(model ,glm::vec3(state(0),state(1),0.f));
		model = glm::rotate(model, glm::radians(90.f),glm::vec3(1.f,0.f,0.f));
		model = glm::rotate(model, glm::radians(90.f),glm::vec3(0.f,1.f,0.f));
		model = glm::rotate(model, state(3),glm::vec3(0.f,1.f,0.f));
		model = glm::scale(model, glm::vec3(0.3f,0.3f,0.3f));
		shader_mesh.setMat4("model",model);
		gl_model.Draw(shader_mesh);
	
		float path_arr[3*path_->size()];
		for (int i=0; i<path_->size(); i++) {
			int p = i*3;
			path_arr[p] = path_->at(i)->val(0);
			path_arr[p+1] = path_->at(i)->val(1);
			path_arr[p+2] = 0.f;
		}
		draw_path(path_->size(), path_arr, shader_path, vertBoxVAO, vertBoxVBO);

		float obs_arr[3*obstacles.size()];
		float obs_rad[obstacles.size()];
		for (int i=0; i<obstacles.size(); i++) {
			int p = i*3;
			obs_rad[i] = obstacles[i]->rad();
			obs_arr[p] = obstacles[i]->pos(0);
			obs_arr[p+1] = obstacles[i]->pos(1);
			obs_arr[p+2] = 0.f;
		}
		draw_obstacles(obstacles.size(), obs_arr, obs_rad, shader_obs_multi_color, shader_obs_multi_color, sphereVAO, metalTexture, sphereEBO);
		draw_wall(100,0,-10,40,shader_single_color, shader_multi_color, cubeVAO, cubeTexture);
		draw_wall(100,0,-10,-40,shader_single_color, shader_multi_color, cubeVAO, cubeTexture);
		draw_wall(80,1,-10,-40,shader_single_color, shader_multi_color, cubeVAO, cubeTexture);

		MatXf pred_states = opt->pred_states();
		float predictions[3*pred_states.cols()];
		for (int i=0; i<pred_states.cols(); i++) {
			int p = i*3;
			predictions[p] = pred_states.col(i)(0);
			predictions[p+1] = pred_states.col(i)(1);
			predictions[p+2] = pred_states.col(i)(3);
		}
		int n__ = sizeof(predictions) /(3*sizeof(float));
		draw_predictions(n__, predictions, shader_pred, vertBoxVAO, vertBoxVBO);

		float trajectory_arr[3*trajectory.size()];
		for (int i=0; i<trajectory.size(); i++) {
			int p = i*3;
			trajectory_arr[p] = trajectory[i](0);
			trajectory_arr[p+1] = trajectory[i](1);
			trajectory_arr[p+2] = 0.f;
		}
		shader_trajectory.use();
		model = glm::mat4(1.0f);
		model = glm::rotate(model, glm::radians(90.f), glm::vec3(0.f,0.f,1.f));
		shader_trajectory.setMat4("model", model);
		draw_trajectory(trajectory.size(), trajectory_arr, shader_trajectory, vertBoxVAO, vertBoxVBO);

		draw_tree(rrt, shader_tree, horzBoxVAO, horzBoxVBO); 
		glfwSwapBuffers(window);
		glfwPollEvents();	
		//dp->render(rrt, path_, trajectory, state, opt->pred_states(),obstacles);
	}
	glfwTerminate();
	return 0;
}

