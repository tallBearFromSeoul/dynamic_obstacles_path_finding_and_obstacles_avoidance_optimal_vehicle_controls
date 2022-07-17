#include <glad/glad.h>
 #include <GLFW/glfw3.h>
 #include <glm/glm.hpp>
 #include <glm/gtc/matrix_transform.hpp>
 #include <glm/gtc/type_ptr.hpp>

 void draw_predictions(int, float *predictions, std::vector<float *>, Shader &);
 void draw_wall(int n_cubes, int axis, float X, float Y, Shader &shader_single_color, Shader &shader_multi_color, unsigned int cubeVAO, unsigned int cubeTexture);
 void draw_objet(Shader &shader_single_color, Shader &shader_multi_color, float X, float Y, float scale, unsigned int VAO, unsigned int floorTexture, bool element, unsigned int EBO=0);

 void framebuffer_size_callback(GLFWwindow*, int, int);
 void mouse_callback(GLFWwindow *, double, double);
 void scroll_callback(GLFWwindow *, double, double);
 void processInput(GLFWwindow *);

 bool fpv = false;
 bool highlight = false;
 bool firstMouse = true;
 const unsigned int W = 1080;
 const unsigned int H = 720;
 float lastX = W / 2.f;
 float lastY = H / 2.f;
 float deltaTime = 0.f;
 float lastFrame = 0.f;
 Camera camera(glm::vec3(5.0f, 5.0f, 15.0f), glm::vec3(0.f,0.f,1.f), 45.f, -75.f);


 glfwInit();
 glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
 glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
 glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
 glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

 GLFWwindow* window = glfwCreateWindow(W, H, "MPC_dynamic_environment_3D_rendering", nullptr, nullptr);
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

 void render() {
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

 	float path_arr[3*path_.size()];
 	for (int i=0; i<path_.size(); i++) {
 		int p = i*3;
 		path_arr[p] = path_.at(i)->val(0);
 		path_arr[p+1] = path_.at(i)->val(1);
 		path_arr[p+2] = 0.f;
 	}
 	draw_path(path_.size(), path_arr, shader_path, vertBoxVAO, vertBoxVBO);

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

 	//draw_tree(rrt, shader_tree, horzBoxVAO, horzBoxVBO); 
 	//dp->render(rrt, &path_, trajectory, state, opt->pred_states(),obstacles);
 	glfwSwapBuffers(window);
 	glfwPollEvents();	

 }

 void draw_edge(const NodePtr &__src, const NodePtr &__dst, Shader &shader, unsigned int horzBoxVAO, unsigned int horzBoxVBO) {
 	float x0, y0, x1, y1;
 	x0 = __src->val(0);
 	y0 = __src->val(1);
 	x1 = __dst->val(0);
 	y1 = __dst->val(1);
 	shader.use();
 	glm::mat4 model = glm::mat4(1.0f);
 	model = glm::translate(model, glm::vec3(x0,y0,0.1));
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
 void draw_vert_plane(int i, float scale, float *predictions, Shader &shader, unsigned int vertBoxVAO, unsigned int vertBoxVBO) {
 	shader.use();
 	float x = *(predictions+i*3);
 	float y = *(predictions+(i*3)+1);
 	float head = *(predictions+(i*3)+2);
 	glm::mat4 model = glm::mat4(1.0f);
 	model = glm::translate(model, glm::vec3(x,y,0));
 	model = glm::rotate(model, head, glm::vec3(0,0,1));
 	model = glm::scale(model, glm::vec3(1,scale, scale*2));
 	shader.setMat4("model", model);
 	glBindVertexArray(vertBoxVAO);
 	glDrawArrays(GL_TRIANGLES,0,18);
 	model = glm::rotate(model, glm::radians(90.f), glm::vec3(0,0,1));
 	shader.setMat4("model",model);
 	glDrawArrays(GL_TRIANGLES,0,18);
 	glDepthMask( GL_TRUE ); 
 	glDisable( GL_BLEND );
 	glBindVertexArray(0);
 }

 void draw_path(int n__, float *predictions, Shader &shader_path, unsigned int vertBoxVAO, unsigned int vertBoxVBO) {
 	for (int i=0; i<n__; i++) {
 		draw_vert_plane(i, 0.2f, predictions, shader_path, vertBoxVAO, vertBoxVBO);
 	}
 }

 void draw_predictions(int n__, float *predictions, Shader &shader_forward, unsigned int vertBoxVAO, unsigned int vertBoxVBO) {
 	for (int i=0; i<n__; i++) {
 		int p = i*18;
 		draw_vert_plane(i, 0.2f, predictions, shader_forward, vertBoxVAO, vertBoxVBO);
 	}
 }

 void draw_trajectory(int n__t, float *trajectory, Shader &shader_forward, unsigned int vertBoxVAO, unsigned int vertBoxVBO) {
 	for (int i=0; i<n__t; i++) {
 		int p = i*18;
 		draw_vert_plane(i, 0.2f, trajectory, shader_forward, vertBoxVAO, vertBoxVBO);
 	}

 }

 void draw_obstacles(int n__o, float *obs, float *obs_rad, Shader &shader_single_color, Shader &shader_multi_color, unsigned int VAO, unsigned int Texture, unsigned int EBO=0) {
 	float x, y, z, scale;
 	for (int i=0; i<n__o; i++) {
 		x = *(obs+i*3);
 		y = *(obs+i*3+1);
 		scale = *(obs_rad+i);
 		draw_objet(shader_single_color, shader_multi_color, x, y, scale, VAO, Texture, true, EBO);
 	}	
 }

 void draw_wall(int n_cubes, int axis, float X, float Y, Shader &shader_single_color, Shader &shader_multi_color, unsigned int cubeVAO, unsigned int cubeTexture) {
 	switch(axis) {
 		case(0):
 			for (float i=0; i<n_cubes; i++) {
 				draw_objet(shader_single_color, shader_multi_color, X+i*1, Y, 1.f, cubeVAO, cubeTexture, false);
 			}
 			break;
 		case(1):
 			for (int i=0; i<n_cubes; i++) {
 				draw_objet(shader_single_color, shader_multi_color, X, i*1+Y, 1.f, cubeVAO, cubeTexture, false);
 			}
 			break;
 	}
 }

 void draw_objet(Shader &shader_single_color, Shader &shader_multi_color, float X, float Y, float scale, unsigned int VAO, unsigned int Texture, bool element, unsigned int EBO) {
 	shader_multi_color.use();
 	glStencilFunc(GL_ALWAYS, 1, 0xFF);
 	glStencilMask(0xFF);
 	// cubes
 	glm::mat4 model(1.0f);
 	if (element) {
 		model = glm::translate(model, glm::vec3(X, Y, scale));
 		model = glm::scale(model, glm::vec3(scale, scale, scale));
 	} else {
 		model = glm::translate(model, glm::vec3(X, Y, 0.5f));
 	}

 	shader_multi_color.setMat4("model", model);
 	glActiveTexture(GL_TEXTURE0);
 	glBindTexture(GL_TEXTURE_2D, Texture);
 	glBindVertexArray(VAO);
 	if (element) {
 		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
 		glDrawElements(GL_TRIANGLES, cubesphere.getIndexSize(), GL_UNSIGNED_INT, (void*)0);
 	} else {
 		glDrawArrays(GL_TRIANGLES, 0, 36);
 	}
 	model = glm::mat4(1.0f);
 	model = glm::translate(model, glm::vec3(X, Y, 0.5f));
 	shader_multi_color.setMat4("model", model);
 	if (element) {
 		glDrawElements(GL_TRIANGLES, cubesphere.getIndexSize(), GL_UNSIGNED_INT, (void*)0);
 	} else {
 		glDrawArrays(GL_TRIANGLES, 0, 36);
 	}

 	if (highlight) {
 		glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
 		glStencilMask(0x00);
 		glDisable(GL_DEPTH_TEST);
 		shader_single_color.use();
 		// cubes
 		float scale = 1.07f;
 		glBindTexture(GL_TEXTURE_2D, Texture);
 		model = glm::mat4(1.0f);
 		model = glm::translate(model, glm::vec3(X, Y, 0.5f));
 		model = glm::scale(model, glm::vec3(scale, scale, scale));
 		shader_single_color.setMat4("model", model);
 		glBindVertexArray(VAO);
 		if (element) {
 			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
 			glDrawElements(GL_TRIANGLES, cubesphere.getIndexCount(), GL_UNSIGNED_INT, (void*)0);
 		} else {
 			glDrawArrays(GL_TRIANGLES, 0, 36);
 		}

 		glBindVertexArray(0);
 		glStencilMask(0xFF);
 		glStencilFunc(GL_ALWAYS, 0, 0xFF);
 		glEnable(GL_DEPTH_TEST);
 	}
 }


 void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
 	glViewport(0, 0, width, height);
 }

 void mouse_callback(GLFWwindow *window, double xposIn, double yposIn) {
 	float xpos = static_cast<float>(xposIn);
 	float ypos = static_cast<float>(yposIn);
 	if (firstMouse) {
 		lastX = xpos;
 		lastY = ypos;
 		firstMouse = false;
 	}
 	float xoffset = xpos-lastX;
 	float yoffset = lastY-ypos;
 	lastX=xpos;
 	lastY=ypos;
 	camera.ProcessMouseMovement(xoffset, yoffset);
 }
 void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
     camera.ProcessMouseScroll(static_cast<float>(yoffset));
 }

 void processInput(GLFWwindow *window) {
 	if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
 		glfwSetWindowShouldClose(window, true);

 	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
 		camera.ProcessKeyboard(FORWARD, deltaTime);
 	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
 		camera.ProcessKeyboard(BACKWARD, deltaTime);
 	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
 		camera.ProcessKeyboard(LEFT, deltaTime);
 	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
 		camera.ProcessKeyboard(RIGHT, deltaTime);
 	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
 		highlight = true;
 		fpv = true;
 	}
 	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE) {
 		highlight = false;
 		fpv = false;
 	}
 }
