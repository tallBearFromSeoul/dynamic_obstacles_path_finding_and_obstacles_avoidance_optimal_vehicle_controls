#include <iostream>
#include "model.hpp"
#include "camera.hpp"
#include "vertices.hpp"
#include "display_utility.hpp"

int main(int argc, char *argv[]) {
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
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
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

	unsigned int floorTexture = loadTexture("../data/textures/road.jpg");
	unsigned int cubeTexture = loadTexture("../data/textures/brick.png");

	Shader shader_mesh("../config/model_loading.vs","../config/model_loading.fs");
	Shader shader_single_color("../config/single_color.vs", "../config/single_color.fs");
	Shader shader_multi_color("../config/multi_color.vs", "../config/multi_color.fs");
	
	Shader shader_forward("../config/forward.vs","../config/forward.fs");
	Shader shader_forward2("../config/forward2.vs","../config/forward2.fs");

	//GLModel gl_model("../Material/bus_setia_negara_texturizer.blend");
	GLModel gl_model("../14-lowpolyfiatuno/LowPolyFiatUNO.obj");
	
	float i = 0;
	while (!glfwWindowShouldClose(window)) {
		float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		
		processInput(window);
		
		glClearColor(0.7f, 0.7f, 0.7f, 0.5f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		glm::mat4 model = glm::mat4(1.0f);
		glm::mat4 view = camera.GetViewMatrix();
		glm::mat4 projection = glm::perspective(glm::radians(45.f), float(W)/float(H), 0.05f, 90.0f);	
	
		shader_forward.use();
		shader_forward.setMat4("model",model);
		shader_forward.setMat4("view",view);
		shader_forward.setMat4("projection",projection);
		
		shader_forward2.use();
		shader_forward2.setMat4("model",model);
		shader_forward2.setMat4("view",view);
		shader_forward2.setMat4("projection",projection);
		
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
		model = glm::translate(model ,glm::vec3(i,0.f,0.f));
		model = glm::rotate(model, glm::radians(90.f),glm::vec3(1.f,0.f,0.f));
		model = glm::rotate(model, glm::radians(90.f),glm::vec3(0.f,1.f,0.f));
		shader_mesh.setMat4("model",model);
		gl_model.Draw(shader_mesh);
		float obs[9] {5.f,3.f,0.f,3.f,2.f,0.f,7.f,1.f,0.f,};
		draw_obstacles(3, obs, shader_single_color, shader_multi_color, cubeVAO, cubeTexture);
		draw_wall(100,0,-10,40,shader_single_color, shader_multi_color, cubeVAO, cubeTexture);
		draw_wall(100,0,-10,-40,shader_single_color, shader_multi_color, cubeVAO, cubeTexture);
		draw_wall(80,1,-10,-40,shader_single_color, shader_multi_color, cubeVAO, cubeTexture);

		float predictions[6] {5.f,0.f,0.f,10.f,0.f,0.f};
		int n__ = sizeof(predictions) /(3*sizeof(float));
		draw_predictions(n__, predictions, shader_forward, vertBoxVAO, vertBoxVBO);
		float trajectory[9] {-3.f,0.f,0.f,-1.f,0.f,0.f,1.f,0.f,0.f};
		draw_trajectory(3, trajectory, shader_forward2, vertBoxVAO, vertBoxVBO);


		glfwSwapBuffers(window);
		glfwPollEvents();	
	}
	glfwTerminate();
	return 0;
}

