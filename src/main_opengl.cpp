#include <iostream>
#include "model.hpp"
#include "camera.hpp"
#include "vertices.hpp"
#include "display_utility.hpp"

int main(int argc, char *argv[]) {
		
	float i = 0;
	while (!glfwWindowShouldClose(window)) {
				
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
	
			}
	glfwTerminate();
	return 0;
}

