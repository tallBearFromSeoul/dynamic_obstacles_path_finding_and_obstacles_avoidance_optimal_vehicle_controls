#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
/*
void draw_vert_plane(int p, float *predictions, std::vector<float *> &, Shader &shader_forward, float yl, float yu, float zl, float zu);
void draw_horz_plane(int p, float *predictions, Shader &shader_forward, float xl, float xu, float yl, float yu, float zu);
void draw_predictions(int, float *predictions, std::vector<float *>, Shader &);
*/
void draw_wall(int n_cubes, int axis, float X, float Y, Shader &shader_single_color, Shader &shader_multi_color, unsigned int cubeVAO, unsigned int cubeTexture);
void draw_cube(Shader &shader_single_color, Shader &shader_multi_color, float X, float Y, unsigned int planeVAO, unsigned int floorTexture);

void framebuffer_size_callback(GLFWwindow*, int, int);
void mouse_callback(GLFWwindow *, double, double);
void scroll_callback(GLFWwindow *, double, double);
void processInput(GLFWwindow *);

bool highlight = false;
bool firstMouse = true;
const unsigned int W = 1080;
const unsigned int H = 720;
float lastX = W / 2.f;
float lastY = H / 2.f;
float deltaTime = 0.f;
float lastFrame = 0.f;
Camera camera(glm::vec3(0.0f, 0.0f, 2.0f));

void draw_vert_plane(int i, float scale, float *predictions, Shader &shader_forward, unsigned int vertBoxVAO, unsigned int vertBoxVBO) {
	shader_forward.use();
	float x = *(predictions+i*3);
	float y = *(predictions+i*3+1);
	float z = *(predictions+i*3+2);
	glm::mat4 model = glm::mat4(1.0f);
	model = glm::translate(model, glm::vec3(x,y,z));
	model = glm::scale(model, glm::vec3(0,scale, scale));
	shader_forward.setMat4("model", model);
	glBindVertexArray(vertBoxVAO);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDrawArrays(GL_TRIANGLES,0,18);
	glDepthMask( GL_TRUE ); 
	glDisable( GL_BLEND );
	glBindVertexArray(0);
}

void draw_predictions(int n__, float *predictions, Shader &shader_forward, unsigned int vertBoxVAO, unsigned int vertBoxVBO) {
	for (int i=0; i<n__; i++) {
		int p = i*18;
		float scale = float(i+1)/float(n__);
		draw_vert_plane(i, scale, predictions, shader_forward, vertBoxVAO, vertBoxVBO);
	}
}

void draw_trajectory(int n__t, float *trajectory, Shader &shader_forward, unsigned int vertBoxVAO, unsigned int vertBoxVBO) {
	for (int i=0; i<n__t; i++) {
		int p = i*18;
		draw_vert_plane(i, 1.f, trajectory, shader_forward, vertBoxVAO, vertBoxVBO);
	}

}

void draw_obstacles(int n__o, float *obs, Shader &shader_single_color, Shader &shader_multi_color, unsigned int cubeVAO, unsigned int cubeTexture) {
	float x, y, z;
	for (int i=0; i<n__o; i++) {
		x = *(obs+i*3);
		y = *(obs+i*3+1);
		z = *(obs+i*3+2);
		draw_cube(shader_single_color, shader_multi_color, x, y, cubeVAO, cubeTexture);
	}	
}

void draw_wall(int n_cubes, int axis, float X, float Y, Shader &shader_single_color, Shader &shader_multi_color, unsigned int cubeVAO, unsigned int cubeTexture) {
	switch(axis) {
		case(0):
			for (float i=0; i<n_cubes; i++) {
				draw_cube(shader_single_color, shader_multi_color, X+i*1, Y, cubeVAO, cubeTexture);
			}
			break;
		case(1):
			for (int i=0; i<n_cubes; i++) {
				draw_cube(shader_single_color, shader_multi_color, X, i*1+Y, cubeVAO, cubeTexture);
			}
			break;
	}
}

void draw_cube(Shader &shader_single_color, Shader &shader_multi_color, float X, float Y, unsigned int cubeVAO, unsigned int cubeTexture) {
	shader_multi_color.use();
	glStencilFunc(GL_ALWAYS, 1, 0xFF);
	glStencilMask(0xFF);
	// cubes
	glm::mat4 model(1.0f);
	model = glm::translate(model, glm::vec3(X, Y, 0.5f));
	shader_multi_color.setMat4("model", model);
	glBindVertexArray(cubeVAO);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, cubeTexture);
	glDrawArrays(GL_TRIANGLES, 0, 36);
	model = glm::mat4(1.0f);
	model = glm::translate(model, glm::vec3(X, Y, 0.5f));
	shader_multi_color.setMat4("model", model);
	glDrawArrays(GL_TRIANGLES, 0, 36);
	
	if (highlight) {
		glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
		glStencilMask(0x00);
		glDisable(GL_DEPTH_TEST);
		shader_single_color.use();
		// cubes
		float scale = 1.07f;
		glBindVertexArray(cubeVAO);
		glBindTexture(GL_TEXTURE_2D, cubeTexture);
		model = glm::mat4(1.0f);
		model = glm::translate(model, glm::vec3(X, Y, 0.5f));
		model = glm::scale(model, glm::vec3(scale, scale, scale));
		shader_single_color.setMat4("model", model);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glBindVertexArray(0);
		glStencilMask(0xFF);
		glStencilFunc(GL_ALWAYS, 0, 0xFF);
		glEnable(GL_DEPTH_TEST);
	}
	//glStencilMask(0x00);
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
	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
		highlight = true;
	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
		highlight = false;
}

