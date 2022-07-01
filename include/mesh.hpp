#include <vector>
#include <glm/gtc/matrix_transform.hpp>
#include "shader.hpp"

#define MAX_BONE_INFLUENCE 4

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
		glm::vec3 Tangent;
		glm::vec3 Bitangent;
		int m_BoneIDs[MAX_BONE_INFLUENCE];
		int m_Weights[MAX_BONE_INFLUENCE];
};

struct Texture {
    unsigned int id;
    std::string type;
		std::string path;
}; 

class Mesh {
	private:
		//  render data
		unsigned int VAO, VBO, EBO;

		void setupMesh() {
			glGenVertexArrays(1, &VAO);
			glGenBuffers(1, &VBO);
			glGenBuffers(1, &EBO);

			glBindVertexArray(VAO);
			glBindBuffer(GL_ARRAY_BUFFER, VBO);
			glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex), (void*)0);
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vertex), (void*)offsetof(Vertex,Normal));
			glEnableVertexAttribArray(2);
			glVertexAttribPointer(2,2,GL_FLOAT,GL_FALSE,sizeof(Vertex), (void*)offsetof(Vertex,TexCoords));
			glEnableVertexAttribArray(3);
			glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,sizeof(Vertex), (void*)offsetof(Vertex,Tangent));
			glEnableVertexAttribArray(4);
			glVertexAttribPointer(4,3,GL_FLOAT,GL_FALSE,sizeof(Vertex), (void*)offsetof(Vertex, Bitangent));
			// ids
			glEnableVertexAttribArray(5);
			glVertexAttribIPointer(5,4,GL_INT,sizeof(Vertex), (void*)offsetof(Vertex, m_BoneIDs));
			// weights
			glEnableVertexAttribArray(6);
			glVertexAttribPointer(6,4,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void*)offsetof(Vertex,m_Weights));
			glBindVertexArray(0);
		}
	public:
		std::vector<Vertex>       vertices;
		std::vector<unsigned int> indices;
		std::vector<Texture>      textures;

		Mesh(std::vector<Vertex> _vertices, std::vector<unsigned int> _indices, std::vector<Texture> _textures) : vertices(_vertices), indices(_indices), textures(_textures) {setupMesh();}
		
		void Draw(Shader &shader) {
			unsigned int diffuseNr = 1;
			unsigned int specularNr = 1;
			unsigned int normalNr = 1;
			unsigned int heightNr = 1;
			for(unsigned int i = 0; i < textures.size(); i++) {
				// activate proper texture unit before binding
				glActiveTexture(GL_TEXTURE0 + i); 
				// retrieve texture number (the N in diffuse_textureN)
				std::string number;
				std::string name = textures[i].type;
				if (name == "texture_diffuse")
					number = std::to_string(diffuseNr++);
				else if (name == "texture_specular")
					number = std::to_string(specularNr++);
				else if (name == "texture_normal")
					number == std::to_string(normalNr++);
				else if (name == "texture_height")
					number = std::to_string(heightNr++);

				glUniform1i(glGetUniformLocation(shader.ID, (name+number).c_str()), i);
				glBindTexture(GL_TEXTURE_2D, textures[i].id);
			}
			// draw mesh
			glBindVertexArray(VAO);
			glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
			glBindVertexArray(0);
			glActiveTexture(GL_TEXTURE0);
		}
};

