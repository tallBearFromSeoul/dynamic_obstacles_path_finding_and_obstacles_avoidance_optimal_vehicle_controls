#version 330 core
out vec4 FragColor;
in vec3 bPos, bNormal;
in vec2 bTexCoords;

uniform vec4 lightPosition;             // should be in the eye space
uniform vec4 lightAmbient;              // light ambient color
uniform vec4 lightDiffuse;              // light diffuse color
uniform vec4 lightSpecular;             // light specular color
uniform vec4 materialAmbient;           // material ambient color
uniform vec4 materialDiffuse;           // material diffuse color
uniform vec4 materialSpecular;          // material specular color
uniform float materialShininess;        // material specular shininess
uniform sampler2D texture1;                 // texture map #1
uniform bool textureUsed;               // flag for texture

void main() {
	vec3 normal = normalize(bNormal);
	vec3 light;
	if (lightPosition.w == 0.0) {
		light = normalize(lightPosition.xyz);
	} else {
		light = normalize(lightPosition.xyz-bPos);
	}
	vec3 view = normalize(-bPos);
	vec3 halfv = normalize(light+view);
	vec3 color = lightAmbient.rgb * materialAmbient.rgb;
	float dotNL = max(dot(normal, light), 0.0);
	color += lightDiffuse.rgb * materialDiffuse.rgb * dotNL;    // add diffuse
	if(textureUsed)
		color *= texture(texture1, bTexCoords).rgb;
	float dotNH = max(dot(normal, halfv), 0.0);
	color += pow(dotNH, materialShininess) * lightSpecular.rgb * materialSpecular.rgb; // add specular

  // set frag color
  FragColor = vec4(color, materialDiffuse.a);

		
	//FragColor = texture(texture1, bTexCoords);
}
