/**
 * @file Gouraud vertex shader
 *
 * @copyright The Board of Trustees of the Leland Stanford Junior University
 * @version 2021/04/01
 */

/* TODO (2.1.2) and (2.1.3) */

var shaderID = "vShaderGouraud";

var shader = document.createTextNode( `
/**
 * varying qualifier is used for passing variables from a vertex shader
 * to a fragment shader. In the fragment shader, these variables are
 * interpolated between neighboring vertexes.
 */
varying vec3 vColor; // Color at a vertex

uniform mat4 viewMat;
uniform mat4 projectionMat;
uniform mat4 modelViewMat;
uniform mat3 normalMat;

struct Material {
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

uniform Material material;

uniform vec3 attenuation;

uniform vec3 ambientLightColor;

attribute vec3 position;
attribute vec3 normal;


/***
 * NUM_POINT_LIGHTS is replaced to the number of point lights by the
 * replaceNumLights() function in teapot.js before the shader is compiled.
 */
#if NUM_POINT_LIGHTS > 0

	struct PointLight {
		vec3 position;
		vec3 color;
	};

	uniform PointLight pointLights[ NUM_POINT_LIGHTS ];

#endif


void main() {

	// Compute ambient reflection
	vec3 ambientReflection = material.ambient * ambientLightColor;

	// Compute diffuse reflection
	vec3 normalInViewSpace = normalize(normalMat * normal);
	vec3 lightInViewSpace = (viewMat * vec4(pointLights[0].position, 1.0)).xyz;
	vec3 positionInViewSpace = (modelViewMat * vec4(position, 1.0)).xyz;
	vec3 displacementInViewSpace = lightInViewSpace - positionInViewSpace;
	vec3 L = normalize(displacementInViewSpace);
	float angularAdjustment = max(dot(L, normalInViewSpace), 0.0);
	vec3 diffuseReflection = (material.diffuse * pointLights[0].color) * angularAdjustment;

	// Compute specular reflection
	vec3 R = reflect(-L, normalInViewSpace);
	vec3 V = normalize(-positionInViewSpace);
	float vantageAdjustment = pow(max(dot(R, V), 0.0), material.shininess);
	vec3 specularReflection = (material.specular * pointLights[0].color) * vantageAdjustment;

	// Compute the distance
	float distance = length(displacementInViewSpace);

	// Compute attenuation factor
	float k_c = attenuation.x;
	float k_l = attenuation.y;
	float k_q = attenuation.z;
	float attenuationFactor = 1.0 / (k_c + k_l * distance + k_q * pow(distance, 2.0));

	vColor = ambientReflection + attenuationFactor * (diffuseReflection + specularReflection);

	gl_Position =
		projectionMat * modelViewMat * vec4( position, 1.0 );

}
` );

var shaderNode = document.createElement( "script" );

shaderNode.id = shaderID;

shaderNode.setAttribute( "type", "x-shader/x-vertex" );

shaderNode.appendChild( shader );

document.body.appendChild( shaderNode );
