/**
 * @file Phong fragment shader with point and directional lights
 *
 * @copyright The Board of Trustees of the Leland Stanford Junior University
 * @version 2021/04/01
 */

/* TODO (2.3) */

var shaderID = "fShaderMultiPhong";

var shader = document.createTextNode( `
/**
 * WebGL doesn't set any default precision for fragment shaders.
 * Precision for vertex shader is set to "highp" as default.
 * Do not use "lowp". Some mobile browsers don't support it.
 */
precision mediump float;

varying vec3 normalCam; // Normal in view coordinate
varying vec3 fragPosCam; // Fragment position in view cooridnate

uniform mat4 viewMat;

struct Material {
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

uniform Material material;

uniform vec3 attenuation;

uniform vec3 ambientLightColor;

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

/***
 * NUM_DIR_LIGHTS is replaced to the number of directional lights by the
 * replaceNumLights() function in teapot.js before the shader is compiled.
 */
#if NUM_DIR_LIGHTS > 0

	struct DirectionalLight {
		vec3 direction;
		vec3 color;
	};

	uniform DirectionalLight directionalLights[ NUM_DIR_LIGHTS ];

#endif


void main() {
	// Re-normalize normal
	vec3 normalInViewSpace = normalize(normalCam);

	// Alias position for code compatibility
	vec3 positionInViewSpace = fragPosCam;

	// Compute ambient reflection
	vec3 ambientReflection = material.ambient * ambientLightColor;

	// Accumulators
	vec3 totalDiffuseReflection = vec3(0);
	vec3 totalSpecularReflection = vec3(0);

	// Superimpose point lights
	for (int i = 0; i < NUM_POINT_LIGHTS; i++) {

		// Compute diffuse reflection
		vec3 lightInViewSpace = (viewMat * vec4(pointLights[i].position, 1.0)).xyz;
		vec3 displacementInViewSpace = lightInViewSpace - positionInViewSpace;
		vec3 L = normalize(displacementInViewSpace);
		float angularAdjustment = max(dot(L, normalInViewSpace), 0.0);
		vec3 diffuseReflection = (material.diffuse * pointLights[i].color) * angularAdjustment;

		// Compute specular reflection
		vec3 R = reflect(-L, normalInViewSpace);
		vec3 V = normalize(-positionInViewSpace);
		float vantageAdjustment = pow(max(dot(R, V), 0.0), material.shininess);
		vec3 specularReflection = (material.specular * pointLights[i].color) * vantageAdjustment;

		// Compute the distance
		float distance = length(displacementInViewSpace);

		// Compute attenuation factor
		float k_c = attenuation.x;
		float k_l = attenuation.y;
		float k_q = attenuation.z;
		float attenuationFactor = 1.0 / (k_c + k_l * distance + k_q * pow(distance, 2.0));

		// Update accumulators
		totalDiffuseReflection += attenuationFactor * diffuseReflection;
		totalSpecularReflection += attenuationFactor * specularReflection;

	}

	// Superimpose directional lights
	for (int i = 0; i < NUM_DIR_LIGHTS; i++) {

		// Compute diffuse reflection
		vec3 L = normalize((viewMat * -vec4(directionalLights[i].direction, 1.0)).xyz);
		float angularAdjustment = max(dot(L, normalInViewSpace), 0.0);
		vec3 diffuseReflection = (material.diffuse * directionalLights[i].color) * angularAdjustment;

		// Compute specular reflection
		vec3 R = reflect(-L, normalInViewSpace);
		vec3 V = normalize(-positionInViewSpace);
		float vantageAdjustment = pow(max(dot(R, V), 0.0), material.shininess);
		vec3 specularReflection = (material.specular * directionalLights[i].color) * vantageAdjustment;

		// Update accumulators
		totalDiffuseReflection += diffuseReflection;
		totalSpecularReflection += specularReflection;

	}

	vec3 fColor = ambientReflection + totalDiffuseReflection + totalSpecularReflection;

	gl_FragColor = vec4( fColor, 1.0 );

}
` );

var shaderNode = document.createElement( "script" );

shaderNode.id = shaderID;

shaderNode.setAttribute( "type", "x-shader/x-fragment" );

shaderNode.appendChild( shader );

document.body.appendChild( shaderNode );
