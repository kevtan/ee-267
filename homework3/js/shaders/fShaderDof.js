/**
 * @file Fragment shader for DoF rendering
 *
 * @copyright The Board of Trustees of the Leland Stanford Junior University
 * @version 2022/04/14
 */

/* TODO (2.3) DoF Rendering */

var shaderID = "fShaderDof";

var shader = document.createTextNode( `
/**
 * WebGL doesn't set any default precision for fragment shaders.
 * Precision for vertex shader is set to "highp" as default.
 * Do not use "lowp". Some mobile browsers don't support it.
 */
precision mediump float;

// uv coordinates after interpolation
varying vec2 textureCoords;

// texture map from the first rendering
uniform sampler2D textureMap;

// depth map from the first rendering
uniform sampler2D depthMap;

// Projection matrix used for the first pass
uniform mat4 projectionMat;

// Inverse of projectionMat
uniform mat4 invProjectionMat;

// resolution of the window in [pixels]
uniform vec2 windowSize;

// Gaze position in [pixels]
uniform vec2 gazePosition;

// Diameter of pupil in [mm]
uniform float pupilDiameter;

// pixel pitch in [mm]
uniform float pixelPitch;

const float searchRad = 11.0;

vec2 textureCoordsToWindowCoords(vec2 textureCoords) {
	return textureCoords * windowSize;
}

vec3 windowCoordsToNDC(vec3 windowCoords) {
	float width = windowSize.x;
	float height = windowSize.y;
	float x = ((2.0 * windowCoords.x) / width) - 1.0;
	float y = ((2.0 * windowCoords.y) / height) - 1.0;
	float z = (2.0 * windowCoords.z) - 1.0;
	return vec3(x, y, z);
}

vec4 NDCToClipCoords(vec3 ndc) {
	float w = projectionMat[2][3] / (ndc.z + projectionMat[2][2]);
	return vec4(ndc * w, w);
}

vec4 clipCoordsToViewCoords(vec4 clipCoords) {
	return invProjectionMat * clipCoords;
}

// Compute the distance to fragment in [mm]
// p: texture coordinate of a fragment / a gaze position
//
// Note: GLSL is column major
float distToFrag( vec2 p ) {

	/* TODO (2.3.1) Distance to Fragment */
	vec2 xyWindow = textureCoordsToWindowCoords(p);
	float zWindow = texture2D(depthMap, p).x;
	vec3 windowCoords = vec3(xyWindow, zWindow);
	vec3 ndc = windowCoordsToNDC(windowCoords);
	vec4 clipCoords = NDCToClipCoords(ndc);
	vec4 viewCoords = clipCoordsToViewCoords(clipCoords);

	return length(viewCoords.xyz);

}


// compute the circle of confusion in [mm]
// fragDist: distance to current fragment in [mm]
// focusDist: distance to focus plane in [mm]
float computeCoC( float fragDist, float focusDist ) {

	/* TODO (2.3.2) Circle of Confusion Computation */

	return 0.0;

}


// compute depth of field blur and return color at current fragment
vec3 computeBlur() {

	/* TODO (2.3.3) Retinal Blur */

	return vec3( 0.0 );
}


void main() {

	gl_FragColor = texture2D( textureMap,  textureCoords );

}
` );


var shaderNode = document.createElement( "script" );

shaderNode.id = shaderID;

shaderNode.setAttribute( "type", "x-shader/x-fragment" );

shaderNode.appendChild( shader );

document.body.appendChild( shaderNode );
