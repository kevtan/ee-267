/**
 * @file Fragment shader for foveated rendering
 *
 * @copyright The Board of Trustees of the Leland Stanford Junior University
 * @version 2022/04/14
 */

/* TODO (2.2.4) Fragment Shader Foveation Blur */

var shaderID = "fShaderFoveated";

var shader = document.createTextNode( `
/***
 * WebGL doesn't set any default precision for fragment shaders.
 * Precision for vertex shader is set to "highp" as default.
 * Do not use "lowp". Some mobile browsers don't support it.
 */
precision mediump float;

// texture or uv coordinates of current fragment in normalized coordinates [0,1]
varying vec2 textureCoords;

// texture map from the first rendering pass
uniform sampler2D textureMap;

// resolution of the window in [pixels]
uniform vec2 windowSize;

// window space coordinates of gaze position in [pixels]
uniform vec2 gazePosition;

// eccentricity angle at boundary of foveal and middle layers
uniform float e1;

// eccentricity angle at boundary of middle and outer layers
uniform float e2;

// visual angle of one pixel
uniform float pixelVA;

// radius of middle layer blur kernel [in pixels]
const float middleKernelRad = 2.0;

// radius of outer layer blur kernel [in pixels]
const float outerKernelRad = 4.0;

// gaussian blur kernel for middle layer (5x5)
uniform float middleBlurKernel[int(middleKernelRad)*2+1];

// gaussian blur kernel for outer layer (9x9)
uniform float outerBlurKernel[int(outerKernelRad)*2+1];

vec2 windowCoordsToTextureCoords(vec2 windowCoords) {
	return windowCoords / windowSize;
}

void main() {

	// Compute this fragment's eccentricity
	vec2 windowCoords = textureCoords * windowSize;
	float pixelDistance = distance(gazePosition, windowCoords);
	float eccentricity = pixelDistance * pixelVA;

	if (eccentricity < e1) {
		// Foveal Layer
		gl_FragColor = texture2D(textureMap,  textureCoords);	
	} else if (eccentricity < e2) {
		// Middle Layer
		vec4 blurredColor = vec4(0);
		for (int i = -int(middleKernelRad); i <= int(middleKernelRad); i++) {
			for (int j = -int(middleKernelRad); j <= int(middleKernelRad); j++) {
				vec2 _windowCoords = windowCoords + vec2(i, j);
				vec2 _textureCoords = windowCoordsToTextureCoords(_windowCoords);
				vec4 _color = texture2D(textureMap, _textureCoords);
				blurredColor += _color * middleBlurKernel[i + int(middleKernelRad)] * middleBlurKernel[j + int(middleKernelRad)];
			}
		}
		gl_FragColor = blurredColor;
	} else {
		// Outer Layer
		vec4 blurredColor = vec4(0);
		for (int i = -int(outerKernelRad); i <= int(outerKernelRad); i++) {
			for (int j = -int(outerKernelRad); j <= int(outerKernelRad); j++) {
				vec2 _windowCoords = windowCoords + vec2(i, j);
				vec2 _textureCoords = windowCoordsToTextureCoords(_windowCoords);
				vec4 _color = texture2D(textureMap, _textureCoords);
				blurredColor += _color * outerBlurKernel[i + int(outerKernelRad)] * outerBlurKernel[j + int(outerKernelRad)];
			}
		}
		gl_FragColor = blurredColor;
	}
}
` );

var shaderNode = document.createElement( "script" );

shaderNode.id = shaderID;

shaderNode.setAttribute( "type", "x-shader/x-fragment" );

shaderNode.appendChild( shader );

document.body.appendChild( shaderNode );
