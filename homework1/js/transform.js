
/**
 * @file functions to compute model/view/projection matrices
 *
 * @copyright The Board of Trustees of the Leland Stanford Junior University
 * @version 2022/03/31

 */

/**
 * MVPmat
 *
 * @class MVPmat
 * @classdesc Class for holding and computing model/view/projection matrices.
 *
 * @param  {DisplayParameters} dispParams    display parameters
 */
var MVPmat = function ( dispParams ) {

	// Alias for accessing this from a closure
	var _this = this;


	// A model matrix
	this.modelMat = new THREE.Matrix4();

	// A view matrix
	this.viewMat = new THREE.Matrix4();

	// A projection matrix
	this.projectionMat = new THREE.Matrix4();


	var topViewMat = new THREE.Matrix4().set(
		1, 0, 0, 0,
		0, 0, - 1, 0,
		0, 1, 0, - 1500,
		0, 0, 0, 1 );

	/* Functions */

	// A function to compute a model matrix based on the current state
	//
	// INPUT
	// state: state of StateController
	function computeModelTransform( state ) {

		/* TODO (2.1.1.3) Matrix Update / (2.1.2) Model Rotation  */
		const x_rotation_radians = THREE.MathUtils.degToRad(state.modelRotation.x);
		const y_rotation_radians = THREE.MathUtils.degToRad(state.modelRotation.y);
		const x_rotation = new THREE.Matrix4().makeRotationX(x_rotation_radians);
		const y_rotation = new THREE.Matrix4().makeRotationY(y_rotation_radians);
		const rotation = y_rotation.premultiply(x_rotation);

		const x = state.modelTranslation.x;
		const y = state.modelTranslation.y;
		const z = state.modelTranslation.z;
		const translation = new THREE.Matrix4().makeTranslation(x, y, z);

		return rotation.premultiply(translation);

	}

	// A function to compute a view matrix based on the current state
	//
	// NOTE
	// Do not use lookAt().
	//
	// INPUT
	// state: state of StateController
	function computeViewTransform( state ) {

		/* TODO (2.2.3) Implement View Transform */

		return new THREE.Matrix4().set(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, - 800,
			0, 0, 0, 1 );

	}

	// A function to compute a perspective projection matrix based on the
	// current state
	//
	// NOTE
	// Do not use makePerspective().
	//
	// INPUT
	// Notations for the input is the same as in the class.
	function computePerspectiveTransform(
		left, right, top, bottom, clipNear, clipFar ) {

		/* TODO (2.3.1) Implement Perspective Projection */
		const P_11 = (2 * clipNear) / (right - left);
		const P_13 = (right + left) / (right - left);
		const P_22 = (2 * clipNear) / (top - bottom);
		const P_23 = (top + bottom) / (top - bottom);
		const P_33 = - (clipFar + clipNear) / (clipFar - clipNear);
		const P_34 = - (2 * clipFar * clipNear) / (clipFar - clipNear);

		return new THREE.Matrix4().set(
			P_11, 0, P_13, 0,
			0, P_22, P_23, 0,
			0, 0, P_33, P_34,
			0, 0, - 1.0, 0 );

	}

	// A function to compute a orthographic projection matrix based on the
	// current state
	//
	// NOTE
	// Do not use makeOrthographic().
	//
	// INPUT
	// Notations for the input is the same as in the class.
	function computeOrthographicTransform(
		left, right, top, bottom, clipNear, clipFar ) {

		/* TODO (2.3.2) Implement Orthographic Projection */

		return new THREE.Matrix4();

	}

	// Update the model/view/projection matrices
	// This function is called in every frame (animate() function in render.js).
	function update( state ) {

		// Compute model matrix
		this.modelMat.copy( computeModelTransform( state ) );

		// Use the hard-coded view and projection matrices for top view
		if ( state.topView ) {

			this.viewMat.copy( topViewMat );

			var right = ( dispParams.canvasWidth * dispParams.pixelPitch / 2 )
				* ( state.clipNear / dispParams.distanceScreenViewer );

			var left = - right;

			var top = ( dispParams.canvasHeight * dispParams.pixelPitch / 2 )
				* ( state.clipNear / dispParams.distanceScreenViewer );

			var bottom = - top;

			this.projectionMat.makePerspective( left, right, top, bottom, 1, 10000 );

		} else {

			// Compute view matrix
			this.viewMat.copy( computeViewTransform( state ) );

			// Compute projection matrix
			if ( state.perspectiveMat ) {

				var right = ( dispParams.canvasWidth * dispParams.pixelPitch / 2 )
				* ( state.clipNear / dispParams.distanceScreenViewer );

				var left = - right;

				var top = ( dispParams.canvasHeight * dispParams.pixelPitch / 2 )
				* ( state.clipNear / dispParams.distanceScreenViewer );

				var bottom = - top;

				this.projectionMat.copy( computePerspectiveTransform(
					left, right, top, bottom, state.clipNear, state.clipFar ) );

			} else {

				var right = dispParams.canvasWidth * dispParams.pixelPitch / 2;

				var left = - right;

				var top = dispParams.canvasHeight * dispParams.pixelPitch / 2;

				var bottom = - top;

				this.projectionMat.copy( computeOrthographicTransform(
					left, right, top, bottom, state.clipNear, state.clipFar ) );

			}

		}

	}



	/* Expose as public functions */

	this.computeModelTransform = computeModelTransform;

	this.computeViewTransform = computeViewTransform;

	this.computePerspectiveTransform = computePerspectiveTransform;

	this.computeOrthographicTransform = computeOrthographicTransform;

	this.update = update;

};
