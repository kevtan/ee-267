#include "OrientationMath.h"

/** TODO: see documentation in header file */
double computeAccPitch(double acc[3]) {

  double acc_magnitude = sqrt(sq(acc[0]) + sq(acc[1]) + sq(acc[2]));
  double acc_direction[3] = {acc[0] / acc_magnitude, acc[1] / acc_magnitude, acc[2] / acc_magnitude};
  
  return -degrees(atan2(acc_direction[2], (acc_direction[1] > 0 ? 1 : (acc_direction[1] < 0 ? -1 : 0)) * sqrt(sq(acc_direction[0]) + sq(acc_direction[1]))));

}

/** TODO: see documentation in header file */
double computeAccRoll(double acc[3]) {

  double acc_magnitude = sqrt(sq(acc[0]) + sq(acc[1]) + sq(acc[2]));
  double acc_direction[3] = {acc[0] / acc_magnitude, acc[1] / acc_magnitude, acc[2] / acc_magnitude};

  return -degrees(atan2(-acc_direction[0], acc_direction[1]));

}

/** TODO: see documentation in header file */
double computeFlatlandRollGyr(double flatlandRollGyrPrev, double gyr[3], double deltaT) {

  return flatlandRollGyrPrev + gyr[2] * deltaT;

}

/** TODO: see documentation in header file */
double computeFlatlandRollAcc(double acc[3]) {

  return atan2(acc[0], acc[1]) * 180 / PI;

}

/** TODO: see documentation in header file */
double computeFlatlandRollComp(double flatlandRollCompPrev, double gyr[3], double flatlandRollAcc, double deltaT, double alpha) {

  double flatlandRollGyr = computeFlatlandRollGyr(flatlandRollCompPrev, gyr, deltaT);
  return alpha * flatlandRollGyr + (1 - alpha) * flatlandRollAcc;

}


/** TODO: see documentation in header file */
void updateQuaternionGyr(Quaternion& q, double gyr[3], double deltaT) {

  const double ANGULAR_VELOCITY_THRESHOLD = 1e-8;

  // Compute the angular velocity
  double angularVelocity = sqrt(sq(gyr[0]) + sq(gyr[1]) + sq(gyr[2]));
  if (angularVelocity < ANGULAR_VELOCITY_THRESHOLD) return;

  // Construct instantaneous rotation quaternion
  double angle = angularVelocity * deltaT;
  double axis[3] = {
    gyr[0] / angularVelocity,
    gyr[1] / angularVelocity,
    gyr[2] / angularVelocity
  };
  Quaternion instantaneousRotationQuaternion;
  instantaneousRotationQuaternion.setFromAngleAxis(angle, axis[0], axis[1], axis[2]);
  instantaneousRotationQuaternion.normalize();

  // Update the quaternion estimate
  q = Quaternion::multiply(q, instantaneousRotationQuaternion);
  
}


/** TODO: see documentation in header file */
void updateQuaternionComp(Quaternion& q, double gyr[3], double acc[3], double deltaT, double alpha) {
  // q is the previous quaternion estimate
  // update it to be the new quaternion estimate

  updateQuaternionGyr(q, gyr, deltaT);

  // Construct acceleration vector quaternion
  Quaternion accelerationVectorQuaternion{0, acc[0], acc[1], acc[2]};
  accelerationVectorQuaternion = accelerationVectorQuaternion.rotate(q);
  accelerationVectorQuaternion.normalize();

  // Find the azimuthal angle
  double phi = degrees(acos(accelerationVectorQuaternion.q[2]));

  // Find the rotation axis
  double rotationAxis[3] = {-accelerationVectorQuaternion.q[3], 0, accelerationVectorQuaternion.q[1]};
  double rotationAxisNorm = sqrt(sq(rotationAxis[0]) + sq(rotationAxis[1]) + sq(rotationAxis[2]));
  rotationAxis[0] /= rotationAxisNorm;
  rotationAxis[1] /= rotationAxisNorm;
  rotationAxis[2] /= rotationAxisNorm;

  // Construct tilt correction quaternion
  Quaternion tiltCorrectionQuaternion;
  tiltCorrectionQuaternion.setFromAngleAxis((1 - alpha) * phi, rotationAxis[0], rotationAxis[1], rotationAxis[2]);

  // Perform tilt correction
  q = Quaternion::multiply(tiltCorrectionQuaternion, q).normalize();

}
