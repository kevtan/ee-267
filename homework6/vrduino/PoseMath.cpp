#include "PoseMath.h"

/**
 * TODO: see header file for documentation
 */
void convertTicksTo2DPositions(uint32_t clockTicks[8], double pos2D[8])
{
  // Compute relative times between sync pulse and sweeps
  double relativeTimes[8];
  for (int i = 0; i < 8; i++)
    relativeTimes[i] = ((double) clockTicks[i]) / CLOCKS_PER_SECOND;
  
  // Compute horizontal and vertical angles (in degrees)
  double angles[8];
  for (int i = 0; i < 8; i += 2)  // horizontal
    angles[i] = -relativeTimes[i] * 60 * 360 + 90;
  for (int i = 1; i < 8; i += 2)  // vertical
    angles[i] = relativeTimes[i] * 60 * 360 - 90;

  // Compute 2D normalized coordinates
  for (int i = 0; i < 8; i++)
    pos2D[i] = tan(radians(angles[i]));
}

/**
 * TODO: see header file for documentation
 */
void formA(double pos2D[8], double posRef[8], double Aout[8][8]) {
  for (int i = 0; i < 4; i++) {
    Aout[2 * i][0] = posRef[2 * i];
    Aout[2 * i][1] = posRef[2 * i + 1];
    Aout[2 * i][2] = 1;
    Aout[2 * i][3] = 0;
    Aout[2 * i][4] = 0;
    Aout[2 * i][5] = 0;
    Aout[2 * i][6] = -posRef[2 * i] * pos2D[2 * i];
    Aout[2 * i][7] = -posRef[2 * i + 1] * pos2D[2 * i];
    Aout[2 * i + 1][0] = 0;
    Aout[2 * i + 1][1] = 0;
    Aout[2 * i + 1][2] = 0;
    Aout[2 * i + 1][3] = posRef[2 * i];
    Aout[2 * i + 1][4] = posRef[2 * i + 1];
    Aout[2 * i + 1][5] = 1;
    Aout[2 * i + 1][6] = -posRef[2 * i] * pos2D[2 * i + 1];
    Aout[2 * i + 1][7] = -posRef[2 * i + 1] * pos2D[2 * i + 1];
  }
}


/**
 * TODO: see header file for documentation
 */
bool solveForH(double A[8][8], double b[8], double hOut[8]) {
  bool inversionSuccessful = Matrix.Invert((double *) A, 8);
  if (not inversionSuccessful) return false;
  Matrix.Multiply((double *) A, b, 8, 8, 1, hOut);
  return true;
}


/**
 * TODO: see header file for documentation
 */
void getRtFromH(double h[8], double ROut[3][3], double pos3DOut[3]) {
  // Estimate the scale factor
  double normCol1 = sqrt(sq(h[0]) + sq(h[3]) + sq(h[6]));
  double normCol2 = sqrt(sq(h[1]) + sq(h[4]) + sq(h[7]));
  double s = 2 / (normCol1 + normCol2);

  // Estimate the translation
  pos3DOut[0] = s * h[2];
  pos3DOut[1] = s * h[5];
  pos3DOut[2] = -s;

  // Estimate the rotation
  ROut[0][0] = h[0] / normCol1;
  ROut[1][0] = h[3] / normCol1;
  ROut[2][0] = -h[6] / normCol1;

  double dot = ROut[0][0] * h[1] + ROut[1][0] * h[4] - ROut[2][0] * h[7];
  ROut[0][1] = h[1] - ROut[0][0] * dot;
  ROut[1][1] = h[4] - ROut[1][0] * dot;
  ROut[2][1] = -h[7] - ROut[2][0] * dot;
  double norm = sqrt(sq(ROut[0][1]) + sq(ROut[1][1]) + sq(ROut[2][1]));
  ROut[0][1] /= norm;
  ROut[1][1] /= norm;
  ROut[2][1] /= norm;

  ROut[0][2] = ROut[1][0] * ROut[2][1] - ROut[2][0] * ROut[1][1];
  ROut[1][2] = ROut[2][0] * ROut[0][1] - ROut[0][0] * ROut[2][1];
  ROut[2][2] = ROut[0][0] * ROut[1][1] - ROut[1][0] * ROut[0][1];
}



/**
 * TODO: see header file for documentation
 */
Quaternion getQuaternionFromRotationMatrix(double R[3][3]) {
  double qW = sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2;
  double qX = (R[2][1] - R[1][2]) / (4 * qW);
  double qY = (R[0][2] - R[2][0]) / (4 * qW);
  double qZ = (R[1][0] - R[0][1]) / (4 * qW);
  return Quaternion(qW, qX, qY, qZ);
}
