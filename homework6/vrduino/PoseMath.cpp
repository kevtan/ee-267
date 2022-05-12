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



}


/**
 * TODO: see header file for documentation
 */
bool solveForH(double A[8][8], double b[8], double hOut[8]) {
  //use Matrix Math library for matrix operations
  //example:
  //int inv = Matrix.Invert((double*)A, 8);
  //if inverse fails (Invert returns 0), return false


  return false;

}


/**
 * TODO: see header file for documentation
 */
void getRtFromH(double h[8], double ROut[3][3], double pos3DOut[3]) {


}



/**
 * TODO: see header file for documentation
 */
Quaternion getQuaternionFromRotationMatrix(double R[3][3]) {

  return Quaternion();

}