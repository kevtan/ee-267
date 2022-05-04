#include "OrientationTracker.h"

OrientationTracker::OrientationTracker(double imuFilterAlphaIn,  bool simulateImuIn) :

  imu(),
  gyr{0,0,0},
  acc{0,0,0},
  gyrBias{0,0,0},
  gyrVariance{0,0,0},
  accBias{0,0,0},
  accVariance{0,0,0},
  previousTimeImu(0),
  imuFilterAlpha(imuFilterAlphaIn),
  deltaT(0.0),
  simulateImu(simulateImuIn),
  simulateImuCounter(0),
  flatlandRollGyr(0),
  flatlandRollAcc(0),
  flatlandRollComp(0),
  quaternionGyr{1,0,0,0},
  eulerAcc{0,0,0},
  quaternionComp{1,0,0,0}

  {

}

void OrientationTracker::initImu() {
  imu.init();
}


/**
 * TODO: see documentation in header file
 */
void OrientationTracker::measureImuBiasVariance() {

  //check if imu.read() returns true
  //then read imu.gyrX, imu.accX, ...

  //compute bias, variance.
  //update:
  //gyrBias[0], gyrBias[1], gyrBias[2]
  //gyrVariance[0], gyrBias[1], gyrBias[2]
  //accBias[0], accBias[1], accBias[2]
  //accVariance[0], accBias[1], accBias[2]

  const int N_MEASUREMENTS = 1000;

  double gyrX_samples[N_MEASUREMENTS];
  double gyrY_samples[N_MEASUREMENTS];
  double gyrZ_samples[N_MEASUREMENTS];
  double accX_samples[N_MEASUREMENTS];
  double accY_samples[N_MEASUREMENTS];
  double accZ_samples[N_MEASUREMENTS];

  // Take the measurements

  for (int i = 0; i < N_MEASUREMENTS; i++) {
    while (not imu.read()) {}
    gyrX_samples[i] = imu.gyrX;
    gyrY_samples[i] = imu.gyrY;
    gyrZ_samples[i] = imu.gyrZ;
    accX_samples[i] = imu.accX;
    accY_samples[i] = imu.accY;
    accZ_samples[i] = imu.accZ;
  }

  // Compute the biases

  double gyrX_sum = 0, gyrY_sum = 0, gyrZ_sum = 0;
  double accX_sum = 0, accY_sum = 0, accZ_sum = 0;

  for (int i = 0; i < N_MEASUREMENTS; i++) {
    gyrX_sum += gyrX_samples[i];
    gyrY_sum += gyrY_samples[i];
    gyrZ_sum += gyrZ_samples[i];
    accX_sum += accX_samples[i];
    accY_sum += accY_samples[i];
    accZ_sum += accZ_samples[i];
  }

  gyrBias[0] = gyrX_sum / N_MEASUREMENTS;
  gyrBias[1] = gyrY_sum / N_MEASUREMENTS;
  gyrBias[2] = gyrZ_sum / N_MEASUREMENTS;
  accBias[0] = accX_sum / N_MEASUREMENTS;
  accBias[1] = accY_sum / N_MEASUREMENTS;
  accBias[2] = accZ_sum / N_MEASUREMENTS;

  // Compute the variances

  double gyrVarianceX_numerator = 0, gyrVarianceY_numerator = 0, gyrVarianceZ_numerator = 0;
  double accVarianceX_numerator = 0, accVarianceY_numerator = 0, accVarianceZ_numerator = 0;

  for (int i = 0; i < N_MEASUREMENTS; i++) {
    gyrVarianceX_numerator += sq(gyrX_samples[i] - gyrBias[0]);
    gyrVarianceY_numerator += sq(gyrY_samples[i] - gyrBias[1]);
    gyrVarianceZ_numerator += sq(gyrZ_samples[i] - gyrBias[2]);
    accVarianceX_numerator += sq(accX_samples[i] - accBias[0]);
    accVarianceY_numerator += sq(accY_samples[i] - accBias[1]);
    accVarianceZ_numerator += sq(accZ_samples[i] - accBias[2]);
  }

  gyrVariance[0] = gyrVarianceX_numerator / (N_MEASUREMENTS - 1);
  gyrVariance[1] = gyrVarianceY_numerator / (N_MEASUREMENTS - 1);
  gyrVariance[2] = gyrVarianceZ_numerator / (N_MEASUREMENTS - 1);
  accVariance[0] = accVarianceX_numerator / (N_MEASUREMENTS - 1);
  accVariance[1] = accVarianceY_numerator / (N_MEASUREMENTS - 1);
  accVariance[2] = accVarianceZ_numerator / (N_MEASUREMENTS - 1);

}

void OrientationTracker::setImuBias(double bias[3]) {

  for (int i = 0; i < 3; i++) {
    gyrBias[i] = bias[i];
  }

}

void OrientationTracker::resetOrientation() {

  flatlandRollGyr = 0;
  flatlandRollAcc = 0;
  flatlandRollComp = 0;
  quaternionGyr = Quaternion();
  eulerAcc[0] = 0;
  eulerAcc[1] = 0;
  eulerAcc[2] = 0;
  quaternionComp = Quaternion();

}

bool OrientationTracker::processImu() {

  if (simulateImu) {

    //get imu values from simulation
    updateImuVariablesFromSimulation();

  } else {

    //get imu values from actual sensor
    if (!updateImuVariables()) {

      //imu data not available
      return false;

    }

  }

  //run orientation tracking algorithms
  updateOrientation();

  return true;

}

void OrientationTracker::updateImuVariablesFromSimulation() {

    deltaT = 0.002;
    //get simulated imu values from external file
    for (int i = 0; i < 3; i++) {
      gyr[i] = imuData[simulateImuCounter + i];
    }
    simulateImuCounter += 3;
    for (int i = 0; i < 3; i++) {
      acc[i] = imuData[simulateImuCounter + i];
    }
    simulateImuCounter += 3;
    simulateImuCounter = simulateImuCounter % nImuSamples;

    //simulate delay
    delay(1);

}

/**
 * TODO: see documentation in header file
 */
bool OrientationTracker::updateImuVariables() {

  //sample imu values
  if (!imu.read()) {
  // return false if there's no data
    return false;
  }

  //call micros() to get current time in microseconds
  //update:
  //previousTimeImu (in seconds)
  //deltaT (in seconds)

  //read imu.gyrX, imu.accX ...
  //update:
  //gyr[0], ...
  //acc[0], ...

  // You also need to appropriately modify the update of gyr as instructed in (2.1.3).
  gyr[0] = imu.gyrX;
  gyr[1] = imu.gyrY;
  gyr[2] = imu.gyrZ;

  acc[0] = imu.accX;
  acc[1] = imu.accY;
  acc[2] = imu.accZ;

  return true;

}


/**
 * TODO: see documentation in header file
 */
void OrientationTracker::updateOrientation() {

  //call functions in OrientationMath.cpp.
  //use only class variables as arguments to functions.

  //update:
  //flatlandRollGyr
  //flatlandRollAcc
  //flatlandRollComp
  //quaternionGyr
  //eulerAcc
  //quaternionComp




}
