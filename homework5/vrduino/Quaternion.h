/**
 * Quaternion class
 *
 * We are using C++! Not JavaScript!
 * Unlike JavaScript, "this" keyword is representing a pointer!
 * If you want to access the member variable q[0], you should write
 * this->q[0].
 *
 * @copyright The Board of Trustees of the Leland Stanford Junior University
 * @version 2021/04/01
 */

#ifndef QUATERNION_H
#define QUATERNION_H

#include "Arduino.h"

class Quaternion {
public:

  /***
   * public member variables to hold the values
   *
   * Definition:
   * q = q[0] + q[1] * i + q[2] * j + q[3] * k
   */
  double q[4];


  /* Default constructor */
  Quaternion() :
    q{1.0, 0.0, 0.0, 0.0} {}


  /* Constructor with some inputs */
  Quaternion(double q0, double q1, double q2, double q3) :
    q{q0, q1, q2, q3} {}


  /* function to create another quaternion with the same values. */
  Quaternion clone() {

    return Quaternion(this->q[0], this->q[1], this->q[2], this->q[3]);

  }

  /* function to construct a quaternion from angle-axis representation. angle is given in degrees. */
  Quaternion& setFromAngleAxis(double angle, double vx, double vy, double vz) {

    double _angle = radians(angle / 2);
    
    double cosine = cos(_angle);
    double sine = sin(_angle);
    
    q[0] = cosine;
    q[1] = vx * sine;
    q[2] = vy * sine;
    q[3] = vz * sine;

    return *this;

  }

  /* function to compute the length of a quaternion */
  double length() {

    return sqrt(sq(q[0]) + sq(q[1]) + sq(q[2]) + sq(q[3]));

  }

  /* function to normalize a quaternion */
  Quaternion& normalize() {

    double norm = length();
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;

    return *this;
  }

  /* function to conjugate a quaternion */
  Quaternion& conjugate() {
    q[1] *= -1;
    q[2] *= -1;
    q[3] *= -1;
    return *this;
  }

  /* function to invert a quaternion */
  Quaternion& inverse() {

    double denominator = sq(length());
    Quaternion conjugate = clone().conjugate();
    q[0] = conjugate.q[0] / denominator;
    q[1] = conjugate.q[1] / denominator;
    q[2] = conjugate.q[2] / denominator;
    q[3] = conjugate.q[3] / denominator;

    return *this;
  }

  /* function to multiply two quaternions */
  static Quaternion multiply(Quaternion a, Quaternion b) {

    Quaternion q;

    q.q[0] = a.q[0] * b.q[0] - a.q[1] * b.q[1] - a.q[2] * b.q[2] - a.q[3] * b.q[3];
    q.q[1] = a.q[0] * b.q[1] + a.q[1] * b.q[0] + a.q[2] * b.q[3] - a.q[3] * b.q[2];
    q.q[2] = a.q[0] * b.q[2] - a.q[1] * b.q[3] + a.q[2] * b.q[0] + a.q[3] * b.q[1];
    q.q[3] = a.q[0] * b.q[3] + a.q[1] * b.q[2] - a.q[2] * b.q[1] + a.q[3] * b.q[0];

    return q;
  }

  /* function to rotate a quaternion by r * q * r^{-1} */
  Quaternion rotate(Quaternion r) {

    Quaternion r_inv = r.clone().inverse();
    return multiply(r, multiply(*this, r_inv));

  }


  /* helper function to print out a quaternion */
  void serialPrint() {
    Serial.print(q[0]);
    Serial.print(" ");
    Serial.print(q[1]);
    Serial.print(" ");
    Serial.print(q[2]);
    Serial.print(" ");
    Serial.print(q[3]);
    Serial.println();
  }
};

#endif // ifndef QUATERNION_H
