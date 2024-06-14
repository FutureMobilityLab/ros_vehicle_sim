#include "steer_actuator.h"

#include <vector>
#include <cmath>
#include <iostream>

SteerActuator::SteerActuator() {
  steer_torque = 0.;
  velocity = 5.0;
  steer_whl_ang = 0.;
}

void SteerActuator::Advance() {
  // Use euler integration to get next output.
  // x[k+1] = x[k] + dx[k]*dt
  vector<double> dx = GetDerivative();
  for (int i=0; i<dx.size(); i++) {
    state[i] += dx[i] * step_size;
  }
  steer_whl_ang = 0.;
  for (int i=0; i<CBias.size(); i++) {
    steer_whl_ang += CBias[i] * state[i];
    steer_whl_ang += CCoeff[i] * velocity * state[i];
  }
  steer_whl_ang += D * steer_torque;
}

vector<double> SteerActuator::GetDerivative() {
  vector<double> dx = {0., 0.};
  for (int i=0; i<ABias.size(); i++) {
    for (int j=0; j<ABias[i].size(); j++) {
      dx[i] = (ABias[i][j] * state[j]) 
        + (this->ACoeff[i][j] * this->velocity * this->state[j])
        + (this->B[i] * this->steer_torque);
    }
  }
  return dx;
}

void SteerActuator::SetInputs(double steer_torque, double vel) {
  steer_torque = steer_torque;
  velocity = max(vel, 5.0);
}

double SteerActuator::GetOutputs() {
  return steer_whl_ang;
}
