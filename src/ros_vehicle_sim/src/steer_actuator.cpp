#include "steer_actuator.h"

#include <vector>
#include <cmath>

SteerActuator::SteerActuator() {
  this->state = {0., 0.};
  this->steer_torque = 0.;
  this->velocity = 5.0;
  this->ACoeff = {{0., 0.}, {-3.43018, 0.490546}};
  this->ABias = {{0, 1}, {2.97703, -15.0677}};
  this->B = {0, 1};
  this->CCoeff = {-0.0729822, -1.36785e-5};
  this->CBias = {2.98067, 0.0888436};
  this->D = 0;
  this->step_size = 1e-2;
}

void SteerActuator::Advance() {
  // Use euler integration to get next output.
  // x[k+1] = x[k] + dx[k]*dt
  vector<double> dx = this->GetDerivative();
  for (int i=0; i<dx.size(); i++) {
    this->state[i] += dx[i] * this->step_size;
  }
  this->steer_whl_ang = 0.;
  for (int i=0; i<this->CBias.size(); i++) {
    this->steer_whl_ang += this->CBias[i] * this->state[i];
    this->steer_whl_ang += this->CCoeff[i] * this->velocity * this->state[i];
  }
  this->steer_whl_ang += this->D * this->steer_torque;
}

vector<double> SteerActuator::GetDerivative() {
  vector<double> dx = {0., 0.};
  for (int i=0; i<this->ABias.size(); i++) {
    for (int j=0; j<this->ABias[i].size(); i++) {
      dx[i] += this->ABias[i][j] * this->state[j];
      dx[i] += this->ACoeff[i][j] * this->velocity * this->state[j];
      dx[i] += this->B[i] * this->steer_torque;
    }
  }
  return dx;
}

void SteerActuator::SetInputs(double steer_torque, double vel) {
  this->steer_torque = steer_torque;
  this->velocity = max(vel, 5.0);
}