#include "steer_actuator.h"

#include <vector>
#include <cmath>
#include <iostream>

SteerActuator::SteerActuator() {
  this->steer_torque = 0.;
  this->velocity = 5.0;
  this->steer_whl_ang = 0.;
}

void SteerActuator::Advance() {
  // Use euler integration to get next output.
  // x[k+1] = x[k] + dx[k]*dt
  // y[k] = C*x[k] + D*u[k]
  //std::cout << "SteerActuator::Advance()" << std::endl;
  this->steer_whl_ang = 0.;
  for (int i=0; i<CBias.size(); i++) {
    this->steer_whl_ang += (this->CBias[i] * this->state[i])
      + (this->CCoeff[i] * this->velocity * this->state[i]);
  }
  this->steer_whl_ang += D * this->steer_torque;
  std::vector<double> dx = GetDerivative();
  for (int i=0; i<dx.size(); i++) {
    this->state[i] += dx[i] * this->step_size;
    //std::cout << state[i] << ", ";
  }
  //std::cout << std::endl;
}

std::vector<double> SteerActuator::GetDerivative() {
  std::vector<double> dx = {0., 0.};
  for (int i=0; i<ABias.size(); i++) {
    for (int j=0; j<ABias[i].size(); j++) {
      dx[i] += (ABias[i][j] * state[j]) 
        + (this->ACoeff[i][j] * this->velocity * this->state[j]);
    }
    dx[i] += (this->B[i] * this->steer_torque);
  }
  return dx;
}

void SteerActuator::SetInputs(const double steer_torque, const double vel) {
  //std::cout << "SteerActuator::SetInputs" << std::endl;
  this->steer_torque = steer_torque;
  this->velocity = std::max(vel, 5.0);
  //std::cout << this->steer_torque << ", " << this->velocity << std::endl;
}

double SteerActuator::GetOutputs() {
  //std::cout << "SteerActuator::GetOutputs" << std::endl;
  //std::cout << this->steer_whl_ang << std::endl;
  return this->steer_whl_ang;
}
