// Class for a dynamic steering actuator model.
#ifndef _steer_actuator_h_included_
#define _steer_actuator_h_included_

#include <vector>

using namespace std;

class SteerActuator{
  private:
    vector<double> state;
    double steer_torque;
    double steer_whl_ang;
    double velocity;
    vector<vector<double>> ABias;
    vector<vector<double>> ACoeff;
    vector<double> B;
    vector<double> CBias;
    vector<double> CCoeff;
    double D;
    double step_size;
  public:
    SteerActuator();

    void Advance();

    vector<double> GetDerivative();

    void SetInputs(double steer_torque, double vel);

    double GetOutputs();
};

#endif