// Class for a dynamic steering actuator model.
#ifndef _steer_actuator_h_included_
#define _steer_actuator_h_included_

#include <vector>

using namespace std;


class SteerActuator{
  private:
    vector<double> state = {0., 0.};
    double steer_torque;
    double steer_whl_ang;
    double velocity;
    const vector<vector<double>> ABias = {
      {0, 1}, 
      {2.97703, -15.0677}
    };
    const vector<vector<double>> ACoeff = {
      {0., 0.}, 
      {-3.43018, 0.490546}
    };
    const vector<double> B = {0, 1};
    const vector<double> CBias = {2.98067, 0.0888436};
    const vector<double> CCoeff = {-0.0729822, -1.36785e-5};
    const double D = 0.;
    const double step_size = 1e-2;
  public:
    SteerActuator();

    void Advance();

    vector<double> GetDerivative();

    void SetInputs(double steer_torque, double vel);

    double GetOutputs();
};

#endif
