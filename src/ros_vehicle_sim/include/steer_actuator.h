// Class for a dynamic steering actuator model.
#ifndef _steer_actuator_h_included_
#define _steer_actuator_h_included_

#include <vector>


class SteerActuator{
  private:
    std::vector<double> state = {0., 0.};
    double steer_torque = 0.;
    double steer_whl_ang = 0.;
    double velocity = 5.0;
    const std::vector<std::vector<double>> ABias = {
      {0., 1.}, 
      {2.97703, -15.0677}
    };
    const std::vector<std::vector<double>> ACoeff = {
      {0., 0.}, 
      {-3.43018, 0.490546}
    };
    const std::vector<double> B = {0., 1.};
    const std::vector<double> CBias = {2.98067, 0.0888436};
    const std::vector<double> CCoeff = {-0.0729822, -1.36785e-5};
    const double D = 0.;
    const double step_size = 1e-3;
  public:
    SteerActuator();

    void Advance();

    std::vector<double> GetDerivative();

    void SetInputs(const double steer_torque, const double vel);

    double GetOutputs();
};

#endif
