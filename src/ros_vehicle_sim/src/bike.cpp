// Class for a dynamic bicycle car model.

#include "bike.h"

#include <vector>
#include <cmath>
#include <string.h>
#include <iostream>


DynamicBike::DynamicBike( 
            double dist_to_front_axle, // distance from C.G. to front axle [m]
            double dist_to_rear_axle, // distance from C.G. to rear axle [m]
            double mass, // vehicle mass [Kg]
            double moment_inertia, // vehicle yaw moment of inertia [kg*m^2]
            double front_stiffness, // Front axle cornering stiffness [N/rad]
            double rear_stiffness, // Rear axle cornering stiffness [N/rad]
            double steer_ratio )
    : a { dist_to_front_axle }
    , b { dist_to_rear_axle }
    , m { mass }
    , J { moment_inertia }
    , Cf { front_stiffness }
    , Cr { rear_stiffness }
    , K { steer_ratio }
{
}

std::vector<double> DynamicBike::GetDerivative() {
    // Compute tire forces with linear tire model.
    // std::cout << "GetDerivative" << std::endl;
    // for (int i=0; i<5;i++) {
    //     std::cout << this->state[i] << ", ";
    // }
    // std::cout << std::endl;
    double alphaF = this->inputs[0]/K - std::atan2(this->state[0] + a*this->state[1],
        this->inputs[1]);
    double alphaR = -1*std::atan2(this->state[0] - b*this->state[1], this->inputs[1]);
    double Fyf = Cf*alphaF;
    double Fyr = Cr*alphaR;
    //std::cout << "Inputs" << std::endl;
    //std::cout << this->inputs[0] << ", " << this->inputs[1] << std::endl;
    //std::cout << alphaF << ", " << alphaR << ", " << Fyf << ", " << Fyr << std::endl;

    // Compute derivatives.
    std::vector<double> dx = {0, 0, 0, 0, 0};
    dx[0] = (Fyf*cos(this->inputs[0]/K) + Fyr) / m - this->inputs[1]*this->state[1];
    dx[1] = (a*Fyf - b*Fyr) / J;
    dx[2] = this->inputs[1]*cos(this->state[4]) - this->state[0]*sin(this->state[4]);
    dx[3] = this->inputs[1]*sin(this->state[4]) + this->state[0]*cos(this->state[4]);
    dx[4] = this->state[1];
    // std::cout << "Derivatives: " << std::endl;
    // for (auto i: dx ) {
    //     std::cout << i << ", ";
    // }
    // std::cout << std::endl;
    return dx;
}

// void DynamicBike::Advance() {
//     // Use Runge Kutta 4th Order where inputs are zero-order held.
//     double k1[5], k2[5], k3[5], k4[5];

//     // Compute K1.
//     GetDerivative(state, inputs, k1);

//     // Compute K2.
//     double intermediate_state[5];
//     for (int i = 0; i < 5; i++) {
//         intermediate_state[i] = state[i] + step_size*k1[i]/2;
//     }
//     GetDerivative(intermediate_state, inputs, k2);

//     // Compute K3.
//     for (int i = 0; i < 5; i++) {
//         intermediate_state[i] = state[i] + step_size*k2[i]/2;
//     }
//     GetDerivative(intermediate_state, inputs, k3);

//     // Compute k4.
//     for (int i = 0; i < 5; i++) {
//         intermediate_state[i] = state[i] + step_size*k3[i];
//     }
//     GetDerivative(intermediate_state, inputs, k4);

//     // Update state.
//     for (int i = 0; i < 5; i++) {
//         state[i] += (step_size/6) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
//     }
//     time += step_size;
// }

void DynamicBike::Advance() {
    //double dx[5] = {0,0,0,0,0};
    std::vector<double> dx = this->GetDerivative();

    for (int i=0; i<5; i++) {
        //std::cout << dx[i] << ": ";
        this->state[i] += dx[i]*this->step_size;
        //std::cout << this->state[i] << std::endl;
    }
    this->time += step_size;
    //std::cout << "exiting Advance" << std::endl;
}

void DynamicBike::SetInputs(const double u[2]) {
    // The tire model cannot model zero velocity behavior.
    inputs[0] = u[0];  // Steering wheel angle [rad].
    inputs[1] = std::max(u[1], 1.0);  // Longitudinal velocity [m/s].
}

void DynamicBike::SetPos(const double pos[2]) {
    state[2] = pos[0];
    state[3] = pos[1];
}

void DynamicBike::SetYaw(const double yaw) {
    state[4] = yaw;
}

void DynamicBike::SetTime(const double new_time) {
    time = new_time;
}

double DynamicBike::GetInputs(int input_num) {
    return inputs[input_num];
}

std::vector<double> DynamicBike::GetPos() {
    return std::vector<double> {state[2], state[3]};
    // pos[0] = state[2]; // Global X position [m].
    // pos[1] = state[3]; // Global Y position [m].
}

double DynamicBike::GetYaw() {
    return state[4]; // Yaw angle [rad].
}

void DynamicBike::GetPosDt(double pos_dt[2]) {
    pos_dt[0] = inputs[1]; // Longitudinal Velocity [m/s].
    pos_dt[1] = state[0];  // Lateral Velocity [m/s].
}

double DynamicBike::GetYawRate() {
    return state[1]; // Yaw Rate [rad/s].
}

double DynamicBike::GetTime() {
    return time;
}