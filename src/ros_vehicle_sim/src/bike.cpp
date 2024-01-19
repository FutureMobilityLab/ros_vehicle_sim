// Class for a dynamic bicycle car model.

#include "bike.h"

#include <vector>
#include <cmath>
#include <string.h>


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

void DynamicBike::GetDerivative(double x[5], double u[2], double dx[5]) {
    // Compute tire forces with linear tire model.
    double alphaF = u[0]/K - atan2(x[0] + a*x[1], u[1]);
    double alphaR = -1*atan2(x[0] - b*x[1], u[1]);
    double Fyf = Cf*alphaF;
    double Fyr = Cr*alphaR;

    // Compute derivatives.
    dx[0] = (Fyf*cos(u[0]/K) + Fyr) / m - u[1]*x[1];
    dx[1] = (a*Fyf - b*Fyr) / J;
    dx[2] = u[1]*cos(x[4]) - x[0]*sin(x[4]);
    dx[3] = u[1]*sin(x[4]) + x[0]*cos(x[4]);
    dx[4] = x[1];
}

void DynamicBike::Advance() {
    // Use Runge Kutta 4th Order where inputs are zero-order held.
    double k1[5], k2[5], k3[5], k4[5];

    // Compute K1.
    GetDerivative(state, inputs, k1);

    // Compute K2.
    double intermediate_state[5];
    for (int i = 0; i < 5; i++) {
        intermediate_state[i] = state[i] + step_size*k1[i]/2;
    }
    GetDerivative(intermediate_state, inputs, k2);

    // Compute K3.
    for (int i = 0; i < 5; i++) {
        intermediate_state[i] = state[i] + step_size*k2[i]/2;
    }
    GetDerivative(intermediate_state, inputs, k3);

    // Compute k4.
    for (int i = 0; i < 5; i++) {
        intermediate_state[i] = state[i] + step_size*k3[i];
    }
    GetDerivative(intermediate_state, inputs, k4);

    // Update state.
    for (int i = 0; i < 5; i++) {
        state[i] += (step_size/6) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }
    time += step_size;
}

void DynamicBike::SetInputs(double u[2]) {
    // The tire model cannot model zero velocity behavior.
    if ( u[1] < 1.0 ) {
        u[1] = 1.0;
    }
    inputs[0] = u[0];  // Steering wheel angle [rad].
    inputs[1] = u[1];  // Longitudinal velocity [m/s].
}

void DynamicBike::SetPos(double pos[2]) {
    state[2] = pos[0];
    state[3] = pos[1];
}

void DynamicBike::SetTime(double new_time) {
    time = new_time;
}

double DynamicBike::GetInputs(int input_num) {
    return inputs[input_num];
}

void DynamicBike::GetPos(double pos[2]) {
    pos[0] = state[2]; // Global X position [m].
    pos[1] = state[3]; // Global Y position [m].
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