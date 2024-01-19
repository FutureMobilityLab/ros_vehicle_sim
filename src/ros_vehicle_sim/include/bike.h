// Class for a dynamic bicycle car model.
#ifndef _bike_h_included_
#define _bike_h_included_

#include <vector>
#include <cmath>
#include <string.h>


class DynamicBike {
    public:
        // Constructor.
        DynamicBike( 
            double dist_to_front_axle, // distance from C.G. to front axle [m]
            double dist_to_rear_axle, // distance from C.G. to rear axle [m]
            double mass, // vehicle mass [Kg]
            double moment_inertia, // vehicle yaw moment of inertia [kg*m^2]
            double front_stiffness, // Front axle cornering stiffness [N/rad]
            double rear_stiffness, // Rear axle cornering stiffness [N/rad]
            double steer_ratio ); // Steering Gear Ratio

        // Compute derivative of dynamic bicycle car model.
        void GetDerivative(double x[5], double u[2], double dx[5]);
        
        // Perform integration step.
        void Advance();

        // Set inputs.
        void SetInputs(double u[2]);

        // Set position. The first index should be global x, the second
        // should be global y in meters.
        void SetPos(double pos[2]);

        // Set time.
        void SetTime(double new_time);

        // Get one of the inputs.
        // 0: steering wheel angle [rad]
        // 1: longitudinal velocity [m/s]
        double GetInputs(int input_num);

        // Get position.
        void GetPos(double pos[2]);

        // Get orientation.
        double GetYaw();

        // Get linear velocity.
        void GetPosDt(double pos_dt[2]);

        // Get rotational velocity.
        double GetYawRate();

        // Get simulation time.
        double GetTime();

    private:
        double a;
        double b;
        double m;
        double J;
        double Cf;
        double Cr;
        double K;
        double inputs[2] = {0.0, 1.0};
        double state[5];
        double time;
        double step_size = 1e-3;
};
#endif