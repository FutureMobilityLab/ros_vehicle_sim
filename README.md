# ros_vehicle_sim
A simple package of vehicle dynamics models with ROS interfaces. The odometry that is published by the vehicle model is under the `/novatel/oem7/odom` topic name and uses the `nav_msgs/Odometry.h` message type. The vehicle model accepts velocity inputs through the `vehicle/accelerator_pedal_cmd` topic, which uses the header:`raptor_dbw_msgs/AcceleratorPedalCmd.h`. The vehicle model accepts steering wheel inputs through the `vehicle/steering_cmd` topic, which uses the header:`raptor_dbw_msgs/SteeringCmd.h`. This is so that the simulator can replace an actual vehicle equipped with a raptor dbw system and a novatel INS system.

## Single Track Model (Bicycle Car Model)
The single track model currently implemented is parameterized for a Jeep Grand Cherokee. Currently, this is implemented in the `DynamicBike` class in `bike.h`.

# Building
To build the ros package navigate to the `ros_vehicle_sim` directory and run `catkin_make`.

# Running
To run the ros package:
1. Source the setup file: `source devel/setup.bash`
2. In a separate terminal, start a ros master: `roscore`
2. Run the simulator: `rosrun ros_vehicle_sim ros_vehicle_sim_node`
