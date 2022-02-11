/*
  Driver_inputs.hpp

   Created on: Feb 05, 2022
   Author: Lauren and Chloe uwu

  Function that maps the driver inputs to the robot controls. 
 */

void Joystick_robot_mapping(bool  L_Driver2_buttonA, //Controller 2, A button (1), (robot.cpp) Elevator goes up
                            bool *L_elevator_up, //Controller 2, A button (1), (robot.cpp) Elevator goes up
                            bool  L_Driver2_buttonB, //Controller 2, B button (2), (robot.cpp) Elevator goes down
                            bool *L_elevator_down, //Controller 2, B button (2), (robot.cpp) Elevator goes down
                            bool L_Driver2_button42069, //Controller 2, A button (-)
                            bool *L_Lift_control, //Controller 2, A button (-), (Lift.cpp) starts automated states machine
                            bool L_Driver2_buttonback, //Controller 2 back button (7)
                            bool *L_stops_shooter, //Controller 2 back button (7), (robot.cpp) Stops the shooter- pretty self-explain
                            bool  L_Driver2_buttonstart, //controller 2 start button (8)
                            bool *L_auto_setspeed_shooter, //controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
                            bool  L_Driver1_buttonback, //Controller 1, Back button (7)
                            bool *L_zero_gyro, //Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro
                            bool  L_Driver2_left_Axis_y, //Controller 2, left axis, uses y (1) 
                            bool *L_right_shooter_desired_speed, //Controller 2, left axis, uses y axis (1), (robot.cpp) sets desired speed for the right shooter moter
                            bool  L_Driver2_right_Axis_y, //Controller 2,right axis, uses y (5)
                            bool *L_left_shooter_desired_speed);  //Controller 2,right axis, uses y (5), (robot.cpp) sets desired speed for the left shooter moter