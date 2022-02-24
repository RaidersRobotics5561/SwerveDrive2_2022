/*
  Driver_inputs.hpp

   Created on: Feb 05, 2022
   Author: Lauren and Chloe uwu

  Function that maps the driver inputs to the robot controls. 
 */

 extern bool                V_Driver_lift_control;
 extern bool                V_Driver_zero_gyro;
 extern bool                V_Driver_stops_shooter;
 extern bool                V_Driver_auto_setspeed_shooter;
 extern bool                V_Driver_elevator_up;
 extern bool                V_Driver_elevator_down;
 extern double              V_Driver_manual_shooter_desired_speed;
 extern bool                V_Driver_intake_in;
 extern bool                V_Driver_intake_out;
 extern double              V_Driver_SwerveForwardBack;
 extern double              V_Driver_SwerveStrafe;
 extern double              V_Driver_SwerveRotate;
 extern double              V_Driver_SwerveSpeed;
 extern bool                V_Driver_SwerveGoalAutoCenter;
 extern bool                V_Driver_SwerveRotateTo0;
 extern bool                V_Driver_SwerveRotateTo90;
 extern bool                V_Driver_LiftYD_Up;
 extern bool                V_Driver_LiftYD_Down;
 extern T_LiftCmndDirection V_Driver_Lift_Cmnd_Direction;
 extern bool                V_Driver_CameraLight;
 extern bool                V_Driver_AutoIntake;

void Joystick_robot_mapping(bool  L_Driver2_buttonA, //Controller 2, A button (1), (robot.cpp) Elevator goes up
                            bool  L_Driver2_buttonB, //Controller 2, B button (2), (robot.cpp) Elevator goes down
                            bool L_Driver2_buttonRB, //Controller 2, X button (3)
                            bool L_Driver2_buttonback, //Controller 2 back button (7)
                            bool  L_Driver2_buttonstart, //controller 2 start button (8)
                            bool  L_Driver1_buttonback, //Controller 1, Back button (7)
                            bool    L_Driver1_buttonstart, //Controller 1, start button (8)
                            bool  L_Driver2_ButtonX,  //Controller 2 (3), controls the intake in on button pressed 
                            bool    L_Driver2_ButtonY, // Controller 2 (4), controls the intake out on button pressed (inverse intake)
                            double  L_Driver2_left_Axis_y, //Controller 2, left axis, uses y (1) 
                            double  L_Driver2_right_Axis_y, //Controller 2,right axis, uses y (5)
                            double  L_Driver1_left_Axis_y,
                            double  L_Driver1_left_Axis_x,
                            double  L_Driver1_right_Axis_x,
                            double  L_Driver1_left_trigger_Axis,
                            bool    L_Driver1_buttonA,
                            bool    L_Driver1_ButtonX,
                            bool    L_Driver1_ButtonY,
                            int     L_Driver2_POV,
                            bool    L_Driver1_buttonRB,
                            bool    L_Driver1_buttonB);
 