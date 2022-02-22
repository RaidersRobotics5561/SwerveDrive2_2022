/*
  Driver_inputs.cpp

   Created on: Feb 05, 2022
   Author: Lauren

  Function that maps the driver inputs to the robot controls. 
 */
#include "Enums.hpp"
#include "Robot.h"

  bool                V_Driver_lift_control = false;
  bool                V_Driver_zero_gyro = false;
  bool                V_Driver_stops_shooter = false;
  bool                V_Driver_auto_setspeed_shooter = false;
  bool                V_Driver_elevator_up = false;
  bool                V_Driver_elevator_down = false;
  double              V_Driver_manual_shooter_desired_speed = 0;
  bool                V_Driver_intake_in = false;
  //bool              V_driver_intake_out = false;
  double              V_Driver_SwerveForwardBack = 0;
  double              V_Driver_SwerveStrafe = 0;
  double              V_Driver_SwerveRotate = 0;
  double              V_Driver_SwerveSpeed = 0;
  bool                V_Driver_SwerveGoalAutoCenter = false;
  bool                V_Driver_SwerveRotateTo0 = false;
  bool                V_Driver_SwerveRotateTo90 = false;
  bool                V_Driver_LiftYD_Up   = false;
  bool                V_Driver_LiftYD_Down = false;
  T_LiftCmndDirection V_Driver_Lift_Cmnd_Direction = E_LiftCmndNone;

/******************************************************************************
 * Function:     Joystick_robot_mapping
 *
 * Description:  Captures and maps driver inputs.
 ******************************************************************************/
void Joystick_robot_mapping(bool    L_Driver2_buttonA,
                            bool    L_Driver2_buttonB,
                            bool    L_Driver2_buttonRB,
                            bool    L_Driver2_buttonback,
                            bool    L_Driver2_buttonstart,
                            bool    L_Driver1_buttonback,
                            bool    L_Driver2_ButtonX,
                            double  L_Driver2_left_Axis_y,
                            double  L_Driver2_right_Axis_y,
                            double  L_Driver1_left_Axis_y,
                            double  L_Driver1_left_Axis_x,
                            double  L_Driver1_right_Axis_x,
                            double  L_Driver1_left_trigger_Axis,
                            bool    L_Driver1_buttonA,
                            bool    L_Driver1_ButtonX,
                            bool    L_Driver1_ButtonY,
                            int     L_Driver2_POV)
  {
  V_Driver_elevator_up = L_Driver2_buttonA;                       //Controller 2, A button (1), (robot.cpp) Elevator goes up
  V_Driver_elevator_down = L_Driver2_buttonB;                     //Controller 2, B button (2), (robot.cpp) Elevator goes down
  V_Driver_zero_gyro = L_Driver1_buttonback;                      //Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro  
  V_Driver_stops_shooter = L_Driver2_buttonback;                  //Controller 2 back button (7), (robot.cpp) Stops the shooter- pretty self-explain
  V_Driver_auto_setspeed_shooter = L_Driver2_buttonstart;         //controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
  V_Driver_manual_shooter_desired_speed = L_Driver2_left_Axis_y;  //Controller 2, left axis, uses y axis (1), (robot.cpp) sets desired speed for the shooter moter
  V_Driver_lift_control = L_Driver2_buttonRB,                     //Controller 2, X button (3), (Lift.cpp) starts automated states machine
  V_Driver_intake_in = L_Driver2_ButtonX;                         //Controller 2 (3), controlls the intake base on trigger pressed 
  V_Driver_SwerveForwardBack =  L_Driver1_left_Axis_y;
  V_Driver_SwerveStrafe = L_Driver1_left_Axis_x;
  V_Driver_SwerveRotate =  L_Driver1_right_Axis_x;
  V_Driver_SwerveSpeed =  L_Driver1_left_trigger_Axis;
  V_Driver_SwerveGoalAutoCenter = L_Driver1_buttonA;
  V_Driver_SwerveRotateTo0 = L_Driver1_ButtonX;
  V_Driver_SwerveRotateTo90 = L_Driver1_ButtonY;

  if (L_Driver2_POV == 0)
    {
    V_Driver_Lift_Cmnd_Direction = E_LiftCmndUp;
    }
  else if (L_Driver2_POV == 180)
    {
    V_Driver_Lift_Cmnd_Direction = E_LiftCmndDown;
    }
  else if (L_Driver2_POV == 90)
    {
    V_Driver_Lift_Cmnd_Direction = E_LiftCmndForward;
    }
  else if (L_Driver2_POV == 270)
    {
    V_Driver_Lift_Cmnd_Direction = E_LiftCmndBack;
    }
  else
    {
    V_Driver_Lift_Cmnd_Direction = E_LiftCmndNone;
    }
  }

