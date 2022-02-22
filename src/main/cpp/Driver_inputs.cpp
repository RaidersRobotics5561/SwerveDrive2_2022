/*
  Driver_inputs.cpp

   Created on: Feb 05, 2022
   Author: Lauren

  Function that maps the driver inputs to the robot controls. 
 */
#include "Enums.hpp"

/******************************************************************************
 * Function:     Joystick_robot_mapping
 *
 * Description:  Captures and maps driver inputs.
 ******************************************************************************/

void Joystick_robot_mapping(bool  L_Driver2_buttonA,
                            bool *L_elevator_up,    
                            bool  L_Driver2_buttonB,
                            bool *L_elevator_down,
                            bool  L_Driver2_buttonRB,
                            bool *L_Lift_control,
                            bool  L_Driver2_buttonback,
                            bool *L_stops_shooter,
                            bool  L_Driver2_buttonstart,
                            bool *L_auto_setspeed_shooter,
                            bool  L_Driver1_buttonback,
                            bool *L_zero_gyro,
                            bool  L_Driver2_ButtonX,
                            bool *L_Driver_intake_in,
                            double  L_Driver2_left_Axis_y,
                            double *L_right_shooter_desired_speed,
                            double  L_Driver2_right_Axis_y,
                            double *L_left_shooter_desired_speed,
                            double  L_Driver1_left_Axis_y,
                            double *L_Driver_SwerveForwardBack,
                            double  L_Driver1_left_Axis_x,
                            double *L_Driver_SwerveStrafe,
                            double  L_Driver1_right_Axis_x,
                            double *L_Driver_SwerveRotate,
                            double  L_Driver1_left_trigger_Axis,
                            double *L_Driver_SwerveSpeed,
                            bool    L_Driver1_buttonA,
                            bool   *L_Driver_SwerveGoalAutoCenter,
                            bool    L_Driver1_ButtonX,
                            bool   *L_Driver_SwerveRotateTo0,
                            bool    L_Driver1_ButtonY,
                            bool   *L_Driver_SwerveRotateTo90,
                            int     L_Driver2_POV,
                            T_LiftCmndDirection *L_DriverLiftCmndDirection)
{
    *L_elevator_up = L_Driver2_buttonA;
    *L_elevator_down = L_Driver2_buttonB;
    *L_zero_gyro = L_Driver1_buttonback;
    *L_stops_shooter = L_Driver2_buttonback;
    *L_auto_setspeed_shooter = L_Driver2_buttonstart;
    *L_right_shooter_desired_speed = L_Driver2_left_Axis_y;
    *L_left_shooter_desired_speed = L_Driver2_right_Axis_y;
    *L_Lift_control = L_Driver2_buttonRB,
    *L_Driver_intake_in = L_Driver2_ButtonX;

    *L_Driver_SwerveForwardBack =  L_Driver1_left_Axis_y;
    *L_Driver_SwerveStrafe = L_Driver1_left_Axis_x;
    *L_Driver_SwerveRotate =  L_Driver1_right_Axis_x;
    *L_Driver_SwerveSpeed =  L_Driver1_left_trigger_Axis;
    *L_Driver_SwerveGoalAutoCenter = L_Driver1_buttonA;
    *L_Driver_SwerveRotateTo0 = L_Driver1_ButtonX;
    *L_Driver_SwerveRotateTo90 = L_Driver1_ButtonY;

    if (L_Driver2_POV == 0)
      {
      *L_DriverLiftCmndDirection = E_LiftCmndUp;
      }
    else if (L_Driver2_POV == 180)
      {
      *L_DriverLiftCmndDirection = E_LiftCmndDown;
      }
    else if (L_Driver2_POV == 90)
      {
      *L_DriverLiftCmndDirection = E_LiftCmndForward;
      }
    else if (L_Driver2_POV == 270)
      {
      *L_DriverLiftCmndDirection = E_LiftCmndBack;
      }
}

