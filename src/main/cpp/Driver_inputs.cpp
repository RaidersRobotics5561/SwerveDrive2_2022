/*
  Driver_inputs.cpp

   Created on: Feb 05, 2022
   Author: Lauren

  Function that maps the driver inputs to the robot controls. 
 */
#include "Enums.hpp"
#include "Lookup.hpp"
#include <math.h>

  bool                V_Driver_lift_control = false;
  bool                V_Driver_zero_gyro = false;
  bool                V_Driver_StopShooterAutoClimbResetGyro = false;
  bool                V_Driver_auto_setspeed_shooter = false;
  bool                V_Driver_elevator_up = false;
  bool                V_Driver_elevator_down = false;
  double              V_Driver_manual_shooter_desired_speed = 0;
  bool                V_Driver_intake_in = false;
  bool                V_Driver_intake_out = false;
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
  bool                V_Driver_CameraLight = false;
  bool                V_Driver_AutoIntake = false;
  bool                V_Driver_JoystickActive = false; // If Driver 1 presses any of the joysticks, indicate true
  bool                V_Driver_VisionDriverModeOverride = false;
  bool                V_Driver_RobotFieldOrientedReq = false;

/******************************************************************************
 * Function:     Joystick_robot_mapping
 *
 * Description:  Captures and maps driver inputs.
 ******************************************************************************/
void Joystick_robot_mapping(bool    L_Driver2_buttonA,
                            bool    L_Driver2_buttonB,
                            bool    L_Driver2_buttonRB,
                            bool    L_Driver2_buttonLB,
                            bool    L_Driver2_buttonstart,
                            bool    L_Driver1_buttonback,
                            bool    L_Driver1_buttonstart,
                            bool    L_Driver2_ButtonX,
                            bool    L_Driver2_ButtonY,
                            double  L_Driver2_left_Axis_y,
                            double  L_Driver2_right_Axis_y,
                            double  L_Driver1_left_Axis_y,
                            double  L_Driver1_left_Axis_x,
                            double  L_Driver1_right_Axis_x,
                            double  L_Driver1_left_trigger_Axis,
                            bool    L_Driver1_buttonA,
                            bool    L_Driver1_ButtonX,
                            bool    L_Driver1_ButtonY,
                            int     L_Driver2_POV,
                            bool    L_Driver1_buttonRB,
                            bool    L_Driver1_buttonB,
                            bool    L_Driver1_ButtonLB,
                            bool    L_Driver2_buttonback)
  {
  double L_AxisToatl = 0;

  V_Driver_elevator_up = L_Driver2_buttonA;                       //Controller 2, A button (1), (robot.cpp) Elevator goes up
  V_Driver_elevator_down = L_Driver2_buttonB;                     //Controller 2, B button (2), (robot.cpp) Elevator goes down
  V_Driver_zero_gyro = (L_Driver1_buttonback || L_Driver1_buttonstart);     //Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro  
  //V_Driver_zero_gyro = (L_Driver1_buttonstart);     //Controller 2, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro, temporary only, for practice 
  //V_Driver_RobotFieldOrientedReq = L_Driver1_buttonback; // Controller 1, toggles the robot from robot oriented to field oriented (and back)
  V_Driver_StopShooterAutoClimbResetGyro = L_Driver2_buttonLB;     //Controller 2 back button (7), (robot.cpp) Stops the shooter- pretty self-explain, pauses auto climb and resets encoders in test mode
  V_Driver_auto_setspeed_shooter = L_Driver2_buttonstart;         //controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
  V_Driver_manual_shooter_desired_speed = L_Driver2_left_Axis_y;  //Controller 2, left axis, uses y axis (1), (robot.cpp) sets desired speed for the shooter moter
  V_Driver_lift_control = L_Driver2_buttonRB;                     //Controller 2, X button (3), (Lift.cpp) starts automated states machine
  V_Driver_intake_in = L_Driver2_ButtonX;                         //Controller 2 (3), controlls the intake in on trigger pressed 
  V_Driver_intake_out = L_Driver2_ButtonY;                         //Controller 2 (4), controlls the intake out on trigger pressed 
  V_Driver_SwerveForwardBack =  ScaleJoystickAxis(L_Driver1_left_Axis_y);  // Scale the axis, also used for debouncing
  V_Driver_SwerveStrafe = ScaleJoystickAxis(L_Driver1_left_Axis_x);        // Scale the axis, also used for debouncing
  V_Driver_SwerveRotate =  ScaleJoystickAxis(L_Driver1_right_Axis_x);      // Scale the axis, also used for debouncing
  V_Driver_SwerveSpeed = ScaleJoystickAxis(L_Driver1_left_trigger_Axis);  // Scale the axis, also used for debouncing
  V_Driver_SwerveGoalAutoCenter = L_Driver1_buttonA;
  V_Driver_SwerveRotateTo0 = L_Driver1_ButtonX;
  V_Driver_SwerveRotateTo90 = L_Driver1_ButtonY;
  V_Driver_CameraLight = L_Driver1_buttonRB;                      //Controller 1, X button (3), when held, turns on the camera light
  V_Driver_AutoIntake = L_Driver1_buttonB;
  V_Driver_VisionDriverModeOverride = L_Driver1_ButtonLB;

  L_AxisToatl = (fabs(V_Driver_SwerveStrafe) + fabs(V_Driver_SwerveRotate) + fabs(V_Driver_SwerveSpeed));
  
  if (L_AxisToatl > 0)
    {
    V_Driver_JoystickActive = true;
    }
  else
    {
    V_Driver_JoystickActive = false;
    }

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

