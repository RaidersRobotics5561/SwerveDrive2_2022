/*
  ADAS.cpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems)
  Contains the logic and code used for driver assitance control.  This is meant 
  to serve as a high level controller sending commands/requests to the lower 
  level controls while also tracking and managing the various sytems. This 
  contains manages the following features:

  - Auto upper targeting
    - Turns on camera light, auto centers robot on target, spins the rollers up to the correct speed, disables camera light
  - Auto ball targeting
    - Centers robot on ball, turns on intake roller, drives froward to intake ball, exits
  - Auton Opt 1
    - More info to come

  Changes:
  2022-02-25 -> Beta
 */

#include <math.h>

#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Const.hpp"
#include "ADAS_UT.hpp"
#include "ADAS_BT.hpp"

T_ADAS_ActiveFeature V_ADAS_ActiveFeature;

/******************************************************************************
 * Function:     ADAS_ControlMainTeleop
 *
 * Description:  Main calling function for the ADAS (advanced driver assistance 
 *               system)control when in teleop. This will call and manage the 
 *               various ADAS features when in teleop mode. 
 ******************************************************************************/
void ADAS_ControlMainTeleop(double               *L_Pct_FwdRev,
                            double               *L_Pct_Strafe,
                            double               *L_Pct_Rotate,
                            double               *L_RPM_Launcher,
                            double               *L_Pct_Intake,
                            double               *L_Pct_Elevator,
                            bool                 *L_CameraLightCmndOn,
                            bool                  L_Driver1_JoystickActive,
                            bool                  L_Driver_stops_shooter,
                            bool                  L_Driver_SwerveGoalAutoCenter,
                            bool                  L_Driver_AutoIntake,
                            double                L_Deg_GyroAngleDeg,
                            bool                  L_VisionTopTargetAquired,
                            double                L_TopTargetYawDegrees,
                            double                L_VisionTopTargetDistanceMeters,
                            bool                  L_VisionBottomTargetAquired,
                            double                L_VisionBottomYaw,
                            double                L_VisionBottomTargetDistanceMeters,
                            T_RobotState          L_RobotState,
                            double                L_LauncherRPM_Measured,
                            bool                  L_BallDetected,
                            bool                  L_DriverRequestElevatorUp,
                            bool                  L_DriverRequestElevatorDwn)
  {
  T_ADAS_ActiveFeature L_ADAS_ActiveFeaturePrev = V_ADAS_ActiveFeature;

  /* Enable criteria goes here: */
  if (L_Driver_SwerveGoalAutoCenter == true)
    {
    V_ADAS_ActiveFeature = E_ADAS_AutoUpperTarget;
    }
  else if (L_Driver_AutoIntake == true)
    {
    V_ADAS_ActiveFeature = E_ADAS_AutoBallTarget;
    }

  /* Abort criteria goes here: */
  if ((L_Driver1_JoystickActive == true) || (L_Driver_stops_shooter == true))
    {
    /* Abort criteria goes here. */
    V_ADAS_ActiveFeature = E_ADAS_Disabled;
    }

  if (V_ADAS_ActiveFeature != L_ADAS_ActiveFeaturePrev)
    {
    /* Hmm, there was a transition, let's go ahead and reset all of the variables before we start: */
    ADAS_UT_Reset();
    ADAS_BT_Reset();
    }

  switch (V_ADAS_ActiveFeature)
    {
      case E_ADAS_AutoUpperTarget:
          V_ADAS_ActiveFeature = ADAS_UT_Main(L_Pct_FwdRev,
                                              L_Pct_Strafe,
                                              L_Pct_Rotate,
                                              L_RPM_Launcher,
                                              L_Pct_Intake,
                                              L_Pct_Elevator,
                                              L_CameraLightCmndOn,
                                              V_ADAS_ActiveFeature,
                                              L_VisionTopTargetAquired,
                                              L_TopTargetYawDegrees,
                                              L_VisionTopTargetDistanceMeters,
                                              L_RobotState,
                                              L_LauncherRPM_Measured,
                                              L_BallDetected,
                                              L_DriverRequestElevatorUp,
                                              L_DriverRequestElevatorDwn);
      break;
      case E_ADAS_AutoBallTarget:
          V_ADAS_ActiveFeature = ADAS_BT_Main(L_Pct_FwdRev,
                                              L_Pct_Strafe,
                                              L_Pct_Rotate,
                                              L_RPM_Launcher,
                                              L_Pct_Intake,
                                              L_Pct_Elevator,
                                              L_CameraLightCmndOn,
                                              V_ADAS_ActiveFeature,
                                              L_VisionBottomTargetAquired,
                                              L_VisionBottomYaw,
                                              L_VisionBottomTargetDistanceMeters,
                                              L_RobotState,
                                              L_BallDetected);
      break;
    }
  /* If nothing is active, don't modify the controls.  Allow the driver controls to pass through unaltered. */
  }