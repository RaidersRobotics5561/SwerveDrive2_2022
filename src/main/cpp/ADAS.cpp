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

#include "Enums.hpp"
#include "ADAS_UT.hpp"
#include "ADAS_BT.hpp"

T_ADAS_ActiveFeature V_ADAS_ActiveFeature;

double               V_ADAS_Pct_SD_FwdRev = 0;
double               V_ADAS_Pct_SD_Strafe = 0;
double               V_ADAS_Pct_SD_Rotate = 0;
double               V_ADAS_RPM_BH_Launcher = 0;
double               V_ADAS_Pct_BH_Intake = 0;
double               V_ADAS_Pct_BH_Elevator = 0;
bool                 V_ADAS_CameraUpperLightCmndOn = false;
bool                 V_ADAS_CameraLowerLightCmndOn = false;
bool                 V_ADAS_SD_RobotOriented = false;

/******************************************************************************
 * Function:     ADAS_Main_Reset
 *
 * Description:  Reset all applicable ADAS variables.
 ******************************************************************************/
void ADAS_Main_Reset(void)
  {
  V_ADAS_ActiveFeature = E_ADAS_Disabled;
  V_ADAS_Pct_SD_FwdRev = 0;
  V_ADAS_Pct_SD_Strafe = 0;
  V_ADAS_Pct_SD_Rotate = 0;
  V_ADAS_RPM_BH_Launcher = 0;
  V_ADAS_Pct_BH_Intake = 0;
  V_ADAS_Pct_BH_Elevator = 0;
  V_ADAS_CameraUpperLightCmndOn = false;
  V_ADAS_CameraLowerLightCmndOn = false;
  V_ADAS_SD_RobotOriented = false;
  }

/******************************************************************************
 * Function:     ADAS_ControlMainTeleop
 *
 * Description:  Main calling function for the ADAS (advanced driver assistance 
 *               system)control when in teleop. This will call and manage the 
 *               various ADAS features when in teleop mode. 
 ******************************************************************************/
T_ADAS_ActiveFeature ADAS_ControlMainTeleop(double               *L_Pct_FwdRev,
                                            double               *L_Pct_Strafe,
                                            double               *L_Pct_Rotate,
                                            double               *L_RPM_Launcher,
                                            double               *L_Pct_Intake,
                                            double               *L_Pct_Elevator,
                                            bool                 *L_CameraUpperLightCmndOn,
                                            bool                 *L_CameraLowerLightCmndOn,
                                            bool                 *L_SD_RobotOriented,
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
                                            bool                  L_DriverRequestElevatorDwn,
                                            T_ADAS_ActiveFeature  L_ADAS_ActiveFeature)
  {
  T_ADAS_ActiveFeature L_ADAS_ActiveFeaturePrev = L_ADAS_ActiveFeature;

  if (L_RobotState == E_Teleop)
    {
    /* Enable criteria goes here: */
    if (L_Driver_SwerveGoalAutoCenter == true)
      {
      L_ADAS_ActiveFeature = E_ADAS_AutoUpperTarget;
      }
    else if (L_Driver_AutoIntake == true)
      {
      L_ADAS_ActiveFeature = E_ADAS_AutoBallTarget;
      }
  
    /* Abort criteria goes here: */
    if ((L_Driver1_JoystickActive == true) || (L_Driver_stops_shooter == true))
      {
      /* Abort criteria goes here. */
      L_ADAS_ActiveFeature = E_ADAS_Disabled;
      }
    }
  else if (L_RobotState == E_Auton)
    {
    L_ADAS_ActiveFeature = E_ADAS_Disabled;  // Need to create Auton!!!
    }
  else
    {
    L_ADAS_ActiveFeature = E_ADAS_Disabled;
    }


  if (L_ADAS_ActiveFeature != L_ADAS_ActiveFeaturePrev)
    {
    /* Hmm, there was a transition, let's go ahead and reset all of the variables before we start: */
    ADAS_UT_Reset();
    ADAS_BT_Reset();
    }

  switch (L_ADAS_ActiveFeature)
    {
      case E_ADAS_AutoUpperTarget:
          L_ADAS_ActiveFeature = ADAS_UT_Main(L_Pct_FwdRev,
                                              L_Pct_Strafe,
                                              L_Pct_Rotate,
                                              L_RPM_Launcher,
                                              L_Pct_Intake,
                                              L_Pct_Elevator,
                                              L_CameraUpperLightCmndOn,
                                              L_CameraLowerLightCmndOn,
                                              L_SD_RobotOriented,
                                              L_ADAS_ActiveFeature,
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
          L_ADAS_ActiveFeature = ADAS_BT_Main(L_Pct_FwdRev,
                                              L_Pct_Strafe,
                                              L_Pct_Rotate,
                                              L_RPM_Launcher,
                                              L_Pct_Intake,
                                              L_Pct_Elevator,
                                              L_CameraUpperLightCmndOn,
                                              L_CameraLowerLightCmndOn,
                                              L_SD_RobotOriented,
                                              L_ADAS_ActiveFeature,
                                              L_VisionBottomTargetAquired,
                                              L_VisionBottomYaw,
                                              L_VisionBottomTargetDistanceMeters,
                                              L_RobotState,
                                              L_BallDetected);
      break;
      case E_ADAS_Disabled:
          *L_Pct_FwdRev = 0;
          *L_Pct_Strafe = 0;
          *L_Pct_Rotate = 0;
          *L_RPM_Launcher = 0;
          *L_Pct_Intake = 0;
          *L_Pct_Elevator = 0;
          *L_CameraUpperLightCmndOn = false;
          *L_CameraLowerLightCmndOn = false;
      break;
    }
  return (L_ADAS_ActiveFeature);
  }