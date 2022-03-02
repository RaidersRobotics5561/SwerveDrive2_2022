/*
  ADAS_DM.cpp

  Created on: Mar 02, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems) Drive Manuvering (DM)
  Contains the logic and code used for the drive manuvering control:

  Changes:
  2022-02-25 -> Beta
 */

#if 0
#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Const.hpp"

T_ADAS_DM_DriveManuvering V_ADAS_DM_State             = E_ADAS_DM_Disabled;
double                V_ADAS_DM_DebounceTime      = 0;
double                V_ADAS_DM_RotateErrorPrev   = 0;

/* Configuration cals: */
double KV_ADAS_DM_DebounceTime;
double KV_ADAS_DM_RotateDeadbandAngle;


/******************************************************************************
 * Function:     ADAS_DM_ConfigsInit
 *
 * Description:  Contains the configurations for ADAS DM.
 ******************************************************************************/
void ADAS_DM_ConfigsInit()
  {
  // set coefficients
  KV_ADAS_UT_LightDelayTIme = K_ADAS_UT_LightDelayTIme;
  KV_ADAS_UT_LostTargetGx = K_ADAS_UT_LostTargetGx;
  KV_ADAS_UT_NoTargetError = K_ADAS_UT_NoTargetError;
  KV_ADAS_UT_DebounceTime = K_ADAS_UT_DebounceTime;
  KV_ADAS_UT_AllowedLauncherError = K_ADAS_UT_AllowedLauncherError;
  KV_ADAS_UT_AllowedLauncherTime = K_ADAS_UT_AllowedLauncherTime;
  KV_ADAS_UT_RotateDeadbandAngle = K_ADAS_UT_RotateDeadbandAngle;
  KV_ADAS_UT_TargetVisionAngle = K_ADAS_UT_TargetVisionAngle;

  #ifdef ADAS_DM_Test
  // display coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("KV_ADAS_UT_LightDelayTIme", KV_ADAS_UT_LightDelayTIme);
  frc::SmartDashboard::PutNumber("KV_ADAS_UT_LostTargetGx", KV_ADAS_UT_LostTargetGx);
  frc::SmartDashboard::PutNumber("KV_ADAS_UT_NoTargetError", KV_ADAS_UT_NoTargetError);
  frc::SmartDashboard::PutNumber("KV_ADAS_UT_DebounceTime", KV_ADAS_UT_DebounceTime);
  frc::SmartDashboard::PutNumber("KV_ADAS_UT_AllowedLauncherError", KV_ADAS_UT_AllowedLauncherError);
  frc::SmartDashboard::PutNumber("KV_ADAS_UT_AllowedLauncherTime", KV_ADAS_UT_AllowedLauncherTime);
  frc::SmartDashboard::PutNumber("KV_ADAS_UT_RotateDeadbandAngle", KV_ADAS_UT_RotateDeadbandAngle);
  frc::SmartDashboard::PutNumber("KV_ADAS_UT_TargetVisionAngle", KV_ADAS_UT_TargetVisionAngle);
  #endif
  }


/******************************************************************************
 * Function:     ADAS_DM_ConfigsCal
 *
 * Description:  Contains the configurations for ADAS DM.  This 
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void ADAS_DM_ConfigsCal()
  {
  // read coefficients from SmartDashboard
  #ifdef ADAS_DM_Test
  KV_ADAS_UT_LightDelayTIme = frc::SmartDashboard::GetNumber("KV_ADAS_UT_LightDelayTIme", KV_ADAS_UT_LightDelayTIme);
  KV_ADAS_UT_LostTargetGx = frc::SmartDashboard::GetNumber("KV_ADAS_UT_LostTargetGx", KV_ADAS_UT_LostTargetGx);
  KV_ADAS_UT_NoTargetError = frc::SmartDashboard::GetNumber("KV_ADAS_UT_NoTargetError", KV_ADAS_UT_NoTargetError);
  KV_ADAS_UT_DebounceTime = frc::SmartDashboard::GetNumber("KV_ADAS_UT_DebounceTime", KV_ADAS_UT_DebounceTime);
  KV_ADAS_UT_AllowedLauncherError = frc::SmartDashboard::GetNumber("KV_ADAS_UT_AllowedLauncherError", KV_ADAS_UT_AllowedLauncherError);
  KV_ADAS_UT_AllowedLauncherTime = frc::SmartDashboard::GetNumber("KV_ADAS_UT_AllowedLauncherTime", KV_ADAS_UT_AllowedLauncherTime);
  KV_ADAS_UT_RotateDeadbandAngle = frc::SmartDashboard::GetNumber("KV_ADAS_UT_RotateDeadbandAngle", KV_ADAS_UT_RotateDeadbandAngle);
  KV_ADAS_UT_TargetVisionAngle = frc::SmartDashboard::GetNumber("KV_ADAS_UT_TargetVisionAngle", KV_ADAS_UT_TargetVisionAngle);
  #endif
  }


/******************************************************************************
 * Function:     ADAS_DM_Reset
 *
 * Description:  Reset all applicable DM variables.
 ******************************************************************************/
void ADAS_DM_Reset(void)
  {
  V_ADAS_UT_State = E_ADAS_UT_Disabled;
  V_ADAS_UT_DebounceTime = 0;
  V_ADAS_UT_RotateErrorPrev = 0;
  V_ADAS_UT_LauncherSpeedPrev = 0;
  V_ADAS_UT_TargetAquiredPrev = false;
  }


/******************************************************************************
 * Function:     ADAS_DM_DriveStraight
 *
 * Description:  Drive straight control.
 ******************************************************************************/
T_ADAS_UT_UpperTarget ADAS_DM_DriveStraight(double       *L_Pct_FwdRev,
                                              double       *L_Pct_Strafe,
                                              double       *L_Pct_Rotate,
                                              double       *L_RPM_Launcher,
                                              double       *L_Pct_Intake,
                                              double       *L_Pct_Elevator,
                                              bool         *L_CameraUpperLightCmndOn,
                                              bool         *L_CameraLowerLightCmndOn,
                                              bool         *L_SD_RobotOriented,
                                              T_RobotState  L_RobotState,
                                              double        L_LauncherRPM_Measured,
                                              bool          L_BallDetected,
                                              bool          L_DriverRequestElevatorUp,
                                              bool          L_DriverRequestElevatorDwn)
  {
  T_ADAS_DM_DriveManuvering L_ADAS_DM_State = E_ADAS_DM_DriveStraight;
  double                L_LauncherError = 0;
  double                L_LauncherSpeedCmnd = 0;

  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;

  L_LauncherError = fabs(L_LauncherRPM_Measured - V_ADAS_UT_LauncherSpeedPrev);

  *L_Pct_FwdRev = 0;

  if (L_RobotState == E_Teleop)
    {
    /* Ok, we are in teleop.  Driver 2 will handle the triggering of the eleveator, 
       but lets use the ball detector and launcher RPM to limit the eleveator.  If 
       the launcher has dropped in RPM and the detector sees a ball, don't allow 
       the elevator to move up.  The eleveator can still be moved down if necessary.*/
    if (L_DriverRequestElevatorDwn == true)
      {
      *L_Pct_Intake = 0;
      *L_Pct_Elevator = K_ElevatorPowerDwn;
      }
    else if ((L_BallDetected == false) ||
             (L_LauncherError <= KV_ADAS_UT_AllowedLauncherError))
      {
      *L_Pct_Intake = K_IntakePower;
      *L_Pct_Elevator = K_ElevatorPowerUp;
      }
    else
      {
      *L_Pct_Intake = 0;
      *L_Pct_Elevator = 0;
      }
    }
  else
    {
    /* Ok, we are in auton.  We need to handle the triggering of the eleveator automatically.
       Lets use the ball detector and launcher RPM to limit the eleveator.  If 
       the launcher has dropped in RPM and the detector sees a ball, don't allow 
       the elevator to move up.*/
    V_ADAS_UT_DebounceTime += C_ExeTime;

    if (((L_BallDetected == false) ||
         (L_LauncherError <= KV_ADAS_UT_AllowedLauncherError)) &&

        (V_ADAS_UT_DebounceTime < KV_ADAS_UT_AllowedLauncherTime))
      {
      *L_Pct_Intake = K_IntakePower;
      *L_Pct_Elevator = K_ElevatorPowerUp;
      }
    else
      {
      *L_Pct_Intake = 0;
      *L_Pct_Elevator = 0;
      }

    if (V_ADAS_UT_DebounceTime >= KV_ADAS_UT_AllowedLauncherTime)
      {
      /* Once time has expired, exit elevator control */
      L_ADAS_UT_State = E_ADAS_UT_Disabled;
      V_ADAS_UT_DebounceTime = 0;
      }
    }

  return (L_ADAS_UT_State);
  }


/******************************************************************************
 * Function:     ADAS_UT_Main
 *
 * Description:  Manages and controls the drive manuvering (DM).
 ******************************************************************************/
T_ADAS_ActiveFeature ADAS_DM_Main(double               *L_Pct_FwdRev,
                                  double               *L_Pct_Strafe,
                                  double               *L_Pct_Rotate,
                                  double               *L_RPM_Launcher,
                                  double               *L_Pct_Intake,
                                  double               *L_Pct_Elevator,
                                  bool                 *L_CameraUpperLightCmndOn,
                                  bool                 *L_CameraLowerLightCmndOn,
                                  bool                 *L_SD_RobotOriented,
                                  T_ADAS_ActiveFeature  L_ADAS_ActiveFeature,
                                   bool                 L_VisionTopTargetAquired,
                                  double                L_TopTargetYawDegrees,
                                  double                L_VisionTopTargetDistanceMeters,
                                  T_RobotState          L_RobotState,
                                  double                L_LauncherRPM_Measured,
                                  bool                  L_BallDetected,
                                  bool                  L_DriverRequestElevatorUp,
                                  bool                  L_DriverRequestElevatorDwn,
                                  double                L_DistanceRequested,
                                  double                L_AngleRequested)
  {
    
  E_ADAS_DM_Disabled,
  E_ADAS_DM_DriveStraight,
  E_ADAS_DM_Rotate

  if (L_ADAS_ActiveFeature == E_ADAS_DriveStraight)
    {
    switch (V_ADAS_DM_State)
      {
      case E_ADAS_DM_Disabled:
      case E_ADAS_DM_DriveStraight:
          V_ADAS_DM_State = ADAS_DM_DriveStraight(L_Pct_FwdRev,
                                                    L_Pct_Strafe,
                                                    L_Pct_Rotate,
                                                    L_RPM_Launcher,
                                                    L_Pct_Intake,
                                                    L_Pct_Elevator,
                                                    L_CameraUpperLightCmndOn,
                                                    L_CameraLowerLightCmndOn,
                                                    L_SD_RobotOriented,
                                                    L_RobotState,
                                                    L_LauncherRPM_Measured,
                                                    L_BallDetected,
                                                    L_DriverRequestElevatorUp,
                                                    L_DriverRequestElevatorDwn);
      break;
      }
    }

  if (V_ADAS_UT_State == E_ADAS_UT_Disabled)
    {
    ADAS_DM_Reset();
    L_ADAS_ActiveFeature = E_ADAS_Disabled;
    }
  
  return (L_ADAS_ActiveFeature);
  }

#endif