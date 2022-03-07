/*
  ADAS_DM.cpp

  Created on: Mar 02, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems) Drive Manuvering (DM)
  Contains the logic and code used for the drive manuvering control.
  As of 03-02, this will just blindly launch the ball and drive forward 
  for a set amount of time.

  Changes:
  2022-03-02 -> Beta, not tested....
 */

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Const.hpp"

T_ADAS_DM_DriveManuvering V_ADAS_DM_State             = E_ADAS_DM_Disabled;
double                    V_ADAS_DM_DebounceTime      = 0;
double                    V_ADAS_DM_AutonSelectionRaw;

/* Configuration cals: */
// double KV_ADAS_DM_DebounceTime;
// double KV_ADAS_DM_RotateDeadbandAngle;


/******************************************************************************
 * Function:     ADAS_DM_ConfigsInit
 *
 * Description:  Contains the configurations for ADAS DM.
 ******************************************************************************/
void ADAS_DM_ConfigsInit()
  {

  frc::SmartDashboard::PutNumber("Auton Selection (DM)", V_ADAS_DM_AutonSelectionRaw);

    
  // set coefficients
  // KV_ADAS_UT_LightDelayTIme = K_ADAS_UT_LightDelayTIme;
  // KV_ADAS_UT_LostTargetGx = K_ADAS_UT_LostTargetGx;
  // KV_ADAS_UT_NoTargetError = K_ADAS_UT_NoTargetError;
  // KV_ADAS_UT_DebounceTime = K_ADAS_UT_DebounceTime;
  // KV_ADAS_UT_AllowedLauncherError = K_ADAS_UT_AllowedLauncherError;
  // KV_ADAS_UT_AllowedLauncherTime = K_ADAS_UT_AllowedLauncherTime;
  // KV_ADAS_UT_RotateDeadbandAngle = K_ADAS_UT_RotateDeadbandAngle;
  // KV_ADAS_UT_TargetVisionAngle = K_ADAS_UT_TargetVisionAngle;

  // #ifdef ADAS_DM_Test
  // // display coefficients on SmartDashboard
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_LightDelayTIme", KV_ADAS_UT_LightDelayTIme);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_LostTargetGx", KV_ADAS_UT_LostTargetGx);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_NoTargetError", KV_ADAS_UT_NoTargetError);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_DebounceTime", KV_ADAS_UT_DebounceTime);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_AllowedLauncherError", KV_ADAS_UT_AllowedLauncherError);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_AllowedLauncherTime", KV_ADAS_UT_AllowedLauncherTime);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_RotateDeadbandAngle", KV_ADAS_UT_RotateDeadbandAngle);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_TargetVisionAngle", KV_ADAS_UT_TargetVisionAngle);
  // #endif
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
  // KV_ADAS_UT_LightDelayTIme = frc::SmartDashboard::GetNumber("KV_ADAS_UT_LightDelayTIme", KV_ADAS_UT_LightDelayTIme);
  // KV_ADAS_UT_LostTargetGx = frc::SmartDashboard::GetNumber("KV_ADAS_UT_LostTargetGx", KV_ADAS_UT_LostTargetGx);
  // KV_ADAS_UT_NoTargetError = frc::SmartDashboard::GetNumber("KV_ADAS_UT_NoTargetError", KV_ADAS_UT_NoTargetError);
  // KV_ADAS_UT_DebounceTime = frc::SmartDashboard::GetNumber("KV_ADAS_UT_DebounceTime", KV_ADAS_UT_DebounceTime);
  // KV_ADAS_UT_AllowedLauncherError = frc::SmartDashboard::GetNumber("KV_ADAS_UT_AllowedLauncherError", KV_ADAS_UT_AllowedLauncherError);
  // KV_ADAS_UT_AllowedLauncherTime = frc::SmartDashboard::GetNumber("KV_ADAS_UT_AllowedLauncherTime", KV_ADAS_UT_AllowedLauncherTime);
  // KV_ADAS_UT_RotateDeadbandAngle = frc::SmartDashboard::GetNumber("KV_ADAS_UT_RotateDeadbandAngle", KV_ADAS_UT_RotateDeadbandAngle);
  // KV_ADAS_UT_TargetVisionAngle = frc::SmartDashboard::GetNumber("KV_ADAS_UT_TargetVisionAngle", KV_ADAS_UT_TargetVisionAngle);
  #endif
  }


/******************************************************************************
 * Function:     ADAS_DM_Reset
 *
 * Description:  Reset all applicable DM variables.
 ******************************************************************************/
void ADAS_DM_Reset(void)
  {
  V_ADAS_DM_AutonSelectionRaw = frc::SmartDashboard::GetNumber("Blind Shot Auton Selection", V_ADAS_DM_AutonSelectionRaw);

  if (V_ADAS_DM_AutonSelectionRaw )

  

  V_ADAS_DM_State             = E_ADAS_DM_Disabled;
  V_ADAS_DM_DebounceTime      = 0;
  }


/******************************************************************************
 * Function:     ADAS_DM_DriveStraight
 *
 * Description:  Drive straight control.
 ******************************************************************************/
T_ADAS_DM_DriveManuvering ADAS_DM_DriveStraight(double     *L_Pct_FwdRev,
                                              double       *L_Pct_Strafe,
                                              double       *L_Pct_Rotate,
                                              double       *L_RPM_Launcher,
                                              double       *L_Pct_Intake,
                                              double       *L_Pct_Elevator,
                                              bool         *L_CameraUpperLightCmndOn,
                                              bool         *L_CameraLowerLightCmndOn,
                                              bool         *L_SD_RobotOriented)
  {
  T_ADAS_DM_DriveManuvering L_ADAS_DM_State = E_ADAS_DM_DriveStraight;

  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  V_ADAS_DM_DebounceTime += C_ExeTime;

  if (V_ADAS_DM_DebounceTime <= K_ADAS_DM_DriveTime)
    {
    *L_Pct_FwdRev = K_ADAS_DM_DriveFWD_Pct;
    }
  else
    {
    *L_Pct_FwdRev = 0;
    *L_SD_RobotOriented = false;
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_State = E_ADAS_DM_Disabled;
    }
  return (L_ADAS_DM_State);
  }


/******************************************************************************
 * Function:     ADAS_DM_BlindShot
 *
 * Description:  Blindly shoot the ball at a prescribed speed.
 ******************************************************************************/
T_ADAS_DM_DriveManuvering ADAS_DM_BlindShot(double       *L_Pct_FwdRev,
                                            double       *L_Pct_Strafe,
                                            double       *L_Pct_Rotate,
                                            double       *L_RPM_Launcher,
                                            double       *L_Pct_Intake,
                                            double       *L_Pct_Elevator,
                                            bool         *L_CameraUpperLightCmndOn,
                                            bool         *L_CameraLowerLightCmndOn,
                                            bool         *L_SD_RobotOriented,
                                            double        L_ADAS_DM_AutonSelection)
  {
  T_ADAS_DM_DriveManuvering L_ADAS_DM_State = E_ADAS_DM_BlindLaunch;
  double L_ADAS_DM_ShooterSpeed;


  *L_SD_RobotOriented = false;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;
  *L_Pct_FwdRev = 0;

  V_ADAS_DM_DebounceTime += C_ExeTime;

  // if (fabs(L_ADAS_DM_AutonSelection) < 1.5){
  //   L_ADAS_DM_ShooterSpeed = K_ADAS_DM_BlindShotLauncherLow;

  // }
  // else{
  //   L_ADAS_DM_ShooterSpeed = K_ADAS_DM_BlindShotLauncherHigh;
  // }


  if (V_ADAS_DM_DebounceTime <= K_ADAS_DM_BlindShotTime)
    {
      
    *L_Pct_Elevator = K_ADAS_DM_BlindShotElevator;
    *L_RPM_Launcher = K_ADAS_DM_BlindShotLauncherLow;
    }
  else
    {
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_State = E_ADAS_DM_DriveStraight;
    *L_Pct_Elevator = 0;
    *L_RPM_Launcher = 0;
    }
  return (L_ADAS_DM_State);
  }


/******************************************************************************
 * Function:     ADAS_DM_Main
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
                                  bool                 *L_VisionTargetingRequest,
                                  T_ADAS_ActiveFeature  L_ADAS_ActiveFeature,
                                   bool                 L_VisionTopTargetAquired,
                                  double                L_TopTargetYawDegrees,
                                  double                L_VisionTopTargetDistanceMeters,
                                  T_RobotState          L_RobotState,
                                  double                L_LauncherRPM_Measured,
                                  bool                  L_BallDetected,
                                  bool                  L_DriverRequestElevatorUp,
                                  bool                  L_DriverRequestElevatorDwn)
  {

  if (L_ADAS_ActiveFeature == E_ADAS_DriveAndShootBlind1)
    {
    switch (V_ADAS_DM_State)
      {
      case E_ADAS_DM_Disabled:
      case E_ADAS_DM_BlindLaunch:
          V_ADAS_DM_State = ADAS_DM_BlindShot(L_Pct_FwdRev,
                                                    L_Pct_Strafe,
                                                    L_Pct_Rotate,
                                                    L_RPM_Launcher,
                                                    L_Pct_Intake,
                                                    L_Pct_Elevator,
                                                    L_CameraUpperLightCmndOn,
                                                    L_CameraLowerLightCmndOn,
                                                    L_SD_RobotOriented,
                                                    V_ADAS_DM_AutonSelectionRaw);
      break;
      case E_ADAS_DM_DriveStraight:
          V_ADAS_DM_State = ADAS_DM_DriveStraight(L_Pct_FwdRev,
                                                    L_Pct_Strafe,
                                                    L_Pct_Rotate,
                                                    L_RPM_Launcher,
                                                    L_Pct_Intake,
                                                    L_Pct_Elevator,
                                                    L_CameraUpperLightCmndOn,
                                                    L_CameraLowerLightCmndOn,
                                                    L_SD_RobotOriented);
      break;
      }
    }

  if (V_ADAS_DM_State == E_ADAS_DM_Disabled)
    {
    ADAS_DM_Reset();
    L_ADAS_ActiveFeature = E_ADAS_Disabled;
    }
  
  return (L_ADAS_ActiveFeature);
  }
