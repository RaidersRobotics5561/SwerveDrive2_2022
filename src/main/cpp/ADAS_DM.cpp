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

double                    V_ADAS_DM_DebounceTime      = 0;
bool                      V_ADAS_DM_StateInit = false;
double                    V_ADAS_DM_Rotate180TargetAngle = 0;

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
  V_ADAS_DM_DebounceTime      = 0;
  V_ADAS_DM_StateInit = false;
  V_ADAS_DM_Rotate180TargetAngle = 0;
  }


/******************************************************************************
 * Function:     ADAS_DM_Rotate180
 *
 * Description:  Rotate 180 degrees control.
 ******************************************************************************/
bool ADAS_DM_Rotate180(double     *L_Pct_FwdRev,
                       double     *L_Pct_Strafe,
                       double     *L_Pct_Rotate,
                       double     *L_RPM_Launcher,
                       double     *L_Pct_Intake,
                       double     *L_Pct_Elevator,
                       bool       *L_CameraUpperLightCmndOn,
                       bool       *L_CameraLowerLightCmndOn,
                       bool       *L_SD_RobotOriented,
                       double      L_Deg_GyroAngleDeg)
  {
  bool L_ADAS_DM_StateComplete = false;
  double L_RotateError = 0;

  *L_SD_RobotOriented = false;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  if (V_ADAS_DM_StateInit == false)
    {
    /* Need to find the target angle.  The gyro in use will only report values of -180 to 180. Need to account for this:*/
    V_ADAS_DM_Rotate180TargetAngle = L_Deg_GyroAngleDeg - 180;

    // V_ADAS_DM_Rotate180TargetAngle = std::fmod((V_ADAS_DM_Rotate180TargetAngle), 180);

    if (V_ADAS_DM_Rotate180TargetAngle >= 180)
      {
      V_ADAS_DM_Rotate180TargetAngle -= 360;
      }
    else if (V_ADAS_DM_Rotate180TargetAngle <= -180)
      {
      V_ADAS_DM_Rotate180TargetAngle += 360;
      }

    V_ADAS_DM_StateInit = true;
    }

  L_RotateError = V_ADAS_DM_Rotate180TargetAngle - L_Deg_GyroAngleDeg;

    if (fabs(L_RotateError) <= K_ADAS_DM_RotateDeadbandAngle && V_ADAS_DM_DebounceTime < K_ADAS_DM_RotateDebounceTime)
    {
    V_ADAS_DM_DebounceTime += C_ExeTime;
    }
  else if (fabs(L_RotateError) > K_ADAS_DM_RotateDeadbandAngle)
    {
    /* Reset the timer, we have gone out of bounds */
    V_ADAS_DM_DebounceTime = 0;
    }
  else if (V_ADAS_DM_DebounceTime >= K_ADAS_DM_RotateDebounceTime)
    {
    /* Reset the time, proceed to next state. */
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_DebounceTime = 0;
    }

  if (L_ADAS_DM_StateComplete == false)
    {
    *L_Pct_Rotate = DesiredRotateSpeed(L_RotateError);
    }
  else
    {
    /* We have been at the correct location for the set amount of time.
       We have previously set the state to the next one, now set the rotate command to off. */
    *L_Pct_Rotate = 0;
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_StateInit = false;
    }

  return (L_ADAS_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_DriveStraight
 *
 * Description:  Drive straight control.
 ******************************************************************************/
bool ADAS_DM_DriveStraight(double     *L_Pct_FwdRev,
                           double     *L_Pct_Strafe,
                           double     *L_Pct_Rotate,
                           double     *L_RPM_Launcher,
                           double     *L_Pct_Intake,
                           double     *L_Pct_Elevator,
                           bool       *L_CameraUpperLightCmndOn,
                           bool       *L_CameraLowerLightCmndOn,
                           bool       *L_SD_RobotOriented)
  {
  bool L_ADAS_DM_StateComplete = false;

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
    L_ADAS_DM_StateComplete = true;
    }
  return (L_ADAS_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_ReverseAndIntake
 *
 * Description:  Drive in reverse and turn on intake.
 ******************************************************************************/
bool ADAS_DM_ReverseAndIntake(double     *L_Pct_FwdRev,
                              double     *L_Pct_Strafe,
                              double     *L_Pct_Rotate,
                              double     *L_RPM_Launcher,
                              double     *L_Pct_Intake,
                              double     *L_Pct_Elevator,
                              bool       *L_CameraUpperLightCmndOn,
                              bool       *L_CameraLowerLightCmndOn,
                              bool       *L_SD_RobotOriented)
  {
  bool L_ADAS_DM_StateComplete = false;

  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = K_ADAS_DM_BlindShotIntake;
  *L_Pct_Elevator = 0; // Elevator should automatically enable when necessary when intake is commanded on

  V_ADAS_DM_DebounceTime += C_ExeTime;

  if (V_ADAS_DM_DebounceTime <= K_ADAS_DM_DriveTime)
    {
    *L_Pct_FwdRev = K_ADAS_DM_DriveREV_Pct;
    }
  else
    {
    *L_Pct_FwdRev = 0;
    *L_SD_RobotOriented = false;
    *L_Pct_Intake = 0;
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    }
  return (L_ADAS_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_BlindShot
 *
 * Description:  Blindly shoot the ball at a prescribed speed.
 ******************************************************************************/
bool ADAS_DM_BlindShot(double       *L_Pct_FwdRev,
                       double       *L_Pct_Strafe,
                       double       *L_Pct_Rotate,
                       double       *L_RPM_Launcher,
                       double       *L_Pct_Intake,
                       double       *L_Pct_Elevator,
                       bool         *L_CameraUpperLightCmndOn,
                       bool         *L_CameraLowerLightCmndOn,
                       bool         *L_SD_RobotOriented)
  {
  bool L_ADAS_DM_StateComplete = false;
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

  if (V_ADAS_DM_DebounceTime <= K_ADAS_DM_BlindShotTime)
    {
    *L_Pct_Intake = K_ADAS_DM_BlindShotElevator;
    *L_Pct_Elevator = K_ADAS_DM_BlindShotElevator;
    *L_RPM_Launcher = K_ADAS_DM_BlindShotLauncherHigh;
    }
  else
    {
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    *L_Pct_Intake = 0;
    *L_Pct_Elevator = 0;
    *L_RPM_Launcher = 0;
    }
  return (L_ADAS_DM_StateComplete);
  }
