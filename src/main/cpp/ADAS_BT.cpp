/*
  ADAS_BT.cpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems) Ball Targeting
  Contains the logic and code used for the ball targeting control:
    - Auto centers robot on ball, enables intake, drives forward to intake ball, disables intake

  Changes:
  2022-02-25 -> Beta
 */

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Const.hpp"

T_ADAS_BT_BallTarget V_ADAS_BT_State             = E_ADAS_BT_Disabled;
double               V_ADAS_BT_DebounceTime      = 0;
double               V_ADAS_BT_RotateErrorPrev   = 0;
bool                 V_ADAS_BT_TargetAquiredPrev = false;
bool                 V_ADAS_BT_DriveForwardInitiated = false;
double               V_ADAS_BT_DriveForwardTime     = 0;

double KV_ADAS_BT_LightDelayTIme;
double KV_ADAS_BT_LostTargetGx;
double KV_ADAS_BT_NoTargetError;
double KV_ADAS_BT_DebounceTime;
double KV_ADAS_BT_RotateDeadbandAngle;
double KV_ADAS_BT_TargetVisionAngle;
double KV_ADAS_BT_DriveTimeAxis[6];
double KV_ADAS_BT_DriveTime[6];
double KV_ADAS_BT_MaxTimeToWaitForCamera;
double KV_ADAS_BT_TimedOutDriveForward;
double KV_ADAS_BT_DriveForwardPct;

/******************************************************************************
 * Function:     ADAS_BT_ConfigsInit
 *
 * Description:  Contains the configurations for the UT.
 ******************************************************************************/
void ADAS_BT_ConfigsInit()
  {
  int L_Index = 0;
  // set coefficients
  for (L_Index = 0;
       L_Index < 6;
       L_Index += 1)
      {
      KV_ADAS_BT_DriveTimeAxis[L_Index] = K_ADAS_BT_DriveTimeAxis[L_Index];
      KV_ADAS_BT_DriveTime[L_Index] = K_ADAS_BT_DriveTime[L_Index];
      }

  KV_ADAS_BT_LightDelayTIme = K_ADAS_BT_LightDelayTIme;
  KV_ADAS_BT_LostTargetGx = K_ADAS_BT_LostTargetGx;
  KV_ADAS_BT_NoTargetError = K_ADAS_BT_NoTargetError;
  KV_ADAS_BT_DebounceTime = K_ADAS_BT_DebounceTime;
  KV_ADAS_BT_RotateDeadbandAngle = K_ADAS_BT_RotateDeadbandAngle;
  KV_ADAS_BT_TargetVisionAngle = K_ADAS_BT_TargetVisionAngle;
  KV_ADAS_BT_MaxTimeToWaitForCamera = K_ADAS_BT_MaxTimeToWaitForCamera;
  KV_ADAS_BT_TimedOutDriveForward = K_ADAS_BT_TimedOutDriveForward;
  KV_ADAS_BT_DriveForwardPct = K_ADAS_BT_DriveForwardPct;

  #ifdef ADAS_BT_Test
  // display coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTimeAxis[0]", KV_ADAS_BT_DriveTimeAxis[0]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTimeAxis[1]", KV_ADAS_BT_DriveTimeAxis[1]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTimeAxis[2]", KV_ADAS_BT_DriveTimeAxis[2]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTimeAxis[3]", KV_ADAS_BT_DriveTimeAxis[3]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTimeAxis[4]", KV_ADAS_BT_DriveTimeAxis[4]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTimeAxis[5]", KV_ADAS_BT_DriveTimeAxis[5]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTime[0]", KV_ADAS_BT_DriveTime[0]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTime[1]", KV_ADAS_BT_DriveTime[1]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTime[2]", KV_ADAS_BT_DriveTime[2]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTime[3]", KV_ADAS_BT_DriveTime[3]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTime[4]", KV_ADAS_BT_DriveTime[4]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveTime[5]", KV_ADAS_BT_DriveTime[5]);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_LightDelayTIme", KV_ADAS_BT_LightDelayTIme);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_LostTargetGx", KV_ADAS_BT_LostTargetGx);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_NoTargetError", KV_ADAS_BT_NoTargetError);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DebounceTime", KV_ADAS_BT_DebounceTime);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_RotateDeadbandAngle", KV_ADAS_BT_RotateDeadbandAngle);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_TargetVisionAngle", KV_ADAS_BT_TargetVisionAngle);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_MaxTimeToWaitForCamera", KV_ADAS_BT_MaxTimeToWaitForCamera);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_TimedOutDriveForward", KV_ADAS_BT_TimedOutDriveForward);
  frc::SmartDashboard::PutNumber("KV_ADAS_BT_DriveForwardPct", KV_ADAS_BT_DriveForwardPct);
  #endif
  }


/******************************************************************************
 * Function:     ADAS_BT_ConfigsCal
 *
 * Description:  Contains the motor configurations for the lift motors.  This 
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void ADAS_BT_ConfigsCal()
  {
  // read coefficients from SmartDashboard
  #ifdef ADAS_BT_Test
  KV_ADAS_BT_DriveTimeAxis[0] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTimeAxis[0]", KV_ADAS_BT_DriveTimeAxis[0]);
  KV_ADAS_BT_DriveTimeAxis[1] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTimeAxis[1]", KV_ADAS_BT_DriveTimeAxis[1]);
  KV_ADAS_BT_DriveTimeAxis[2] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTimeAxis[2]", KV_ADAS_BT_DriveTimeAxis[2]);
  KV_ADAS_BT_DriveTimeAxis[3] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTimeAxis[3]", KV_ADAS_BT_DriveTimeAxis[3]);
  KV_ADAS_BT_DriveTimeAxis[4] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTimeAxis[4]", KV_ADAS_BT_DriveTimeAxis[4]);
  KV_ADAS_BT_DriveTimeAxis[5] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTimeAxis[5]", KV_ADAS_BT_DriveTimeAxis[5]);
  KV_ADAS_BT_DriveTime[0] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTime[0]", KV_ADAS_BT_DriveTime[0]);
  KV_ADAS_BT_DriveTime[1] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTime[1]", KV_ADAS_BT_DriveTime[1]);
  KV_ADAS_BT_DriveTime[2] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTime[2]", KV_ADAS_BT_DriveTime[2]);
  KV_ADAS_BT_DriveTime[3] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTime[3]", KV_ADAS_BT_DriveTime[3]);
  KV_ADAS_BT_DriveTime[4] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTime[4]", KV_ADAS_BT_DriveTime[4]);
  KV_ADAS_BT_DriveTime[5] = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveTime[5]", KV_ADAS_BT_DriveTime[5]);
  KV_ADAS_BT_LightDelayTIme = frc::SmartDashboard::GetNumber("KV_ADAS_BT_LightDelayTIme", KV_ADAS_BT_LightDelayTIme);
  KV_ADAS_BT_LostTargetGx = frc::SmartDashboard::GetNumber("KV_ADAS_BT_LostTargetGx", KV_ADAS_BT_LostTargetGx);
  KV_ADAS_BT_NoTargetError = frc::SmartDashboard::GetNumber("KV_ADAS_BT_NoTargetError", KV_ADAS_BT_NoTargetError);
  KV_ADAS_BT_DebounceTime = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DebounceTime", KV_ADAS_BT_DebounceTime);
  KV_ADAS_BT_RotateDeadbandAngle = frc::SmartDashboard::GetNumber("KV_ADAS_BT_RotateDeadbandAngle", KV_ADAS_BT_RotateDeadbandAngle);
  KV_ADAS_BT_TargetVisionAngle = frc::SmartDashboard::GetNumber("KV_ADAS_BT_TargetVisionAngle", KV_ADAS_BT_TargetVisionAngle);
  KV_ADAS_BT_MaxTimeToWaitForCamera = frc::SmartDashboard::GetNumber("KV_ADAS_BT_MaxTimeToWaitForCamera", KV_ADAS_BT_MaxTimeToWaitForCamera);
  KV_ADAS_BT_TimedOutDriveForward = frc::SmartDashboard::GetNumber("KV_ADAS_BT_TimedOutDriveForward", KV_ADAS_BT_TimedOutDriveForward);
  KV_ADAS_BT_DriveForwardPct = frc::SmartDashboard::GetNumber("KV_ADAS_BT_DriveForwardPct", KV_ADAS_BT_DriveForwardPct);
  #endif
  }


/******************************************************************************
 * Function:     ADAS_BT_Reset
 *
 * Description:  Reset all applicable BT variables.
 ******************************************************************************/
void ADAS_BT_Reset(void)
  {
  V_ADAS_BT_State = E_ADAS_BT_Disabled;
  V_ADAS_BT_DebounceTime = 0;
  V_ADAS_BT_RotateErrorPrev = 0;
  V_ADAS_BT_DriveForwardTime = 0;
  V_ADAS_BT_TargetAquiredPrev = false;
  V_ADAS_BT_DriveForwardInitiated = false;
  V_ADAS_BT_DriveForwardTime = 0;
  }


/******************************************************************************
 * Function:     ADAS_UT_CameraLightOn
 *
 * Description:  Initializes and waits for the camera light to come on.
 ******************************************************************************/
T_ADAS_BT_BallTarget ADAS_BT_CameraLightOn(double *L_Pct_FwdRev,
                                            double *L_Pct_Strafe,
                                            double *L_Pct_Rotate,
                                            double *L_RPM_Launcher,
                                            double *L_Pct_Intake,
                                            double *L_Pct_Elevator,
                                            bool   *L_CameraUpperLightCmndOn,
                                            bool   *L_CameraLowerLightCmndOn,
                                            bool   *L_SD_RobotOriented)
  {
  T_ADAS_BT_BallTarget L_ADAS_BT_State = E_ADAS_BT_CameraLightOn;

  /* First thing, let's turn on the light: */
  *L_CameraLowerLightCmndOn = true;
  *L_SD_RobotOriented = true;
  /* Next, set all other values to off as we are just wanting to command the light on: */
  *L_CameraUpperLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;
  /* Indicate that the light is on and we can proceed: */
  /* Start incremeting a debounce time.  We want to give a bit of time for the camera to have light: */
  V_ADAS_BT_DebounceTime += C_ExeTime;

  if (V_ADAS_BT_DebounceTime >= KV_ADAS_BT_LightDelayTIme)
    {
    L_ADAS_BT_State = E_ADAS_BT_AutoCenter;
    V_ADAS_BT_DebounceTime = 0;
    }

  return (L_ADAS_BT_State);
  }


/******************************************************************************
 * Function:     ADAS_BT_AutoCenter
 *
 * Description:  Rotates the robot to face the ball target.
 ******************************************************************************/
T_ADAS_BT_BallTarget ADAS_BT_AutoCenter(double *L_Pct_FwdRev,
                                        double *L_Pct_Strafe,
                                        double *L_Pct_Rotate,
                                        double *L_RPM_Launcher,
                                        double *L_Pct_Intake,
                                        double *L_Pct_Elevator,
                                        bool   *L_CameraUpperLightCmndOn,
                                        bool   *L_CameraLowerLightCmndOn,
                                        bool   *L_SD_RobotOriented,
                                        double  L_VisionBottomTargetAquired,
                                        double  L_VisionBottomYaw)
  {
  T_ADAS_BT_BallTarget L_ADAS_BT_State = E_ADAS_BT_AutoCenter;
  double L_RotateErrorCalc = 0;

  *L_CameraLowerLightCmndOn = true;
  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;
  *L_RPM_Launcher = 0;

  /* Ok, now let's focus on the auto centering: */
  if (L_VisionBottomTargetAquired == true)
    {
    L_RotateErrorCalc = KV_ADAS_BT_TargetVisionAngle - L_VisionBottomYaw;
    V_ADAS_BT_RotateErrorPrev = L_RotateErrorCalc;
    V_ADAS_BT_TargetAquiredPrev = true;
    }
  else if (V_ADAS_BT_TargetAquiredPrev == true)
    {
    /* Hmm, we see to have lost the target.  Use previous value, but reduce so that we don't go too far. */
    L_RotateErrorCalc = V_ADAS_BT_RotateErrorPrev * KV_ADAS_BT_LostTargetGx;
    }
  else
    {
    /* Ehh, we don't seem to have observed a good value from the camera yet.
       Let's take a stab in the dark and hope that we can see something... */
    L_RotateErrorCalc = KV_ADAS_BT_NoTargetError;
    }
  

  if (fabs(L_RotateErrorCalc) <= KV_ADAS_BT_RotateDeadbandAngle && V_ADAS_BT_DebounceTime < KV_ADAS_BT_DebounceTime)
    {
    V_ADAS_BT_DebounceTime += C_ExeTime;
    }
  else if (fabs(L_RotateErrorCalc) > KV_ADAS_BT_RotateDeadbandAngle)
    {
    /* Reset the timer, we have gone out of bounds */
    V_ADAS_BT_DebounceTime = 0;
    }
  else if (V_ADAS_BT_DebounceTime >= KV_ADAS_BT_DebounceTime)
    {
    /* Reset the time, proceed to next state. */
    L_ADAS_BT_State = E_ADAS_BT_IntakeAndRun;
    V_ADAS_BT_DebounceTime = 0;
    V_ADAS_BT_RotateErrorPrev = 0;
    V_ADAS_BT_TargetAquiredPrev = false;
    }

  if (L_ADAS_BT_State == E_ADAS_BT_AutoCenter)
    {
    *L_Pct_Rotate = DesiredRotateSpeed(L_RotateErrorCalc);
    }
  else
    {
    /* We have been at the correct location for the set amount of time.
       We have previously set the state to the next one, now set the rotate command to off. */
    *L_Pct_Rotate = 0;
    }
  
  return (L_ADAS_BT_State);
  }


/******************************************************************************
 * Function:     ADAS_UT_LauncherSpeed
 *
 * Description:  Spins up the launcher to the correct speed.
 ******************************************************************************/
T_ADAS_BT_BallTarget ADAS_BT_IntakeAndRun(double *L_Pct_FwdRev,
                                          double *L_Pct_Strafe,
                                          double *L_Pct_Rotate,
                                          double *L_RPM_Launcher,
                                          double *L_Pct_Intake,
                                          double *L_Pct_Elevator,
                                          bool   *L_CameraUpperLightCmndOn,
                                          bool   *L_CameraLowerLightCmndOn,
                                          bool   *L_SD_RobotOriented,
                                          double  L_VisionBottomTargetAquired,
                                          double  L_VisionBottomTargetDistanceMeters)
  {
  T_ADAS_BT_BallTarget L_ADAS_BT_State = E_ADAS_BT_IntakeAndRun;
  bool                 L_DistanceFound = false;

  *L_CameraLowerLightCmndOn = true;
  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_Pct_Elevator = 0;

  /* Ok, now let's focus on the getting the correct estimated distance: */
  if (V_ADAS_BT_DriveForwardInitiated == false)
    {
    if (L_VisionBottomTargetAquired == true)
      {
      V_ADAS_BT_DriveForwardTime = DtrmnTimeToDriveToCaptureBall(L_VisionBottomTargetDistanceMeters);
      V_ADAS_BT_DriveForwardInitiated = true;
      }
    else
      {
      /* Hmm, we see to have lost the target.  Let's wait a bit... */
      V_ADAS_BT_DebounceTime += C_ExeTime;
      if (V_ADAS_BT_DebounceTime >= KV_ADAS_BT_TimedOutDriveForward)
        {
        /* Well, we have waited long enough.  Let's pick a value and go!! */
        V_ADAS_BT_DriveForwardTime = KV_ADAS_BT_TimedOutDriveForward;
        V_ADAS_BT_DriveForwardInitiated = true;
        V_ADAS_BT_DebounceTime = 0;
        }
      }
    }

  if (V_ADAS_BT_DriveForwardInitiated == true)
    {
    V_ADAS_BT_DebounceTime += C_ExeTime;
    if (V_ADAS_BT_DebounceTime < V_ADAS_BT_DriveForwardTime)
      {
      *L_Pct_FwdRev = KV_ADAS_BT_DriveForwardPct;
      *L_Pct_Intake = K_IntakePower;
      }
    else
      {
      *L_Pct_FwdRev = 0;
      *L_Pct_Intake = 0;
      *L_CameraLowerLightCmndOn = false;
      L_ADAS_BT_State = E_ADAS_BT_Disabled;
      }
    }
  else
    {
    *L_Pct_FwdRev = 0;
    *L_Pct_Intake = 0;
    }

  return (L_ADAS_BT_State);
  }


/******************************************************************************
 * Function:     ADAS_BT_Main
 *
 * Description:  Manages and controls the ball targeting (BT).
 ******************************************************************************/
T_ADAS_ActiveFeature ADAS_BT_Main(double               *L_Pct_FwdRev,
                                  double               *L_Pct_Strafe,
                                  double               *L_Pct_Rotate,
                                  double               *L_RPM_Launcher,
                                  double               *L_Pct_Intake,
                                  double               *L_Pct_Elevator,
                                  bool                 *L_CameraUpperLightCmndOn,
                                  bool                 *L_CameraLowerLightCmndOn,
                                  bool                 *L_SD_RobotOriented,
                                  T_ADAS_ActiveFeature  L_ADAS_ActiveFeature,
                                  bool                  L_VisionBottomTargetAquired,
                                  double                L_VisionBottomYaw,
                                  double                L_VisionBottomTargetDistanceMeters,
                                  T_RobotState          L_RobotState,
                                  bool                  L_BallDetected)
  {
  if (L_ADAS_ActiveFeature == E_ADAS_AutoBallTarget)
    {
    switch (V_ADAS_BT_State)
      {
      case E_ADAS_BT_Disabled:
          V_ADAS_BT_State = ADAS_BT_CameraLightOn(L_Pct_FwdRev,
                                                  L_Pct_Strafe,
                                                  L_Pct_Rotate,
                                                  L_RPM_Launcher,
                                                  L_Pct_Intake,
                                                  L_Pct_Elevator,
                                                  L_CameraUpperLightCmndOn,
                                                  L_CameraLowerLightCmndOn,
                                                  L_SD_RobotOriented);
      break;
      case E_ADAS_BT_AutoCenter:
          V_ADAS_BT_State = ADAS_BT_AutoCenter(L_Pct_FwdRev,
                                               L_Pct_Strafe,
                                               L_Pct_Rotate,
                                               L_RPM_Launcher,
                                               L_Pct_Intake,
                                               L_Pct_Elevator,
                                               L_CameraUpperLightCmndOn,
                                               L_CameraLowerLightCmndOn,
                                               L_SD_RobotOriented,
                                               L_VisionBottomTargetAquired,
                                               L_VisionBottomYaw);
      break;
      case E_ADAS_BT_IntakeAndRun:
          V_ADAS_BT_State = ADAS_BT_IntakeAndRun(L_Pct_FwdRev,
                                                 L_Pct_Strafe,
                                                 L_Pct_Rotate,
                                                 L_RPM_Launcher,
                                                 L_Pct_Intake,
                                                 L_Pct_Elevator,
                                                 L_CameraUpperLightCmndOn,
                                                 L_CameraLowerLightCmndOn,
                                                 L_SD_RobotOriented,
                                                 L_VisionBottomTargetAquired,
                                                 L_VisionBottomTargetDistanceMeters);
      break;
      }
    }

  if (V_ADAS_BT_State == E_ADAS_BT_Disabled)
    {
    ADAS_BT_Reset();
    L_ADAS_ActiveFeature = E_ADAS_Disabled;
    }
  
  return (L_ADAS_ActiveFeature);
  }