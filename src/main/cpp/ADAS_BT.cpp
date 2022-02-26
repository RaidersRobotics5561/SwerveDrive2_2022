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

#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Const.hpp"

T_ADAS_BT_BallTarget V_ADAS_BT_State             = E_ADAS_BT_Disabled;
double               V_ADAS_BT_DebounceTime      = 0;
double               V_ADAS_BT_RotateErrorPrev   = 0;
bool                 V_ADAS_BT_TargetAquiredPrev = false;
bool                 V_ADAS_BT_DriveForwardInitiated = false;
double               V_ADAS_BT_DriveForwardTime     = 0;


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
                                        bool   *L_CameraLightCmndOn,
                                        double  L_VisionBottomTargetAquired,
                                        double  L_VisionBottomYaw)
  {
  T_ADAS_BT_BallTarget L_ADAS_BT_State = E_ADAS_BT_AutoCenter;
  double L_RotateErrorCalc = 0;

  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;
  *L_RPM_Launcher = 0;

  /* Ok, now let's focus on the auto centering: */
  if (L_VisionBottomTargetAquired == true)
    {
    L_RotateErrorCalc = K_ADAS_BT_TargetVisionAngle - L_VisionBottomYaw;
    V_ADAS_BT_RotateErrorPrev = L_RotateErrorCalc;
    V_ADAS_BT_TargetAquiredPrev = true;
    }
  else if (V_ADAS_BT_TargetAquiredPrev == true)
    {
    /* Hmm, we see to have lost the target.  Use previous value, but reduce so that we don't go too far. */
    L_RotateErrorCalc = V_ADAS_BT_RotateErrorPrev * K_ADAS_BT_LostTargetGx;
    }
  else
    {
    /* Ehh, we don't seem to have observed a good value from the camera yet.
       Let's take a stab in the dark and hope that we can see something... */
    L_RotateErrorCalc = K_ADAS_BT_NoTargetError;
    }
  

  if (fabs(L_RotateErrorCalc) <= K_ADAS_BT_RotateDeadbandAngle && V_ADAS_BT_DebounceTime < K_ADAS_BT_DebounceTime)
    {
    V_ADAS_BT_DebounceTime += C_ExeTime;
    }
  else if (fabs(L_RotateErrorCalc) > K_ADAS_BT_RotateDeadbandAngle)
    {
    /* Reset the timer, we have gone out of bounds */
    V_ADAS_BT_DebounceTime = 0;
    }
  else if (V_ADAS_BT_DebounceTime >= K_ADAS_BT_DebounceTime)
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
                                          bool   *L_CameraLightCmndOn,
                                          double  L_VisionBottomTargetAquired,
                                          double  L_VisionBottomTargetDistanceMeters)
  {
  T_ADAS_BT_BallTarget L_ADAS_BT_State = E_ADAS_BT_IntakeAndRun;
  bool                 L_DistanceFound = false;

  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraLightCmndOn = false;
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
      if (V_ADAS_BT_DebounceTime >= K_ADAS_BT_TimedOutDriveForward)
        {
        /* Well, we have waited long enough.  Let's pick a value and go!! */
        V_ADAS_BT_DriveForwardTime = K_ADAS_BT_TimedOutDriveForward;
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
      *L_Pct_FwdRev = K_ADAS_BT_DriveForwardPct;
      *L_Pct_Intake = K_IntakePower;
      }
    else
      {
      *L_Pct_FwdRev = 0;
      *L_Pct_Intake = 0;
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
                                  bool                 *L_CameraLightCmndOn,
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
      case E_ADAS_BT_AutoCenter:
          V_ADAS_BT_State = ADAS_BT_AutoCenter(L_Pct_FwdRev,
                                               L_Pct_Strafe,
                                               L_Pct_Rotate,
                                               L_RPM_Launcher,
                                               L_Pct_Intake,
                                               L_Pct_Elevator,
                                               L_CameraLightCmndOn,
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
                                                 L_CameraLightCmndOn,
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