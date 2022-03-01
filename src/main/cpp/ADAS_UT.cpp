/*
  ADAS_UT.cpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems) Upper Targeting
  Contains the logic and code used for the upper targeting control:
    - Turns on camera light, auto centers robot on target, spins the rollers up to the correct speed, disables camera light

  Changes:
  2022-02-25 -> Beta
 */

#include <math.h>

#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Const.hpp"

T_ADAS_UT_UpperTarget V_ADAS_UT_State             = E_ADAS_UT_Disabled;
double                V_ADAS_UT_DebounceTime      = 0;
double                V_ADAS_UT_RotateErrorPrev   = 0;
double                V_ADAS_UT_LauncherSpeedPrev = 0;
bool                  V_ADAS_UT_TargetAquiredPrev = false;

/******************************************************************************
 * Function:     ADAS_UT_Reset
 *
 * Description:  Reset all applicable UT variables.
 ******************************************************************************/
void ADAS_UT_Reset(void)
  {
  V_ADAS_UT_State = E_ADAS_UT_Disabled;
  V_ADAS_UT_DebounceTime = 0;
  V_ADAS_UT_RotateErrorPrev = 0;
  V_ADAS_UT_LauncherSpeedPrev = 0;
  V_ADAS_UT_TargetAquiredPrev = false;
  }


/******************************************************************************
 * Function:     ADAS_UT_CameraLightOn
 *
 * Description:  Initializes and waits for the camera light to come on.
 ******************************************************************************/
T_ADAS_UT_UpperTarget ADAS_UT_CameraLightOn(double *L_Pct_FwdRev,
                                            double *L_Pct_Strafe,
                                            double *L_Pct_Rotate,
                                            double *L_RPM_Launcher,
                                            double *L_Pct_Intake,
                                            double *L_Pct_Elevator,
                                            bool   *L_CameraUpperLightCmndOn,
                                            bool   *L_CameraLowerLightCmndOn,
                                            bool   *L_SD_RobotOriented)
  {
  T_ADAS_UT_UpperTarget L_ADAS_UT_State = E_ADAS_UT_CameraLightOn;

  /* First thing, let's turn on the light: */
  *L_CameraUpperLightCmndOn = true;

  *L_SD_RobotOriented = true;
  /* Next, set all other values to off as we are just wanting to command the light on: */
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;
  /* Indicate that the light is on and we can proceed: */
  /* Start incremeting a debounce time.  We want to give a bit of time for the camera to have light: */
  V_ADAS_UT_DebounceTime += C_ExeTime;

  if (V_ADAS_UT_DebounceTime >= K_ADAS_UT_LightDelayTIme)
    {
    L_ADAS_UT_State = E_ADAS_UT_AutoCenter;
    V_ADAS_UT_DebounceTime = 0;
    }

  return (L_ADAS_UT_State);
  }


/******************************************************************************
 * Function:     ADAS_UT_AutoCenter
 *
 * Description:  Rotates the robot to face the upper target.
 ******************************************************************************/
T_ADAS_UT_UpperTarget ADAS_UT_AutoCenter(double *L_Pct_FwdRev,
                                         double *L_Pct_Strafe,
                                         double *L_Pct_Rotate,
                                         double *L_RPM_Launcher,
                                         double *L_Pct_Intake,
                                         double *L_Pct_Elevator,
                                         bool   *L_CameraUpperLightCmndOn,
                                         bool   *L_CameraLowerLightCmndOn,
                                         bool   *L_SD_RobotOriented,
                                         double  L_VisionTopTargetAquired,
                                         double  L_TopTargetYawDegrees)
  {
  T_ADAS_UT_UpperTarget L_ADAS_UT_State = E_ADAS_UT_AutoCenter;
  double L_RotateErrorCalc = 0;

  /* First thing, let's keep the light on: */
  *L_CameraUpperLightCmndOn = true;

  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  /* Get the launcher ready by setting it to the lowest expected speed */
  *L_RPM_Launcher = K_BH_LauncherSpeed[0];

  /* Ok, now let's focus on the auto centering: */
  if (L_VisionTopTargetAquired == true)
    {
    L_RotateErrorCalc = K_ADAS_UT_TargetVisionAngle - L_TopTargetYawDegrees;
    V_ADAS_UT_RotateErrorPrev = L_RotateErrorCalc;
    V_ADAS_UT_TargetAquiredPrev = true;
    }
  else if (V_ADAS_UT_TargetAquiredPrev == true)
    {
    /* Hmm, we see to have lost the target.  Use previous value, but reduce so that we don't go too far. */
    L_RotateErrorCalc = V_ADAS_UT_RotateErrorPrev * K_ADAS_UT_LostTargetGx;
    }
  else
    {
    /* Ehh, we don't seem to have observed a good value from the camera yet.
       Let's take a stab in the dark and hope that we can see something... */
    L_RotateErrorCalc = K_ADAS_UT_NoTargetError;
    }
  

  if (fabs(L_RotateErrorCalc) <= K_ADAS_UT_RotateDeadbandAngle && V_ADAS_UT_DebounceTime < K_ADAS_UT_DebounceTime)
    {
    V_ADAS_UT_DebounceTime += C_ExeTime;
    }
  else if (fabs(L_RotateErrorCalc) > K_ADAS_UT_RotateDeadbandAngle)
    {
    /* Reset the timer, we have gone out of bounds */
    V_ADAS_UT_DebounceTime = 0;
    }
  else if (V_ADAS_UT_DebounceTime >= K_ADAS_UT_DebounceTime)
    {
    /* Reset the time, proceed to next state. */
    L_ADAS_UT_State = E_ADAS_UT_LauncherSpeed;
    V_ADAS_UT_DebounceTime = 0;
    V_ADAS_UT_RotateErrorPrev = 0;
    V_ADAS_UT_TargetAquiredPrev = false;
    }

  if (L_ADAS_UT_State == E_ADAS_UT_AutoCenter)
    {
    *L_Pct_Rotate = DesiredRotateSpeed(L_RotateErrorCalc);
    }
  else
    {
    /* We have been at the correct location for the set amount of time.
       We have previously set the state to the next one, now set the rotate command to off. */
    *L_Pct_Rotate = 0;
    }
  
  return (L_ADAS_UT_State);
  }


/******************************************************************************
 * Function:     ADAS_UT_LauncherSpeed
 *
 * Description:  Spins up the launcher to the correct speed.
 ******************************************************************************/
T_ADAS_UT_UpperTarget ADAS_UT_LauncherSpeed(double *L_Pct_FwdRev,
                                            double *L_Pct_Strafe,
                                            double *L_Pct_Rotate,
                                            double *L_RPM_Launcher,
                                            double *L_Pct_Intake,
                                            double *L_Pct_Elevator,
                                            bool   *L_CameraUpperLightCmndOn,
                                            bool   *L_CameraLowerLightCmndOn,
                                            bool   *L_SD_RobotOriented,
                                            double  L_VisionTopTargetAquired,
                                            double  L_VisionTopTargetDistanceMeters)
  {
  T_ADAS_UT_UpperTarget L_ADAS_UT_State = E_ADAS_UT_LauncherSpeed;
  double                L_LauncherSpeedCmnd = 0;

  /* First thing, let's keep the light on: */
  *L_CameraUpperLightCmndOn = true;

  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  /* Ok, now let's focus on the getting the launcher up to the correct speed: */
  if (L_VisionTopTargetAquired == true)
    {
    L_LauncherSpeedCmnd = DtrmnAutoLauncherSpeed(L_VisionTopTargetDistanceMeters);
    V_ADAS_UT_LauncherSpeedPrev = L_LauncherSpeedCmnd;
    V_ADAS_UT_TargetAquiredPrev = true;
    V_ADAS_UT_DebounceTime += C_ExeTime;
    }
  else if (V_ADAS_UT_TargetAquiredPrev == true)
    {
    /* Hmm, we see to have lost the target.  Use previous value. */
    L_LauncherSpeedCmnd = V_ADAS_UT_LauncherSpeedPrev;
    }
  else
    {
    /* Ehh, we don't seem to have observed a good value from the camera yet.
       Let's take a stab in the dark and hold it at the initial value and 
       hope that we can see something soon... */
    L_LauncherSpeedCmnd = K_BH_LauncherSpeed[0];
    V_ADAS_UT_DebounceTime = 0;
    }
  
  if (V_ADAS_UT_DebounceTime >= K_ADAS_UT_DebounceTime)
    {
    /* Ok, we have had enough good camera values/time to beleive we have a decent distance estimate. */
    L_ADAS_UT_State = E_ADAS_UT_ElevatorControl;
    V_ADAS_UT_DebounceTime = 0;
    V_ADAS_UT_TargetAquiredPrev = false;
    // V_ADAS_UT_LauncherSpeedPrev -> Don't reset the previous speed, we will need this later
    }

  *L_RPM_Launcher = L_LauncherSpeedCmnd;

  return (L_ADAS_UT_State);
  }


/******************************************************************************
 * Function:     ADAS_UT_ElevatorControl
 *
 * Description:  Controls the elevator.  This will differ if in teleop or auton.
 ******************************************************************************/
T_ADAS_UT_UpperTarget ADAS_UT_ElevatorControl(double       *L_Pct_FwdRev,
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
  T_ADAS_UT_UpperTarget L_ADAS_UT_State = E_ADAS_UT_ElevatorControl;
  double                L_LauncherError = 0;
  double                L_LauncherSpeedCmnd = 0;

  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = V_ADAS_UT_LauncherSpeedPrev; // Hold the desired speed

  L_LauncherError = fabs(L_LauncherRPM_Measured - V_ADAS_UT_LauncherSpeedPrev);



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
             (L_LauncherError <= K_ADAS_UT_AllowedLauncherError))
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
         (L_LauncherError <= K_ADAS_UT_AllowedLauncherError)) &&

        (V_ADAS_UT_DebounceTime < K_ADAS_UT_AllowedLauncherTime))
      {
      *L_Pct_Intake = K_IntakePower;
      *L_Pct_Elevator = K_ElevatorPowerUp;
      }
    else
      {
      *L_Pct_Intake = 0;
      *L_Pct_Elevator = 0;
      }

    if (V_ADAS_UT_DebounceTime >= K_ADAS_UT_AllowedLauncherTime)
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
 * Description:  Manages and controls the upper targeting (UT) and spinning up of 
 *               the launcher.
 ******************************************************************************/
T_ADAS_ActiveFeature ADAS_UT_Main(double               *L_Pct_FwdRev,
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
                                  bool                  L_DriverRequestElevatorDwn)
  {

  if (L_ADAS_ActiveFeature == E_ADAS_AutoUpperTarget)
    {
    switch (V_ADAS_UT_State)
      {
      case E_ADAS_UT_Disabled:
          V_ADAS_UT_State = ADAS_UT_CameraLightOn(L_Pct_FwdRev,
                                                  L_Pct_Strafe,
                                                  L_Pct_Rotate,
                                                  L_RPM_Launcher,
                                                  L_Pct_Intake,
                                                  L_Pct_Elevator,
                                                  L_CameraUpperLightCmndOn,
                                                  L_CameraLowerLightCmndOn,
                                                  L_SD_RobotOriented);
      break;
      case E_ADAS_UT_AutoCenter:
          V_ADAS_UT_State = ADAS_UT_AutoCenter(L_Pct_FwdRev,
                                               L_Pct_Strafe,
                                               L_Pct_Rotate,
                                               L_RPM_Launcher,
                                               L_Pct_Intake,
                                               L_Pct_Elevator,
                                               L_CameraUpperLightCmndOn,
                                               L_CameraLowerLightCmndOn,
                                               L_SD_RobotOriented,
                                               L_VisionTopTargetAquired,
                                               L_TopTargetYawDegrees);
      break;
      case E_ADAS_UT_LauncherSpeed:
          V_ADAS_UT_State = ADAS_UT_LauncherSpeed(L_Pct_FwdRev,
                                                  L_Pct_Strafe,
                                                  L_Pct_Rotate,
                                                  L_RPM_Launcher,
                                                  L_Pct_Intake,
                                                  L_Pct_Elevator,
                                                  L_CameraUpperLightCmndOn,
                                                  L_CameraLowerLightCmndOn,
                                                  L_SD_RobotOriented,
                                                  L_VisionTopTargetAquired,
                                                  L_VisionTopTargetDistanceMeters);
      break;
      case E_ADAS_UT_ElevatorControl:
          V_ADAS_UT_State = ADAS_UT_ElevatorControl(L_Pct_FwdRev,
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
    ADAS_UT_Reset();
    L_ADAS_ActiveFeature = E_ADAS_Disabled;
    }
  
  return (L_ADAS_ActiveFeature);
  }