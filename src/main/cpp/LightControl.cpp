/*
  LightControl.cpp

  Created on: Feb 15, 2022
  Author: Biggs

  This file contains functions related to control of lights on the robot.
  This can include but is not limited to:
   - LED vanity lights
   - Camera lights
 */

#include <frc/DriverStation.h>

#include "Const.hpp"

/* V_CameraLightOnTime: Indication of how long the light has been consecutivly on. */
double V_CameraLightOnTime = 0;

/* V_CameraLightStatus: Indication of the camera light status. */
T_CameraLightStatus V_CameraLightStatus = E_LightTurnedOff;

/* V_CameraLightCmndOn: Commanded camera light on/off state. */
bool V_CameraLightCmndOn = false;

/* V_VanityLightCmnd: PWM command to be sent to the blinkin controller. */
double  V_VanityLightCmnd = 0;

/******************************************************************************
 * Function:     CameraLightControl
 *
 * Description:  Contains the functionality for controlling the camera light.
 *               - Limits on time to prevent damaging light.
 *               - Informs targeting logic when camera feed should have had 
 *                 enough time with light on for accurate data.
 ******************************************************************************/
bool CameraLightControl(bool             L_AutoAlignRequest,
                        bool             L_AutoLauncherRequest,
                        T_LauncherStates L_LauncherState,
                        bool             L_SwerveTargetLocking,
                        bool             L_Driver_CameraLight,
                        bool             L_ShooterTargetSpeedReached)
  {
    bool L_CameraLightCmndOn = false;

    if ((L_AutoAlignRequest == true) ||    /* Swerve drive targeting has been requested or is in process */
        (L_SwerveTargetLocking == true) ||

        ((L_ShooterTargetSpeedReached == false) &&  /* Ball targeting has been requested or is in process */
         (L_AutoLauncherRequest == true) ||
         (L_LauncherState == E_LauncherAutoTargetActive)) ||
        
        (L_Driver_CameraLight == true))  /* Driver override is present */
      {
      L_CameraLightCmndOn = true;
      }

    if ((L_CameraLightCmndOn == true) &&
        (V_CameraLightOnTime < K_CameraLightMaxOnTime) &&
        (V_CameraLightStatus != E_LightForcedOffDueToOvertime))
      {
      V_CameraLightOnTime += C_ExeTime;

      if (V_CameraLightOnTime >= K_CameraLightDelay)
        {
        V_CameraLightStatus = E_LightOnTargetingReady;
        }
      else
        {
        V_CameraLightStatus = E_LightOnWaitingForTarget;
        }
      }
    else if ((L_CameraLightCmndOn == true) &&
             (V_CameraLightOnTime >= K_CameraLightMaxOnTime))
      {
        L_CameraLightCmndOn = false; // turn light off, give time to cool down

        V_CameraLightStatus = E_LightForcedOffDueToOvertime;
      }
    else
      {
      V_CameraLightOnTime = 0;
      L_CameraLightCmndOn = false;
      V_CameraLightStatus = E_LightTurnedOff;
      }
  
  /* Flip the command as the camera light is inverted */
  if (L_CameraLightCmndOn == true)
    {
    L_CameraLightCmndOn = false;
    }
  else
    {
    L_CameraLightCmndOn = true;
    }

  return (L_CameraLightCmndOn);
  }

/******************************************************************************
 * Function:     VanityLightControl
 *
 * Description:  Contains the functionality for controlling the vanity lights.
 *               - Changes colors based on alliance.
 *               - Will change color when in end game to help inform driver to
 *                 take action.
 ******************************************************************************/
double VanityLightControl(double L_MatchTimeRemaining,
                          frc::DriverStation::Alliance L_AllianceColor)
  {
    double L_LED_Command = 0;

    if (L_MatchTimeRemaining <= C_End_game_time)
      {
      L_LED_Command = -0.89;
      }
    else if (L_AllianceColor == frc::DriverStation::Alliance::kRed)
      {
      L_LED_Command = -0.17;
      }
    else if (L_AllianceColor == frc::DriverStation::Alliance::kBlue)
      {
      L_LED_Command = -0.15;
      }
    else
      {
      L_LED_Command = -0.27;
      }

    return(L_LED_Command);
  }

/******************************************************************************
 * Function:     LightControlMain
 *
 * Description:  Contains the functionality for controlling the camera 
 *               illumination lights and LED vanity lights.
 ******************************************************************************/
void LightControlMain(bool                         L_AutoAlignRequest,
                      bool                         L_AutoLauncherRequest,
                      double                       L_MatchTimeRemaining,
                      frc::DriverStation::Alliance L_AllianceColor,
                      T_LauncherStates             L_LauncherState,
                      bool                         L_SwerveTargetLocking,
                      bool                         L_Driver_CameraLight,
                      bool                         L_ShooterTargetSpeedReached,
                      bool                        *L_CameraLightCmndOn,
                      double                      *L_VanityLightCmnd)
  {
  *L_CameraLightCmndOn = CameraLightControl(L_AutoAlignRequest,
                                            L_AutoLauncherRequest,
                                            L_LauncherState,
                                            L_SwerveTargetLocking,
                                            L_Driver_CameraLight,
                                            L_ShooterTargetSpeedReached);

  *L_VanityLightCmnd = VanityLightControl(L_MatchTimeRemaining,
                                          L_AllianceColor);
  }