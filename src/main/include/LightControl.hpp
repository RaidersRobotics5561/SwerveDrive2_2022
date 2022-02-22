/*
  LightControl.cpp

  Created on: Feb 15, 2022
  Author: Biggs

  This file contains functions related to control of lights on the robot.
  This can include but is not limited to:
   - LED vanity lights
   - Camera lights
 */

extern T_CameraLightStatus V_CameraLightStatus;
extern bool V_CameraLightCmndOn;
extern double  V_VanityLightCmnd;

void LightControlMain(bool                         L_AutoAlignRequest,
                      bool                         L_AutoLauncherRequest,
                      double                       L_MatchTimeRemaining,
                      frc::DriverStation::Alliance L_AllianceColor,
                      T_LauncherStates             L_LauncherState,
                      bool                         L_SwerveTargetLocking,
                      bool                        *L_CameraLightCmndOn,
                      double                      *L_VanityLightCmnd);