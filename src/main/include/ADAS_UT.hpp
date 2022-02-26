/*
  ADAS_UT.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS Upper Targeting

  Changes:
  2022-02-25 -> Beta
 */


void ADAS_UT_Reset(void);

T_ADAS_ActiveFeature ADAS_UT_Main(double               *L_Pct_FwdRev,
                                  double               *L_Pct_Strafe,
                                  double               *L_Pct_Rotate,
                                  double               *L_RPM_Launcher,
                                  double               *L_Pct_Intake,
                                  double               *L_Pct_Elevator,
                                  bool                 *L_CameraLightCmndOn,
                                  T_ADAS_ActiveFeature  L_ADAS_ActiveFeature,
                                   bool                 L_VisionTopTargetAquired,
                                  double                L_TopTargetYawDegrees,
                                  double                L_VisionTopTargetDistanceMeters,
                                  T_RobotState          L_RobotState,
                                  double                L_LauncherRPM_Measured,
                                  bool                  L_BallDetected,
                                  bool                  L_DriverRequestElevatorUp,
                                  bool                  L_DriverRequestElevatorDwn);