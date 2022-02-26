/*
  ADAS_BT.hpp

  Created on: Feb 26, 2022
  Author: Biggs

  ADAS Ball Targeting

  Changes:
  2022-02-25 -> Beta
 */

void ADAS_BT_Reset(void);

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
                                  bool                  L_BallDetected);