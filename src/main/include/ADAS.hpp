/*
  ADAS.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems)

  Changes:
  2022-02-25 -> Beta
 */

extern T_ADAS_ActiveFeature V_ADAS_ActiveFeature;

extern double               V_ADAS_Pct_SD_FwdRev;
extern double               V_ADAS_Pct_SD_Strafe;
extern double               V_ADAS_Pct_SD_Rotate;
extern double               V_ADAS_RPM_BH_Launcher;
extern double               V_ADAS_Pct_BH_Intake;
extern double               V_ADAS_Pct_BH_Elevator;
extern bool                 V_ADAS_CameraUpperLightCmndOn;
extern bool                 V_ADAS_CameraLowerLightCmndOn;
extern bool                 V_ADAS_SD_RobotOriented;

void ADAS_Main_Reset(void);

T_ADAS_ActiveFeature ADAS_ControlMainTeleop(double              *L_Pct_FwdRev,
                                           double               *L_Pct_Strafe,
                                           double               *L_Pct_Rotate,
                                           double               *L_RPM_Launcher,
                                           double               *L_Pct_Intake,
                                           double               *L_Pct_Elevator,
                                           bool                 *L_CameraUpperLightCmndOn,
                                           bool                 *L_CameraLowerLightCmndOn,
                                           bool                 *L_SD_RobotOriented,
                                           bool                  L_Driver1_JoystickActive,
                                           bool                  L_Driver_stops_shooter,
                                           bool                  L_Driver_SwerveGoalAutoCenter,
                                           bool                  L_Driver_AutoIntake,
                                           double                L_Deg_GyroAngleDeg,
                                           bool                  L_VisionTopTargetAquired,
                                           double                L_TopTargetYawDegrees,
                                           double                L_VisionTopTargetDistanceMeters,
                                           bool                  L_VisionBottomTargetAquired,
                                           double                L_VisionBottomYaw,
                                           double                L_VisionBottomTargetDistanceMeters,
                                           T_RobotState          L_RobotState,
                                           double                L_LauncherRPM_Measured,
                                           bool                  L_BallDetected,
                                           bool                  L_DriverRequestElevatorUp,
                                           bool                  L_DriverRequestElevatorDwn,
                                           T_ADAS_ActiveFeature  L_ADAS_ActiveFeature);