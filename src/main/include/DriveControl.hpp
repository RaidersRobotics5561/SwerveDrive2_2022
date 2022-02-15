/*
  DriveControl.hpp

   Created on: Feb 25, 2020

   Author: 5561

   Updates:
   2022-02-15: Cleaned up file
 */

extern double V_WheelAngleCmnd[E_RobotCornerSz];
extern double V_WheelSpeedCmnd[E_RobotCornerSz];

void DriveControlInit(void);

void DriveControlMain(double       L_JoyStick1Axis1Y,
                      double       L_JoyStick1Axis1X,
                      double       L_JoyStick1Axis2X,
                      double       L_JoyStick1Axis3,
                      bool         L_JoyStick1Button1,
                      double       L_JoyStick1Button3,
                      double       L_JoyStick1Button4,
                      double       L_JoyStick1Button5,
                      double       L_GyroAngleDegrees,
                      double       L_GyroAngleRadians,
                      double       L_VisionAngleDeg,
                      double      *L_WheelAngleFwd,
                      double      *L_WheelAngleRev,
                      double      *L_WheelSpeedCmnd,
                      double      *L_WheelAngleCmnd,
                      bool        *L_RobotInit,
                      bool        *L_TargetFin,
                      T_RobotState L_RobotState);

