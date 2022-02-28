/*
  DriveControl.hpp

   Created on: Feb 25, 2020

   Author: 5561

   Updates:
   2022-02-15: Cleaned up file
 */

extern double V_WheelAngleCmnd[E_RobotCornerSz];
extern double V_WheelSpeedCmnd[E_RobotCornerSz];
extern bool   V_SwerveTargetLockingUpper;
extern bool   V_b_DriveStraight;
extern double V_RotateErrorCalc;
extern bool   V_SD_DriveWheelsInPID;

void SwerveDriveMotorConfigsInit(rev::SparkMaxPIDController m_frontLeftDrivePID,
                                 rev::SparkMaxPIDController m_frontRightDrivePID,
                                 rev::SparkMaxPIDController m_rearLeftDrivePID,
                                 rev::SparkMaxPIDController m_rearRightDrivePID);

void SwerveDriveMotorConfigsCal(rev::SparkMaxPIDController m_frontLeftDrivePID,
                                rev::SparkMaxPIDController m_frontRightDrivePID,
                                rev::SparkMaxPIDController m_rearLeftDrivePID,
                                rev::SparkMaxPIDController m_rearRightDrivePID);

void DriveControlInit(void);

void DriveControlMain(double       L_JoyStick1Axis1Y,
                      double       L_JoyStick1Axis1X,
                      double       L_JoyStick1Axis2X,
                      double       L_JoyStick1Axis3,
                      bool         L_JoyStick1Button1,
                      bool         L_JoyStick1Button3,
                      bool         L_JoyStick1Button4,
                      double       L_GyroAngleDegrees,
                      double       L_GyroAngleRadians,
                      bool         L_VisionTopTargetAquired,
                      double       L_TopTargetYawDegrees,
                      double      *L_WheelAngleFwd,
                      double      *L_WheelAngleRev,
                      double      *L_WheelSpeedCmnd,
                      double      *L_WheelAngleCmnd,
                      bool        *L_TargetFin,
                      T_RobotState L_RobotState,
                      bool         L_Driver_AutoIntake,
                      double       L_VisionBottomTargetDistanceMeters,
                      bool         L_VisionBottomTargetAquired,
                      double       L_VisionBottomYaw);

