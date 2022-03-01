/*
  DriveControl.hpp

   Created on: Feb 25, 2020

   Author: 5561

   Updates:
   2022-02-15: Cleaned up file
 */

extern double V_WheelAngleCmnd[E_RobotCornerSz];
extern double V_WheelSpeedCmnd[E_RobotCornerSz];
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

void DriveControlMain(double              L_JoyStick1Axis1Y,  // swerve control forward/back
                      double              L_JoyStick1Axis1X,  // swerve control strafe
                      double              L_JoyStick1Axis2X,  // rotate the robot joystick
                      double              L_JoyStick1Axis3,   // extra speed trigger
                      bool                L_JoyStick1Button3, // auto rotate to 0 degrees
                      bool                L_JoyStick1Button4, // auto rotate to 90 degrees
                      T_ADAS_ActiveFeature L_ADAS_ActiveFeature,
                      double               L_ADAS_Pct_SD_FwdRev,
                      double               L_ADAS_Pct_SD_Strafe,
                      double               L_ADAS_Pct_SD_Rotate,
                      bool                 L_ADAS_SD_RobotOriented,
                      double              L_GyroAngleDegrees,
                      double              L_GyroAngleRadians,
                      double             *L_WheelAngleFwd,
                      double             *L_WheelAngleRev,
                      double             *L_WheelSpeedCmnd,
                      double             *L_WheelAngleCmnd);

