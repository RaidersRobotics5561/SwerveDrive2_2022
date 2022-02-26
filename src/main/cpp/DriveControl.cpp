/*
  DriveControl.cpp

  Created on: Feb 25, 2020
  Author: 5561

  Changes:
  2021-02-25 -> Updates to help the robot drive straight
 */

#include "rev/CANSparkMax.h"
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Const.hpp"
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Encoders.hpp"

double desiredAngle;
double rotateDeBounce;
double rotateErrorCalc;
bool   rotateMode;
bool   V_SwerveTargetLockingUpper;
bool   V_b_DriveStraight;
double V_RotateErrorCalc;
bool   V_AutoRotateComplete;
double V_Deg_DesiredAngPrev = 0;
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
double V_WheelAngleCmnd[E_RobotCornerSz];
double V_WheelSpeedCmnd[E_RobotCornerSz];
double V_WheelSpeedError[E_RobotCornerSz];
double V_WheelSpeedIntergral[E_RobotCornerSz];
double V_WheelAngleArb[E_RobotCornerSz]; // This is the arbitrated wheel angle that is used in the PID controller

double V_WheelSpeedPID_V2_Gx[E_PID_SparkMaxCalSz];

T_GetDaBalls V_AutoIntake;

/******************************************************************************
 * Function:     SwerveDriveMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the swerve drive motors.
 *               - Steer (power cmnd only)
 *               - Drive (PID Control in motor controller)
 ******************************************************************************/
void SwerveDriveMotorConfigsInit(rev::SparkMaxPIDController m_frontLeftDrivePID,
                                 rev::SparkMaxPIDController m_frontRightDrivePID,
                                 rev::SparkMaxPIDController m_rearLeftDrivePID,
                                 rev::SparkMaxPIDController m_rearRightDrivePID)
  {
  // set PID coefficients
  m_frontLeftDrivePID.SetP(K_WheelSpeedPID_V2_Gx[E_kP]);
  m_frontLeftDrivePID.SetI(K_WheelSpeedPID_V2_Gx[E_kI]);
  m_frontLeftDrivePID.SetD(K_WheelSpeedPID_V2_Gx[E_kD]);
  m_frontLeftDrivePID.SetIZone(K_WheelSpeedPID_V2_Gx[E_kIz]);
  m_frontLeftDrivePID.SetFF(K_WheelSpeedPID_V2_Gx[E_kFF]);
  m_frontLeftDrivePID.SetOutputRange(K_WheelSpeedPID_V2_Gx[E_kMinOutput], K_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  m_frontRightDrivePID.SetP(K_WheelSpeedPID_V2_Gx[E_kP]);
  m_frontRightDrivePID.SetI(K_WheelSpeedPID_V2_Gx[E_kI]);
  m_frontRightDrivePID.SetD(K_WheelSpeedPID_V2_Gx[E_kD]);
  m_frontRightDrivePID.SetIZone(K_WheelSpeedPID_V2_Gx[E_kIz]);
  m_frontRightDrivePID.SetFF(K_WheelSpeedPID_V2_Gx[E_kFF]);
  m_frontRightDrivePID.SetOutputRange(K_WheelSpeedPID_V2_Gx[E_kMinOutput], K_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  m_rearLeftDrivePID.SetP(K_WheelSpeedPID_V2_Gx[E_kP]);
  m_rearLeftDrivePID.SetI(K_WheelSpeedPID_V2_Gx[E_kI]);
  m_rearLeftDrivePID.SetD(K_WheelSpeedPID_V2_Gx[E_kD]);
  m_rearLeftDrivePID.SetIZone(K_WheelSpeedPID_V2_Gx[E_kIz]);
  m_rearLeftDrivePID.SetFF(K_WheelSpeedPID_V2_Gx[E_kFF]);
  m_rearLeftDrivePID.SetOutputRange(K_WheelSpeedPID_V2_Gx[E_kMinOutput], K_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  m_rearRightDrivePID.SetP(K_WheelSpeedPID_V2_Gx[E_kP]);
  m_rearRightDrivePID.SetI(K_WheelSpeedPID_V2_Gx[E_kI]);
  m_rearRightDrivePID.SetD(K_WheelSpeedPID_V2_Gx[E_kD]);
  m_rearRightDrivePID.SetIZone(K_WheelSpeedPID_V2_Gx[E_kIz]);
  m_rearRightDrivePID.SetFF(K_WheelSpeedPID_V2_Gx[E_kFF]);
  m_rearRightDrivePID.SetOutputRange(K_WheelSpeedPID_V2_Gx[E_kMinOutput], K_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  /**
   * Smart Motion coefficients are set on a SparkMaxPIDController object
   * 
   * - SetSmartMotionMaxVelocity() will limit the velocity in RPM of
   * the pid controller in Smart Motion mode
   * - SetSmartMotionMinOutputVelocity() will put a lower bound in
   * RPM of the pid controller in Smart Motion mode
   * - SetSmartMotionMaxAccel() will limit the acceleration in RPM^2
   * of the pid controller in Smart Motion mode
   * - SetSmartMotionAllowedClosedLoopError() will set the max allowed
   * error for the pid controller in Smart Motion mode
   */
  m_frontLeftDrivePID.SetSmartMotionMaxVelocity(K_WheelSpeedPID_V2_Gx[E_kMaxVel]);
  m_frontLeftDrivePID.SetSmartMotionMinOutputVelocity(K_WheelSpeedPID_V2_Gx[E_kMinVel]);
  m_frontLeftDrivePID.SetSmartMotionMaxAccel(K_WheelSpeedPID_V2_Gx[E_kMaxAcc]);
  m_frontLeftDrivePID.SetSmartMotionAllowedClosedLoopError(K_WheelSpeedPID_V2_Gx[E_kAllErr]);

  m_frontRightDrivePID.SetSmartMotionMaxVelocity(K_WheelSpeedPID_V2_Gx[E_kMaxVel]);
  m_frontRightDrivePID.SetSmartMotionMinOutputVelocity(K_WheelSpeedPID_V2_Gx[E_kMinVel]);
  m_frontRightDrivePID.SetSmartMotionMaxAccel(K_WheelSpeedPID_V2_Gx[E_kMaxAcc]);
  m_frontRightDrivePID.SetSmartMotionAllowedClosedLoopError(K_WheelSpeedPID_V2_Gx[E_kAllErr]);
  
  m_rearLeftDrivePID.SetSmartMotionMaxVelocity(K_WheelSpeedPID_V2_Gx[E_kMaxVel]);
  m_rearLeftDrivePID.SetSmartMotionMinOutputVelocity(K_WheelSpeedPID_V2_Gx[E_kMinVel]);
  m_rearLeftDrivePID.SetSmartMotionMaxAccel(K_WheelSpeedPID_V2_Gx[E_kMaxAcc]);
  m_rearLeftDrivePID.SetSmartMotionAllowedClosedLoopError(K_WheelSpeedPID_V2_Gx[E_kAllErr]);

  m_rearRightDrivePID.SetSmartMotionMaxVelocity(K_WheelSpeedPID_V2_Gx[E_kMaxVel]);
  m_rearRightDrivePID.SetSmartMotionMinOutputVelocity(K_WheelSpeedPID_V2_Gx[E_kMinVel]);
  m_rearRightDrivePID.SetSmartMotionMaxAccel(K_WheelSpeedPID_V2_Gx[E_kMaxAcc]);
  m_rearRightDrivePID.SetSmartMotionAllowedClosedLoopError(K_WheelSpeedPID_V2_Gx[E_kAllErr]);

  #ifdef DriveMotorTest
  T_PID_SparkMaxCal L_Index = E_kP;

  for (L_Index = E_kP;
       L_Index < E_PID_SparkMaxCalSz;
       L_Index = T_PID_SparkMaxCal(int(L_Index) + 1))
      {
      V_WheelSpeedPID_V2_Gx[L_Index] = K_WheelSpeedPID_V2_Gx[L_Index];
      }
  
  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", K_WheelSpeedPID_V2_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain", K_WheelSpeedPID_V2_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain", K_WheelSpeedPID_V2_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone", K_WheelSpeedPID_V2_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Feed Forward", K_WheelSpeedPID_V2_Gx[E_kFF]);
  frc::SmartDashboard::PutNumber("Max Output", K_WheelSpeedPID_V2_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output", K_WheelSpeedPID_V2_Gx[E_kMinOutput]);

  // display Smart Motion coefficients
  frc::SmartDashboard::PutNumber("Max Velocity", K_WheelSpeedPID_V2_Gx[E_kMaxVel]);
  frc::SmartDashboard::PutNumber("Min Velocity", K_WheelSpeedPID_V2_Gx[E_kMinVel]);
  frc::SmartDashboard::PutNumber("Max Acceleration", K_WheelSpeedPID_V2_Gx[E_kMaxAcc]);
  frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", K_WheelSpeedPID_V2_Gx[E_kAllErr]);
  #endif
  }


/******************************************************************************
 * Function:     SwerveDriveMotorConfigsCal
 *
 * Description:  Contains the motor configurations for the swerve drive motors.
 *               - Steer (power cmnd only)
 *               - Drive (PID Control in motor controller)
 ******************************************************************************/
void SwerveDriveMotorConfigsCal(rev::SparkMaxPIDController m_frontLeftDrivePID,
                                rev::SparkMaxPIDController m_frontRightDrivePID,
                                rev::SparkMaxPIDController m_rearLeftDrivePID,
                                rev::SparkMaxPIDController m_rearRightDrivePID)
  {
  // read PID coefficients from SmartDashboard
  #ifdef DriveMotorTest
  double L_p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double L_i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double L_d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double L_iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double L_ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double L_max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double L_min = frc::SmartDashboard::GetNumber("Min Output", 0);
  double L_maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
  double L_minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
  double L_maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
  double L_allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

  if((L_p != V_WheelSpeedPID_V2_Gx[E_kP]))   { m_frontLeftDrivePID.SetP(L_p); m_frontRightDrivePID.SetP(L_p); m_rearLeftDrivePID.SetP(L_p); m_rearRightDrivePID.SetP(L_p); V_WheelSpeedPID_V2_Gx[E_kP] = L_p; }
  if((L_i != V_WheelSpeedPID_V2_Gx[E_kI]))   { m_frontLeftDrivePID.SetI(L_i); m_frontRightDrivePID.SetI(L_i); m_rearLeftDrivePID.SetI(L_i); m_rearRightDrivePID.SetI(L_i); V_WheelSpeedPID_V2_Gx[E_kI] = L_i; }
  if((L_d != V_WheelSpeedPID_V2_Gx[E_kD]))   { m_frontLeftDrivePID.SetD(L_d); m_frontRightDrivePID.SetD(L_d); m_rearLeftDrivePID.SetD(L_d); m_rearRightDrivePID.SetD(L_d); V_WheelSpeedPID_V2_Gx[E_kD] = L_d; }
  if((L_iz != V_WheelSpeedPID_V2_Gx[E_kIz])) { m_frontLeftDrivePID.SetIZone(L_iz); m_frontRightDrivePID.SetIZone(L_iz); m_rearLeftDrivePID.SetIZone(L_iz); m_rearRightDrivePID.SetIZone(L_iz); V_WheelSpeedPID_V2_Gx[E_kIz] = L_iz; }
  if((L_ff != V_WheelSpeedPID_V2_Gx[E_kFF])) { m_frontLeftDrivePID.SetFF(L_ff); m_frontRightDrivePID.SetFF(L_ff); m_rearLeftDrivePID.SetFF(L_ff); m_rearRightDrivePID.SetFF(L_ff); V_WheelSpeedPID_V2_Gx[E_kFF] = L_ff; }
  if((L_max != V_WheelSpeedPID_V2_Gx[E_kMaxOutput]) || (L_min != K_LauncherPID_Gx[E_kMinOutput])) { m_frontLeftDrivePID.SetOutputRange(L_min, L_max); m_frontRightDrivePID.SetOutputRange(L_min, L_max);  m_rearLeftDrivePID.SetOutputRange(L_min, L_max); m_rearRightDrivePID.SetOutputRange(L_min, L_max); V_WheelSpeedPID_V2_Gx[E_kMinOutput] = L_min; V_WheelSpeedPID_V2_Gx[E_kMaxOutput] = L_max; }
  if((L_maxV != V_WheelSpeedPID_V2_Gx[E_kMaxVel])) { m_frontLeftDrivePID.SetSmartMotionMaxVelocity(L_maxV); m_frontRightDrivePID.SetSmartMotionMaxVelocity(L_maxV); m_rearLeftDrivePID.SetSmartMotionMaxVelocity(L_maxV); m_rearRightDrivePID.SetSmartMotionMaxVelocity(L_maxV); V_WheelSpeedPID_V2_Gx[E_kMaxVel] = L_maxV; }
  if((L_minV != V_WheelSpeedPID_V2_Gx[E_kMinVel])) { m_frontLeftDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_frontRightDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_rearLeftDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_rearRightDrivePID.SetSmartMotionMinOutputVelocity(L_minV); V_WheelSpeedPID_V2_Gx[E_kMinVel] = L_minV; }
  if((L_maxA != V_WheelSpeedPID_V2_Gx[E_kMaxAcc])) { m_frontLeftDrivePID.SetSmartMotionMaxAccel(L_maxA); m_frontRightDrivePID.SetSmartMotionMaxAccel(L_maxA); m_rearLeftDrivePID.SetSmartMotionMaxAccel(L_maxA); m_rearRightDrivePID.SetSmartMotionMaxAccel(L_maxA); V_WheelSpeedPID_V2_Gx[E_kMaxAcc] = L_maxA; }
  if((L_allE != V_WheelSpeedPID_V2_Gx[E_kAllErr])) { m_frontLeftDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_frontRightDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_rearLeftDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_rearRightDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); V_WheelSpeedPID_V2_Gx[E_kAllErr] = L_allE; }
  #endif
  }


/******************************************************************************
 * Function:     DriveControlInit
 *
 * Description:  Initialization function for the drive control.
 ******************************************************************************/
void DriveControlInit()
  {
    int L_Index;

      for (L_Index = E_FrontLeft;
           L_Index < E_RobotCornerSz;
           L_Index = T_RobotCorner(int(L_Index) + 1))
      {
        V_WheelAngleError[L_Index] = 0;
        V_WheelAngleIntegral[L_Index] = 0;
        V_WheelSpeedError[L_Index] = 0;
        V_WheelSpeedIntergral[L_Index] = 0;
        V_WheelAngleArb[L_Index] = 0;
      }
  
  V_SwerveTargetLockingUpper = false;

  V_b_DriveStraight = false;
  V_RotateErrorCalc = 0;
  
  V_AutoIntake = E_GetDaOff;
  }


/******************************************************************************
 * Function:     DtrmnEncoderRelativeToCmnd
 *
 * Description:  tbd
 ******************************************************************************/
double DtrmnEncoderRelativeToCmnd(double          L_JoystickCmnd,
                                  double          L_EncoderReading)
  {
    double L_Opt1;
    double L_Opt2;
    double L_Opt3;
    double L_Output;

    L_Opt1 = fabs(L_JoystickCmnd - L_EncoderReading);
    L_Opt2 = fabs(L_JoystickCmnd - (L_EncoderReading + 360));
    L_Opt3 = fabs(L_JoystickCmnd - (L_EncoderReading - 360));

    if ((L_Opt1 < L_Opt2) && (L_Opt1 < L_Opt3))
      {
        L_Output = L_EncoderReading;
      }
    else if ((L_Opt2 < L_Opt1) && (L_Opt2 < L_Opt3))
      {
        L_Output = L_EncoderReading + 360;
      }
    else
      {
        L_Output = L_EncoderReading - 360;
      }

    return (L_Output);
  }


/************************************************************************************************************************
 * Function:     DriveControlMain
 *
 * Description:  Main calling function for the drive control.
 *               Swerve srive calculations obtained from the following link:
 *               https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
 ************************************************************************************************************************/
void DriveControlMain(double              L_JoyStick1Axis1Y,  // swerve control forward/back
                      double              L_JoyStick1Axis1X,  // swerve control strafe
                      double              L_JoyStick1Axis2X,  // rotate the robot joystick
                      double              L_JoyStick1Axis3,   // extra speed trigger
                      bool                L_UpperTargetButtonCentering, // goal auto center
                      bool                L_JoyStick1Button3, // auto rotate to 0 degrees
                      bool                L_JoyStick1Button4, // auto rotate to 90 degrees
                      double              L_GyroAngleDegrees,
                      double              L_GyroAngleRadians,
                      bool                L_VisionTopTargetAquired,
                      double              L_TopTargetYawDegrees,
                      double             *L_WheelAngleFwd,
                      double             *L_WheelAngleRev,
                      double             *L_WheelSpeedCmnd,
                      double             *L_WheelAngleCmnd,
                      bool               *L_TargetFin,
                      T_RobotState        L_RobotState,
                      bool                L_Driver_AutoIntake,
                      double              L_VisionBottomTargetDistanceMeters,
                      bool                L_VisionBottomTargetAquired,
                      double              L_VisionBottomYaw)
  {
  double L_FWD = 0;
  double L_STR = 0;
  double L_RCW = 0;
  int    L_Index;
  double L_temp;
  double L_A;
  double L_B;
  double L_C;
  double L_D;
  double L_Gain;
  double L_Max;
  double L_WA_FWD;
  double L_WA_FWD_Delta;
  double L_WA_REV;
  double L_WA_REV_Delta;
  double L_WA[E_RobotCornerSz];
  double L_WS[E_RobotCornerSz];
  double L_RotateErrorCalc;
  double L_JoyStick1Axis1Y_Scaled;
  double L_JoyStick1Axis1X_Scaled;
  double L_JoyStick1Axis2X_Scaled;

  /* Scale the joysticks based on a calibratable lookup when in teleop: */
  if (L_RobotState == E_Teleop)
    {
      L_JoyStick1Axis1Y_Scaled = ScaleJoystickAxis(L_JoyStick1Axis1Y);
      L_JoyStick1Axis1X_Scaled = ScaleJoystickAxis(-L_JoyStick1Axis1X);
      L_JoyStick1Axis2X_Scaled = ScaleJoystickAxis(-L_JoyStick1Axis2X);
    }
  else /* In auton, just past through the commands: */
    {
      L_JoyStick1Axis1Y_Scaled = L_JoyStick1Axis1Y;
      L_JoyStick1Axis1X_Scaled = -L_JoyStick1Axis1X;
      L_JoyStick1Axis2X_Scaled = L_JoyStick1Axis2X;
    }

  /* Let's place a deadband around the joystick readings */
  L_FWD = L_JoyStick1Axis1Y_Scaled * -1;
  L_STR = L_JoyStick1Axis1X_Scaled;
  L_RCW = L_JoyStick1Axis2X_Scaled;

   //turning rotatemode on/off & setting desired angle
    if ((fabs(L_JoyStick1Axis1Y_Scaled) > 0) ||
        (fabs(L_JoyStick1Axis1X_Scaled) > 0) ||
        (fabs(L_JoyStick1Axis2X_Scaled) > 0))
      {
      // Abort out of auto rotate and/or auto target if the driver moves the joysticks
      V_SwerveTargetLockingUpper = false;
      V_AutoIntake = E_GetDaOff;
      V_AutoRotateComplete = false;
      rotateMode = false;

      if (fabs(L_JoyStick1Axis2X_Scaled) > 0)
        {
        // Ok, the driver is attempting to rotate the robot 
        desiredAngle = L_GyroAngleDegrees;
        V_b_DriveStraight   = false;
        }
      else
        {
        // Ok, the driver is attempting to not rotate the robot (i.e. drive straight and/or strafe)
        // Set the desired angle of the robot equal to the previously measured/commanded angle and enable rotate mode
        desiredAngle = V_Deg_DesiredAngPrev;
        V_b_DriveStraight   = true;
        }  
      }
    else if(L_UpperTargetButtonCentering || V_SwerveTargetLockingUpper == true)
      {
      /* Auto targeting Shooter */
      V_b_DriveStraight = false;
      V_SwerveTargetLockingUpper = true;
      desiredAngle = K_TargetVisionAngleUpper; // This is due to the offset of the camera
      }
    else if(L_Driver_AutoIntake || V_AutoIntake == E_GetDaRotation)
      {
      /* Auto targeting Balls */
      V_b_DriveStraight = false;
      V_AutoIntake = E_GetDaRotation;
      desiredAngle = K_TargetVisionAngleLower; // This is due to the offset of the camera
      }
    else if (L_JoyStick1Button4)
      {
      V_b_DriveStraight = false;
      rotateMode = true;
      desiredAngle = 90;
      }
    else if (L_JoyStick1Button3)
      {
      V_b_DriveStraight = false;
      rotateMode = true;
      desiredAngle = 0;
      }
    else if ((fabs(L_JoyStick1Axis1Y_Scaled) == 0) &&
             (fabs(L_JoyStick1Axis1X_Scaled) == 0) &&
             (fabs(L_JoyStick1Axis2X_Scaled) == 0))
      {
      // No driver input
      desiredAngle      = L_GyroAngleDegrees;
      V_b_DriveStraight = false;
      }
      
    if ((rotateMode == true) ||
        (V_b_DriveStraight == true))
      {
      // Use gyro as target when in auto rotate or drive straight mode
      L_RotateErrorCalc = desiredAngle - L_GyroAngleDegrees;
      }
    else if((V_SwerveTargetLockingUpper == true) &&
            (L_VisionTopTargetAquired == true))
      {
      // Use photon vison as target when in auto beam lock
      L_RotateErrorCalc = desiredAngle - L_TopTargetYawDegrees;
      V_AutoRotateComplete = false;
      }
    else if((V_AutoIntake == E_GetDaRotation) &&
            (L_VisionBottomTargetAquired == true))
      {
      // Use photon vison as target when in auto beam lock
      L_RotateErrorCalc = desiredAngle - L_VisionBottomYaw;
      V_AutoRotateComplete = false;
      }
    else
      {
      L_RotateErrorCalc = 0;
      }
    
    if ((V_b_DriveStraight     == true           && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce <= K_RotateDebounceTime) ||
        (rotateMode            == true           && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce <= K_RotateDebounceTime) || 
        (V_SwerveTargetLockingUpper == true      && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce <= K_RotateDebounceTime) ||
        (V_AutoIntake == E_GetDaRotation         && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce <= K_RotateDebounceTime))
      {
      V_AutoRotateComplete = false;
      // V_SwerveTargetLockingUpper = true;
      rotateDeBounce += C_ExeTime;
      }
    else if ((V_b_DriveStraight     == true      && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce >= K_RotateDebounceTime) ||
             (rotateMode            == true      && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce >= K_RotateDebounceTime) ||
             (V_SwerveTargetLockingUpper == true && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce >= K_RotateDebounceTime) ||
             (V_AutoIntake == E_GetDaRotation    && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce >= K_RotateDebounceTime))
      {
      rotateMode = false;
      V_SwerveTargetLockingUpper = false;
      V_AutoIntake = E_GetDaOff;
      rotateDeBounce = 0;
      V_AutoRotateComplete = true;
      *L_TargetFin = true;
      }

    if (rotateMode == true)
      {
      L_RCW = DesiredRotateSpeed(L_RotateErrorCalc);
      }
    else if (V_b_DriveStraight == true)
      {
      L_RCW = DesiredAutoRotateSpeed(L_RotateErrorCalc);
      }
    else if (V_SwerveTargetLockingUpper == true)
      {
      L_RCW = -DesiredRotateSpeed(L_RotateErrorCalc);
      }
    else if (V_AutoIntake == E_GetDaRotation)
      {
      L_RCW = -DesiredRotateSpeed(L_RotateErrorCalc);
      }
  
    L_temp =  L_FWD * cos(L_GyroAngleRadians) + L_STR * sin(L_GyroAngleRadians);
    L_STR  = -L_FWD * sin(L_GyroAngleRadians) + L_STR * cos(L_GyroAngleRadians);
    L_FWD  =  L_temp;

    //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr
    L_A = L_STR - L_RCW * (C_L/C_R);
    L_B = L_STR + L_RCW * (C_L/C_R);
    L_C = L_FWD - L_RCW * (C_W/C_R);
    L_D = L_FWD + L_RCW * (C_W/C_R);

    L_WS[E_FrontRight] = pow((L_B * L_B + L_C * L_C), 0.5);
    L_WS[E_FrontLeft]  = pow((L_B * L_B + L_D * L_D), 0.5);
    L_WS[E_RearLeft]   = pow((L_A * L_A + L_D * L_D), 0.5);
    L_WS[E_RearRight]  = pow((L_A * L_A + L_C * L_C), 0.5);

    // L_WA[E_FrontRight] = atan2(L_B, L_C) *180/C_PI;
    // L_WA[E_FrontLeft]  = atan2(L_B, L_D) *180/C_PI;
    // L_WA[E_RearLeft]   = atan2(L_A, L_D) *180/C_PI;
    // L_WA[E_RearRight]  = atan2(L_A, L_C) *180/C_PI;

    L_WA[E_FrontRight] = atan2(L_B, L_D) *180/C_PI;
    L_WA[E_FrontLeft]  = atan2(L_B, L_C) *180/C_PI;
    L_WA[E_RearLeft]   = atan2(L_A, L_C) *180/C_PI;
    L_WA[E_RearRight]  = atan2(L_A, L_D) *180/C_PI;

    L_Max = L_WS[E_FrontRight];

    if (L_WS[E_FrontLeft] > L_Max)
      {
      L_Max = L_WS[E_FrontLeft];
      }
    if (L_WS[E_RearLeft] > L_Max)
      {
      L_Max = L_WS[E_RearLeft];
      }
    if (L_WS[E_RearRight] > L_Max)
      {
      L_Max = L_WS[E_RearRight];
      }

    if (L_Max > 1)
      {
      L_WS[E_FrontRight] /= L_Max;
      L_WS[E_FrontLeft]  /= L_Max;
      L_WS[E_RearLeft]   /= L_Max;
      L_WS[E_RearRight]  /= L_Max;
      }

    L_Gain = K_RotateDebounceThreshold;

    if (L_JoyStick1Axis3 > L_Gain)
      {
      L_Gain = L_JoyStick1Axis3;
      }
    else if ((rotateMode   == true) ||
             (V_SwerveTargetLockingUpper == true) ||
             (V_AutoIntake == E_GetDaRotation))
      {
      L_Gain = K_AutoRotateGx;
      }

    if (L_Gain >= K_MaxGain)
      {
      L_Gain = K_MaxGain;
      }

    L_WS[E_FrontRight] *= (K_WheelMaxSpeed * L_Gain);
    L_WS[E_FrontLeft]  *= (K_WheelMaxSpeed * L_Gain);
    L_WS[E_RearLeft]   *= (K_WheelMaxSpeed * L_Gain);
    L_WS[E_RearRight]  *= (K_WheelMaxSpeed * L_Gain);

    for (L_Index = E_FrontLeft;
         L_Index < E_RobotCornerSz;
         L_Index = T_RobotCorner(int(L_Index) + 1))
      {
      L_WA_FWD = DtrmnEncoderRelativeToCmnd(L_WA[L_Index],
                                            L_WheelAngleFwd[L_Index]);

      L_WA_FWD_Delta = fabs(L_WA[L_Index] - L_WA_FWD);

      L_WA_REV = DtrmnEncoderRelativeToCmnd(L_WA[L_Index],
                                            L_WheelAngleRev[L_Index]);

      L_WA_REV_Delta = fabs(L_WA[L_Index] - L_WA_REV);

      if (L_WA_FWD_Delta <= L_WA_REV_Delta)
        {
          V_WheelAngleArb[L_Index] = L_WA_FWD;
        }
      else
        {
          V_WheelAngleArb[L_Index] = L_WA_REV;
          L_WS[L_Index] *= (-1); // Need to flip sign of drive wheel to account for reverse direction
        }
      }

  /* Output the wheel speed and angle targets along with init state: */
    for (L_Index = E_FrontLeft;
         L_Index < E_RobotCornerSz;
         L_Index = T_RobotCorner(int(L_Index) + 1))
      {
      L_WheelAngleCmnd[L_Index] =  Control_PID( L_WA[L_Index],
                                                V_WheelAngleArb[L_Index],
                                               &V_WheelAngleError[L_Index],
                                               &V_WheelAngleIntegral[L_Index],
                                                K_WheelAnglePID_Gx[E_P_Gx],
                                                K_WheelAnglePID_Gx[E_I_Gx],
                                                K_WheelAnglePID_Gx[E_D_Gx],
                                                K_WheelAnglePID_Gx[E_P_Ul],
                                                K_WheelAnglePID_Gx[E_P_Ll],
                                                K_WheelAnglePID_Gx[E_I_Ul],
                                                K_WheelAnglePID_Gx[E_I_Ll],
                                                K_WheelAnglePID_Gx[E_D_Ul],
                                                K_WheelAnglePID_Gx[E_D_Ll],
                                                K_WheelAnglePID_Gx[E_Max_Ul],
                                                K_WheelAnglePID_Gx[E_Max_Ll]);
      #ifdef DriveMotorTest
      L_WheelSpeedCmnd[L_Index] = L_WS[L_Index];
      #endif
      #ifndef DriveMotorTest
      L_WheelSpeedCmnd[L_Index] = Control_PID( L_WS[L_Index],
                                               V_WheelVelocity[L_Index],
                                              &V_WheelSpeedError[L_Index],
                                              &V_WheelSpeedIntergral[L_Index],
                                               K_WheelSpeedPID_Gx[E_P_Gx],
                                               K_WheelSpeedPID_Gx[E_I_Gx],
                                               K_WheelSpeedPID_Gx[E_D_Gx],
                                               K_WheelSpeedPID_Gx[E_P_Ul],
                                               K_WheelSpeedPID_Gx[E_P_Ll],
                                               K_WheelSpeedPID_Gx[E_I_Ul],
                                               K_WheelSpeedPID_Gx[E_I_Ll],
                                               K_WheelSpeedPID_Gx[E_D_Ul],
                                               K_WheelSpeedPID_Gx[E_D_Ll],
                                               K_WheelSpeedPID_Gx[E_Max_Ul],
                                               K_WheelSpeedPID_Gx[E_Max_Ll]);
      #endif
      }

    V_Deg_DesiredAngPrev = desiredAngle;
    V_RotateErrorCalc = L_RotateErrorCalc;
  }
