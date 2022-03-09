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
bool   V_b_DriveStraight;
double V_RotateErrorCalc;
double V_Deg_DesiredAngPrev = 0;
double V_WheelAngleError[E_RobotCornerSz];
double V_WheelAngleIntegral[E_RobotCornerSz];
double V_WheelAngleCmnd[E_RobotCornerSz]; // Command sent to motor controller.  Command is power based.
double V_WheelSpeedCmnd[E_RobotCornerSz]; // Command sent to motor controller   Command is either in power or speed request.
double V_WheelSpeedError[E_RobotCornerSz];
double V_WheelSpeedIntergral[E_RobotCornerSz];
double V_WheelAngleArb[E_RobotCornerSz]; // This is the arbitrated wheel angle that is used in the PID controller

bool V_SD_DriverRobotOrientedRequested; // Requested driver mode override
bool V_SD_DriverRobotOrientedRequestedLatched; // Latched state of the driver requested mode
bool V_SD_DriverRobotOrientedRequestedPrev; // Requested driver mode override previous

double V_WheelSpeedPID_V2_Gx[E_PID_SparkMaxCalSz];
double V_SD_WheelSpeedCmndPrev[E_RobotCornerSz];
bool   V_SD_DriveWheelsInPID = false;
double KV_WheelSpeedRampRate = 0;
double V_SD_WheelAngleCmndPrev[E_RobotCornerSz];
double KV_SD_WheelAnglePID_Gx[E_PID_CalSz];
double KV_SD_WheelAngleRampRate = 0;
double V_TestWheelSpeed = 0;
double KV_SD_WheelGx[E_RobotCornerSz];

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
  T_PID_Cal L_Index2 = E_P_Gx;

  // set PID coefficients
  m_frontLeftDrivePID.SetP(K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  m_frontLeftDrivePID.SetI(K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  m_frontLeftDrivePID.SetD(K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  m_frontLeftDrivePID.SetIZone(K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  m_frontLeftDrivePID.SetFF(K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  m_frontLeftDrivePID.SetOutputRange(K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput], K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  m_frontRightDrivePID.SetP(K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  m_frontRightDrivePID.SetI(K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  m_frontRightDrivePID.SetD(K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  m_frontRightDrivePID.SetIZone(K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  m_frontRightDrivePID.SetFF(K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  m_frontRightDrivePID.SetOutputRange(K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput], K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  m_rearLeftDrivePID.SetP(K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  m_rearLeftDrivePID.SetI(K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  m_rearLeftDrivePID.SetD(K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  m_rearLeftDrivePID.SetIZone(K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  m_rearLeftDrivePID.SetFF(K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  m_rearLeftDrivePID.SetOutputRange(K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput], K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  m_rearRightDrivePID.SetP(K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  m_rearRightDrivePID.SetI(K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  m_rearRightDrivePID.SetD(K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  m_rearRightDrivePID.SetIZone(K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  m_rearRightDrivePID.SetFF(K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  m_rearRightDrivePID.SetOutputRange(K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput], K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  KV_WheelSpeedRampRate = K_SD_WheelSpeedPID_V2_Gx[E_kMaxAcc];

  for (L_Index2 = E_P_Gx;
       L_Index2 < E_PID_CalSz;
       L_Index2 = T_PID_Cal(int(L_Index2) + 1))
      {
	  #ifdef CompBot
      KV_SD_WheelAnglePID_Gx[L_Index2] = K_SD_WheelAnglePID_Gx[L_Index2];
	  #endif
	  #ifdef PracticeBot
	  KV_SD_WheelAnglePID_Gx[L_Index2] = K_SD_WheelAnglePID_GxPracticeBot[L_Index2];
	  #endif
      }

T_RobotCorner L_Index3;
  for (L_Index3 = E_FrontLeft;
       L_Index3 < E_RobotCornerSz;
       L_Index3 = T_RobotCorner(int(L_Index3) + 1))
      {
      KV_SD_WheelGx[L_Index3] = K_SD_WheelGx[L_Index3];
      }


  KV_SD_WheelAngleRampRate = 25; // Remove

  #ifdef DriveMotorTest
  T_PID_SparkMaxCal L_Index = E_kP;

  for (L_Index = E_kP;
       L_Index < E_PID_SparkMaxCalSz;
       L_Index = T_PID_SparkMaxCal(int(L_Index) + 1))
      {
      V_WheelSpeedPID_V2_Gx[L_Index] = K_SD_WheelSpeedPID_V2_Gx[L_Index];
      }
  
  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain", K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain", K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone", K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Feed Forward", K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  frc::SmartDashboard::PutNumber("Max Output", K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output", K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("Max Velocity", K_SD_WheelSpeedPID_V2_Gx[E_kMaxVel]);
  frc::SmartDashboard::PutNumber("Min Velocity", K_SD_WheelSpeedPID_V2_Gx[E_kMinVel]);
  frc::SmartDashboard::PutNumber("Max Acceleration", K_SD_WheelSpeedPID_V2_Gx[E_kMaxAcc]);
  frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", K_SD_WheelSpeedPID_V2_Gx[E_kAllErr]);

  frc::SmartDashboard::PutNumber("KV_SD_WheelGx[E_FrontLeft]", KV_SD_WheelGx[E_FrontLeft]);
  frc::SmartDashboard::PutNumber("KV_SD_WheelGx[E_FrontRight]", KV_SD_WheelGx[E_FrontRight]);
  frc::SmartDashboard::PutNumber("KV_SD_WheelGx[E_RearLeft]", KV_SD_WheelGx[E_RearLeft]);
  frc::SmartDashboard::PutNumber("KV_SD_WheelGx[E_RearRight]", KV_SD_WheelGx[E_RearRight]);
  #endif
  #ifdef WheelAngleTest
  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", KV_SD_WheelAnglePID_Gx[E_P_Gx]);
  frc::SmartDashboard::PutNumber("I Gain", KV_SD_WheelAnglePID_Gx[E_I_Gx]);
  frc::SmartDashboard::PutNumber("D Gain", KV_SD_WheelAnglePID_Gx[E_D_Gx]);
  frc::SmartDashboard::PutNumber("P Upper Limit", KV_SD_WheelAnglePID_Gx[E_P_Ul]);
  frc::SmartDashboard::PutNumber("P Lower Limit", KV_SD_WheelAnglePID_Gx[E_P_Ll]);

  frc::SmartDashboard::PutNumber("I Upper Limit", KV_SD_WheelAnglePID_Gx[E_D_Ul]);
  frc::SmartDashboard::PutNumber("I Lower Limit", KV_SD_WheelAnglePID_Gx[E_D_Ll]);

  frc::SmartDashboard::PutNumber("D Upper Limit", KV_SD_WheelAnglePID_Gx[E_D_Ul]);
  frc::SmartDashboard::PutNumber("D Lower Limit", KV_SD_WheelAnglePID_Gx[E_D_Ll]);

  frc::SmartDashboard::PutNumber("Max UL", KV_SD_WheelAnglePID_Gx[E_Max_Ul]);
  frc::SmartDashboard::PutNumber("Max LL", KV_SD_WheelAnglePID_Gx[E_Max_Ll]);
  frc::SmartDashboard::PutNumber("Max Output", KV_SD_WheelAnglePID_Gx[E_Max_Ul]);
  frc::SmartDashboard::PutNumber("Min Output", KV_SD_WheelAnglePID_Gx[E_Max_Ll]);
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

  KV_SD_WheelGx[E_FrontLeft] = frc::SmartDashboard::GetNumber("KV_SD_WheelGx[E_FrontLeft]", KV_SD_WheelGx[E_FrontLeft]);
  KV_SD_WheelGx[E_FrontRight] = frc::SmartDashboard::GetNumber("KV_SD_WheelGx[E_FrontRight]", KV_SD_WheelGx[E_FrontRight]);
  KV_SD_WheelGx[E_RearLeft] = frc::SmartDashboard::GetNumber("KV_SD_WheelGx[E_RearLeft]", KV_SD_WheelGx[E_RearLeft]);
  KV_SD_WheelGx[E_RearRight] = frc::SmartDashboard::GetNumber("KV_SD_WheelGx[E_RearRight]", KV_SD_WheelGx[E_RearRight]);

  if((L_p != V_WheelSpeedPID_V2_Gx[E_kP]))   { m_frontLeftDrivePID.SetP(L_p); m_frontRightDrivePID.SetP(L_p); m_rearLeftDrivePID.SetP(L_p); m_rearRightDrivePID.SetP(L_p); V_WheelSpeedPID_V2_Gx[E_kP] = L_p; }
  if((L_i != V_WheelSpeedPID_V2_Gx[E_kI]))   { m_frontLeftDrivePID.SetI(L_i); m_frontRightDrivePID.SetI(L_i); m_rearLeftDrivePID.SetI(L_i); m_rearRightDrivePID.SetI(L_i); V_WheelSpeedPID_V2_Gx[E_kI] = L_i; }
  if((L_d != V_WheelSpeedPID_V2_Gx[E_kD]))   { m_frontLeftDrivePID.SetD(L_d); m_frontRightDrivePID.SetD(L_d); m_rearLeftDrivePID.SetD(L_d); m_rearRightDrivePID.SetD(L_d); V_WheelSpeedPID_V2_Gx[E_kD] = L_d; }
  if((L_iz != V_WheelSpeedPID_V2_Gx[E_kIz])) { m_frontLeftDrivePID.SetIZone(L_iz); m_frontRightDrivePID.SetIZone(L_iz); m_rearLeftDrivePID.SetIZone(L_iz); m_rearRightDrivePID.SetIZone(L_iz); V_WheelSpeedPID_V2_Gx[E_kIz] = L_iz; }
  if((L_ff != V_WheelSpeedPID_V2_Gx[E_kFF])) { m_frontLeftDrivePID.SetFF(L_ff); m_frontRightDrivePID.SetFF(L_ff); m_rearLeftDrivePID.SetFF(L_ff); m_rearRightDrivePID.SetFF(L_ff); V_WheelSpeedPID_V2_Gx[E_kFF] = L_ff; }
  if((L_max != V_WheelSpeedPID_V2_Gx[E_kMaxOutput]) || (L_min != V_WheelSpeedPID_V2_Gx[E_kMinOutput])) { m_frontLeftDrivePID.SetOutputRange(L_min, L_max); m_frontRightDrivePID.SetOutputRange(L_min, L_max);  m_rearLeftDrivePID.SetOutputRange(L_min, L_max); m_rearRightDrivePID.SetOutputRange(L_min, L_max); V_WheelSpeedPID_V2_Gx[E_kMinOutput] = L_min; V_WheelSpeedPID_V2_Gx[E_kMaxOutput] = L_max; }
  if((L_maxV != V_WheelSpeedPID_V2_Gx[E_kMaxVel])) { V_WheelSpeedPID_V2_Gx[E_kMaxVel] = L_maxV; }
  // if((L_minV != V_WheelSpeedPID_V2_Gx[E_kMinVel])) { m_frontLeftDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_frontRightDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_rearLeftDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_rearRightDrivePID.SetSmartMotionMinOutputVelocity(L_minV); V_WheelSpeedPID_V2_Gx[E_kMinVel] = L_minV; }
  if((L_maxA != V_WheelSpeedPID_V2_Gx[E_kMaxAcc])) { KV_WheelSpeedRampRate = L_maxA; V_WheelSpeedPID_V2_Gx[E_kMaxAcc] = L_maxA; }
  // if((L_allE != V_WheelSpeedPID_V2_Gx[E_kAllErr])) { m_frontLeftDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_frontRightDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_rearLeftDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_rearRightDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); V_WheelSpeedPID_V2_Gx[E_kAllErr] = L_allE; }
  #endif
  #ifdef WheelAngleTest
  KV_SD_WheelAnglePID_Gx[E_P_Gx] = frc::SmartDashboard::GetNumber("P Gain", 0);
  KV_SD_WheelAnglePID_Gx[E_I_Gx] = frc::SmartDashboard::GetNumber("I Gain", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Gx] = frc::SmartDashboard::GetNumber("D Gain", 0);
  KV_SD_WheelAnglePID_Gx[E_P_Ul] = frc::SmartDashboard::GetNumber("P Upper Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_P_Ll] = frc::SmartDashboard::GetNumber("P Lower Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Ul] = frc::SmartDashboard::GetNumber("I Upper Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Ll] = frc::SmartDashboard::GetNumber("I Lower Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Ul] = frc::SmartDashboard::GetNumber("D Upper Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Ll] = frc::SmartDashboard::GetNumber("D Lower Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_Max_Ul] = frc::SmartDashboard::GetNumber("Max UL", 0);
  KV_SD_WheelAnglePID_Gx[E_Max_Ll] = frc::SmartDashboard::GetNumber("Max LL", 0);
  KV_SD_WheelAnglePID_Gx[E_Max_Ul] = frc::SmartDashboard::GetNumber("Max Output", 0);
  KV_SD_WheelAnglePID_Gx[E_Max_Ll] = frc::SmartDashboard::GetNumber("Min Output", 0);
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
        V_SD_WheelSpeedCmndPrev[L_Index] = 0;
        V_SD_WheelAngleCmndPrev[L_Index] = 0;
      }

  V_b_DriveStraight = false;
  V_RotateErrorCalc = 0;
  
  V_SD_DriveWheelsInPID = false;

  V_SD_DriverRobotOrientedRequestedPrev = false;
  V_SD_DriverRobotOrientedRequestedLatched = false;
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
                      bool                L_JoyStick1Button3, // auto rotate to 0 degrees
                      bool                L_JoyStick1Button4, // auto rotate to 90 degrees
                      bool                L_Driver_RobotFieldOrientedReq,
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
                      double             *L_WheelAngleCmnd)
  {
  double L_FWD = 0;
  double L_STR = 0;
  double L_RCW = 0;
  T_RobotCorner L_Index = E_FrontLeft;
  double L_temp;
  double L_A = 0;
  double L_B = 0;
  double L_C = 0;
  double L_D = 0;
  double L_Gain = 0;
  double L_Max = 0;
  double L_WA_FWD = 0;
  double L_WA_FWD_Delta = 0;
  double L_WA_REV = 0;
  double L_WA_REV_Delta = 0;
  double L_WA[E_RobotCornerSz];
  double L_WS[E_RobotCornerSz];
  double L_RotateErrorCalc = 0;
  bool   L_SD_DriveWheelsPowered = false;

  /* Scale the joysticks based on a calibratable lookup when in teleop: */
  if (L_ADAS_ActiveFeature > E_ADAS_Disabled)
    {
    /* ADAS is active, pass throught the commands: */
      L_FWD = -L_ADAS_Pct_SD_FwdRev;
      L_STR = L_ADAS_Pct_SD_Strafe;
      L_RCW = L_ADAS_Pct_SD_Rotate;

      if (L_ADAS_SD_RobotOriented == true)
        {
        /* When true, we want to force the robot into robot orientation (i.e. don't use the gyro) */
        L_GyroAngleDegrees = 0;
        L_GyroAngleRadians = 0;
        }
    }
  else /* In ADAS, just past through the commands: */
    {
    /* ADAS is disabled, use the driver joysticks */
      L_FWD = -L_JoyStick1Axis1Y;
      L_STR = L_JoyStick1Axis1X;
      L_RCW = L_JoyStick1Axis2X;

      /* Check to see what the driver wants for the driver mode: */
      if (L_Driver_RobotFieldOrientedReq != V_SD_DriverRobotOrientedRequestedPrev)
        {
        /* Ok, we seem to have experienced a button press.  Let's flip the latched state*/
        if (V_SD_DriverRobotOrientedRequestedLatched == true)
          {
          V_SD_DriverRobotOrientedRequestedLatched = false; // When false, the driver is requesting to have the robot in field oriented mode
          }
        else
          {
          V_SD_DriverRobotOrientedRequestedLatched = true;  // When true, the driver is requesting to have the robot in robot oriented mode
          }
        }

    /* Save the previous version to help determine when there is a transition in the driver button press. */
      V_SD_DriverRobotOrientedRequestedPrev = L_Driver_RobotFieldOrientedReq;

      if (V_SD_DriverRobotOrientedRequestedLatched == true)
        {
        /* When true, we want to force the robot into robot orientation (i.e. don't use the gyro) */
        L_GyroAngleDegrees = 0;
        L_GyroAngleRadians = 0;
        }

    //turning rotatemode on/off & setting desired angle
    // if ((fabs(L_FWD) > 0) ||
    //     (fabs(L_STR) > 0) ||
    //     (fabs(L_RCW) > 0))
    //   {
    //   // Abort out of auto rotate and/or auto target if the driver moves the joysticks
    //   rotateMode = false;

    //   if (fabs(L_RCW) > 0)
    //     {
    //     // Ok, the driver is attempting to rotate the robot 
    //     desiredAngle = L_GyroAngleDegrees;
    //     V_b_DriveStraight   = false;
    //     }
    //   else
    //     {
    //     // Ok, the driver is attempting to not rotate the robot (i.e. drive straight and/or strafe)
    //     // Set the desired angle of the robot equal to the previously measured/commanded angle and enable rotate mode
    //     desiredAngle = V_Deg_DesiredAngPrev;
    //     V_b_DriveStraight   = true;
    //     }  
    //   }
    // else if (L_JoyStick1Button4)
    //   {
    //   V_b_DriveStraight = false;
    //   rotateMode = true;
    //   desiredAngle = 90;
    //   }
    // else if (L_JoyStick1Button3)
    //   {
    //   V_b_DriveStraight = false;
    //   rotateMode = true;
    //   desiredAngle = 0;
    //   }
    // else if ((fabs(L_FWD) == 0) &&
    //          (fabs(L_STR) == 0) &&
    //          (fabs(L_RCW) == 0))
    //   {
    //   // No driver input
    //   desiredAngle      = L_GyroAngleDegrees;
    //   V_b_DriveStraight = false;
    //   }
      
    // if ((rotateMode == true) ||
    //     (V_b_DriveStraight == true))
    //   {
    //   // Use gyro as target when in auto rotate or drive straight mode
    //   L_RotateErrorCalc = desiredAngle - L_GyroAngleDegrees;
    //   }
    // else
    //   {
    //   L_RotateErrorCalc = 0;
    //   }

    // if ((V_b_DriveStraight     == true           && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce <= K_RotateDebounceTime) ||
    //     (rotateMode            == true           && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce <= K_RotateDebounceTime))
    //   {
    //   rotateDeBounce += C_ExeTime;
    //   }
    // else if ((V_b_DriveStraight     == true      && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce >= K_RotateDebounceTime) ||
    //          (rotateMode            == true      && fabs(L_RotateErrorCalc) <= K_RotateDeadbandAngle && rotateDeBounce >= K_RotateDebounceTime))
    //   {
    //   rotateMode = false;

    //   rotateDeBounce = 0;
    //   }

    // if (rotateMode == true)
    //   {
    //   L_RCW = DesiredRotateSpeed(L_RotateErrorCalc);
    //   }
    // else if (V_b_DriveStraight == true)
    //   {
    //   L_RCW = DesiredAutoRotateSpeed(L_RotateErrorCalc);
    //   }
    // else
    //   {
    //   /* Leave RCW as the driver input value */
    //   }
    }


    /* Swerve drive calculaltions: */
    L_temp =  L_FWD * cos(L_GyroAngleRadians) + L_STR * sin(L_GyroAngleRadians);
    L_STR  = -L_FWD * sin(L_GyroAngleRadians) + L_STR * cos(L_GyroAngleRadians);
    L_FWD  =  L_temp;

    //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr
    L_A = L_STR - L_RCW * (C_SD_L/C_SD_R);
    L_B = L_STR + L_RCW * (C_SD_L/C_SD_R);
    L_C = L_FWD - L_RCW * (C_SD_W/C_SD_R);
    L_D = L_FWD + L_RCW * (C_SD_W/C_SD_R);

    L_WS[E_FrontRight] = pow((L_B * L_B + L_C * L_C), 0.5);
    L_WS[E_FrontLeft]  = pow((L_B * L_B + L_D * L_D), 0.5);
    L_WS[E_RearLeft]   = pow((L_A * L_A + L_D * L_D), 0.5);
    L_WS[E_RearRight]  = pow((L_A * L_A + L_C * L_C), 0.5);

    L_WA[E_FrontRight] = -atan2(L_B, L_C) *180/C_PI;
    L_WA[E_FrontLeft]  = -atan2(L_B, L_D) *180/C_PI;
    L_WA[E_RearLeft]   = -atan2(L_A, L_D) *180/C_PI;
    L_WA[E_RearRight]  = -atan2(L_A, L_C) *180/C_PI;

    // L_WS[E_FrontRight] = pow((L_B * L_B + L_D * L_D), 0.5);
    // L_WS[E_FrontLeft]  = pow((L_B * L_B + L_C * L_C), 0.5);
    // L_WS[E_RearLeft]   = pow((L_A * L_A + L_C * L_C), 0.5);
    // L_WS[E_RearRight]  = pow((L_A * L_A + L_D * L_D), 0.5);

    // L_WA[E_FrontRight] = atan2(L_B, L_D) *180/C_PI;
    // L_WA[E_FrontLeft]  = atan2(L_B, L_C) *180/C_PI;
    // L_WA[E_RearLeft]   = atan2(L_A, L_C) *180/C_PI;
    // L_WA[E_RearRight]  = atan2(L_A, L_D) *180/C_PI;

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

    L_Gain = K_SD_MinGain;
    
    if (L_JoyStick1Axis3 > L_Gain)
      {
      /* Additional speed trigger from driver: */
      L_Gain = L_JoyStick1Axis3;
      }
    else if (rotateMode   == true)
      {
      L_Gain = K_SD_AutoRotateGx;
      }

    if (L_Gain >= K_SD_MaxGain)
      {
      L_Gain = K_SD_MaxGain;
      }

    L_WS[E_FrontRight] *= (K_SD_WheelMaxSpeed * L_Gain);
    L_WS[E_FrontLeft]  *= (K_SD_WheelMaxSpeed * L_Gain);
    L_WS[E_RearLeft]   *= (K_SD_WheelMaxSpeed * L_Gain);
    L_WS[E_RearRight]  *= (K_SD_WheelMaxSpeed * L_Gain);

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
      #ifdef WheelAngleTest
      // L_WA[L_Index] = RampTo(L_WA[L_Index], V_SD_WheelAngleCmndPrev[L_Index], KV_SD_WheelAngleRampRate);

      // V_SD_WheelAngleCmndPrev[L_Index] = L_WA[L_Index];
      #endif

      L_WheelAngleCmnd[L_Index] =  Control_PID( L_WA[L_Index],
                                                V_WheelAngleArb[L_Index],
                                               &V_WheelAngleError[L_Index],
                                               &V_WheelAngleIntegral[L_Index],
                                                KV_SD_WheelAnglePID_Gx[E_P_Gx],
                                                KV_SD_WheelAnglePID_Gx[E_I_Gx],
                                                KV_SD_WheelAnglePID_Gx[E_D_Gx],
                                                KV_SD_WheelAnglePID_Gx[E_P_Ul],
                                                KV_SD_WheelAnglePID_Gx[E_P_Ll],
                                                KV_SD_WheelAnglePID_Gx[E_I_Ul],
                                                KV_SD_WheelAnglePID_Gx[E_I_Ll],
                                                KV_SD_WheelAnglePID_Gx[E_D_Ul],
                                                KV_SD_WheelAnglePID_Gx[E_D_Ll],
                                                KV_SD_WheelAnglePID_Gx[E_Max_Ul],
                                                KV_SD_WheelAnglePID_Gx[E_Max_Ll]);

      L_WheelSpeedCmnd[L_Index] = RampTo((L_WS[L_Index] * KV_SD_WheelGx[L_Index]), V_SD_WheelSpeedCmndPrev[L_Index], KV_WheelSpeedRampRate);
      // L_WheelSpeedCmnd[L_Index] = RampTo(V_WheelSpeedPID_V2_Gx[E_kMaxVel], L_WheelSpeedCmnd[L_Index], KV_WheelSpeedRampRate);
      // L_SD_DriveWheelsPowered = true;
      V_SD_WheelSpeedCmndPrev[L_Index] = L_WheelSpeedCmnd[L_Index];

      if ((fabs(L_WheelSpeedCmnd[L_Index]) >= K_SD_WheelMinCmndSpeed))
        {
        // Ok, so we have at least one wheel that is still trying to command a non zero speed 
        L_SD_DriveWheelsPowered = true;
        }
      }
 
    V_SD_DriveWheelsInPID = L_SD_DriveWheelsPowered;
    V_Deg_DesiredAngPrev = desiredAngle;
    V_RotateErrorCalc = L_RotateErrorCalc;
  }
