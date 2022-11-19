/*
  DriveControl.cpp

  Created on: Feb 25, 2020
  Author: 5561

  Contains the calculations and controls for swerve drive ("_SD_").

  Changes:
  2021-02-25 -> Updates to help the robot drive straight
  2022-03-08 -> Resolved rotation and directionality issues
  2022-03-09 -> Added robot oritented mode
 */

#include "rev/CANSparkMax.h"
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Const.hpp"
#include "control_pid.hpp"
#include "Lookup.hpp"

bool V_SD_DriverRobotOrientedRequested; // Requested driver mode override
bool V_SD_DriverRobotOrientedRequestedLatched; // Latched state of the driver requested mode
bool V_SD_DriverRobotOrientedRequestedPrev; // Requested driver mode override previous

double V_deg_SD_RobotDesiredAngle;  // Desired angle of robot.

double V_SD_WheelAngleError[E_RobotCornerSz];  // Error value for PID control.
double V_SD_WheelAngleIntegral[E_RobotCornerSz];  // Integral value for PID control.
double V_SD_WheelAngleArb[E_RobotCornerSz]; // This is the arbitrated wheel angle that is used in the PID controller
double V_SD_WheelAngleCmnd[E_RobotCornerSz]; // Command sent to motor controller.  Command is power based.

double KV_SD_WheelAnglePID_Gx[E_PID_CalSz];

double V_SD_WheelSpeedCmnd[E_RobotCornerSz]; // Command sent to motor controller   Command is either in power or speed request.
double V_SD_WheelSpeedCmndPrev[E_RobotCornerSz];  // Previous wheel speed command.  Used for ramping of control.
bool   V_SD_DriveWheelsInPID = false;  // flag indicating that PID control is currently active in the motor controller.

double KV_SD_WheelSpeedPID_V2_Gx[E_PID_SparkMaxCalSz];
double KV_SD_WheelSpeedRampRate = 0;
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
  T_PID_SparkMaxCal L_Index  = E_kP;
  T_PID_Cal         L_Index2 = E_P_Gx;
  T_RobotCorner     L_Index3 = E_FrontLeft;

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

  KV_SD_WheelSpeedRampRate = K_SD_WheelSpeedPID_V2_Gx[E_kMaxAcc];

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

  for (L_Index3 = E_FrontLeft;
       L_Index3 < E_RobotCornerSz;
       L_Index3 = T_RobotCorner(int(L_Index3) + 1))
      {
      KV_SD_WheelGx[L_Index3] = K_SD_WheelGx[L_Index3];
      }

  #ifdef DriveMotorTest
  for (L_Index = E_kP;
       L_Index < E_PID_SparkMaxCalSz;
       L_Index = T_PID_SparkMaxCal(int(L_Index) + 1))
      {
      KV_SD_WheelSpeedPID_V2_Gx[L_Index] = K_SD_WheelSpeedPID_V2_Gx[L_Index];
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

  if((L_p != KV_SD_WheelSpeedPID_V2_Gx[E_kP]))   { m_frontLeftDrivePID.SetP(L_p); m_frontRightDrivePID.SetP(L_p); m_rearLeftDrivePID.SetP(L_p); m_rearRightDrivePID.SetP(L_p); KV_SD_WheelSpeedPID_V2_Gx[E_kP] = L_p; }
  if((L_i != KV_SD_WheelSpeedPID_V2_Gx[E_kI]))   { m_frontLeftDrivePID.SetI(L_i); m_frontRightDrivePID.SetI(L_i); m_rearLeftDrivePID.SetI(L_i); m_rearRightDrivePID.SetI(L_i); KV_SD_WheelSpeedPID_V2_Gx[E_kI] = L_i; }
  if((L_d != KV_SD_WheelSpeedPID_V2_Gx[E_kD]))   { m_frontLeftDrivePID.SetD(L_d); m_frontRightDrivePID.SetD(L_d); m_rearLeftDrivePID.SetD(L_d); m_rearRightDrivePID.SetD(L_d); KV_SD_WheelSpeedPID_V2_Gx[E_kD] = L_d; }
  if((L_iz != KV_SD_WheelSpeedPID_V2_Gx[E_kIz])) { m_frontLeftDrivePID.SetIZone(L_iz); m_frontRightDrivePID.SetIZone(L_iz); m_rearLeftDrivePID.SetIZone(L_iz); m_rearRightDrivePID.SetIZone(L_iz); KV_SD_WheelSpeedPID_V2_Gx[E_kIz] = L_iz; }
  if((L_ff != KV_SD_WheelSpeedPID_V2_Gx[E_kFF])) { m_frontLeftDrivePID.SetFF(L_ff); m_frontRightDrivePID.SetFF(L_ff); m_rearLeftDrivePID.SetFF(L_ff); m_rearRightDrivePID.SetFF(L_ff); KV_SD_WheelSpeedPID_V2_Gx[E_kFF] = L_ff; }
  if((L_max != KV_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]) || (L_min != KV_SD_WheelSpeedPID_V2_Gx[E_kMinOutput])) { m_frontLeftDrivePID.SetOutputRange(L_min, L_max); m_frontRightDrivePID.SetOutputRange(L_min, L_max);  m_rearLeftDrivePID.SetOutputRange(L_min, L_max); m_rearRightDrivePID.SetOutputRange(L_min, L_max); KV_SD_WheelSpeedPID_V2_Gx[E_kMinOutput] = L_min; KV_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput] = L_max; }
  if((L_maxV != KV_SD_WheelSpeedPID_V2_Gx[E_kMaxVel])) { KV_SD_WheelSpeedPID_V2_Gx[E_kMaxVel] = L_maxV; }
  // if((L_minV != KV_SD_WheelSpeedPID_V2_Gx[E_kMinVel])) { m_frontLeftDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_frontRightDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_rearLeftDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_rearRightDrivePID.SetSmartMotionMinOutputVelocity(L_minV); KV_SD_WheelSpeedPID_V2_Gx[E_kMinVel] = L_minV; }
  if((L_maxA != KV_SD_WheelSpeedPID_V2_Gx[E_kMaxAcc])) { KV_SD_WheelSpeedRampRate = L_maxA; KV_SD_WheelSpeedPID_V2_Gx[E_kMaxAcc] = L_maxA; }
  // if((L_allE != KV_SD_WheelSpeedPID_V2_Gx[E_kAllErr])) { m_frontLeftDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_frontRightDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_rearLeftDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_rearRightDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); KV_SD_WheelSpeedPID_V2_Gx[E_kAllErr] = L_allE; }
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
        V_SD_WheelAngleError[L_Index] = 0;
        V_SD_WheelAngleIntegral[L_Index] = 0;
        V_SD_WheelAngleArb[L_Index] = 0;
        V_SD_WheelSpeedCmndPrev[L_Index] = 0;
      }
  V_SD_DriveWheelsInPID = false;

  V_SD_DriverRobotOrientedRequestedPrev = false;
  V_SD_DriverRobotOrientedRequestedLatched = false;

  V_deg_SD_RobotDesiredAngle = 0;
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
                      T_ADAS_ActiveFeature L_ADAS_ActiveFeature,          // ADAS command
                      double               L_ADAS_Pct_SD_FwdRev,          // ADAS command
                      double               L_ADAS_Pct_SD_Strafe,          // ADAS command
                      double               L_ADAS_Pct_SD_Rotate,          // ADAS command
                      bool                 L_ADAS_SD_RobotOriented,       // ADAS command
                      double              L_GyroAngleDegrees,
                      double              L_GyroAngleRadians,
                      double             *L_WheelAngleFwd,
                      double             *L_WheelAngleRev,
                      double             *LaWheelSpeedCmnd,
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
  bool   L_SD_DriveWheelsPowered = false;
  double Le_SD_RobotAngleError = 0;

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
      if (L_Driver_RobotFieldOrientedReq != V_SD_DriverRobotOrientedRequestedPrev && L_Driver_RobotFieldOrientedReq == true)
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
    }

    /* Let's determine what the "desired" angle of the robot is: */
    // if (fabs(L_RCW) > 0)
    //   {
    //     /* The rotate command is non zero, we are trying to rotate.  Update the "desired" angle */
    //     V_deg_SD_RobotDesiredAngle = L_GyroAngleDegrees;
    //   }

    // /* Determine the error between the desired commanded direction and the robots actual direction */
    // Le_SD_RobotAngleError = 

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

    /* Normalized everything to a max value of 1 for wheel speeds: */
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

    /* Ok, now lets apply gains to the normalized wheel speeds to obtain the desired motor speed */
    L_Gain = K_SD_MinGain;
    
    if (L_JoyStick1Axis3 > L_Gain)
      {
      /* Additional speed trigger from driver: */
      L_Gain = L_JoyStick1Axis3;
      }

    if (L_Gain >= K_SD_MaxGain)
      {
      L_Gain = K_SD_MaxGain;
      }

    L_WS[E_FrontRight] *= (K_SD_WheelMaxSpeed * (-L_Gain));
    L_WS[E_FrontLeft]  *= (K_SD_WheelMaxSpeed * (-L_Gain));
    L_WS[E_RearLeft]   *= (K_SD_WheelMaxSpeed * L_Gain);
    L_WS[E_RearRight]  *= (K_SD_WheelMaxSpeed * (-L_Gain));

    /* Now we need to detrime if we want to keep the desired commanded angle or flip 180* and flip the direction of the wheel speed.  
       This is intended to find the quickest way to reach the commanded angle. */
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
          V_SD_WheelAngleArb[L_Index] = L_WA_FWD;
        }
      else
        {
          V_SD_WheelAngleArb[L_Index] = L_WA_REV;
          L_WS[L_Index] *= (-1); // Need to flip sign of drive wheel to account for reverse direction
        }
      }

    /* Next, let's determine the amount of offset that needs to be applied to each wheel speed in order 
       to help keep the bot pointing in the correct direction. */
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
          V_SD_WheelAngleArb[L_Index] = L_WA_FWD;
        }
      else
        {
          V_SD_WheelAngleArb[L_Index] = L_WA_REV;
          L_WS[L_Index] *= (-1); // Need to flip sign of drive wheel to account for reverse direction
        }
      }

  /* Output the wheel speed and angle commands: */
    for (L_Index = E_FrontLeft;
         L_Index < E_RobotCornerSz;
         L_Index = T_RobotCorner(int(L_Index) + 1))
      {
      /* We do PID control within the Rio for angle control: */
      L_WheelAngleCmnd[L_Index] =  Control_PID( L_WA[L_Index],
                                                V_SD_WheelAngleArb[L_Index],
                                               &V_SD_WheelAngleError[L_Index],
                                               &V_SD_WheelAngleIntegral[L_Index],
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

      /* Wheel speed control resides externally in the independent motor controlers.
      Don't send the final value, ramp to the desired final value to help prevent integral windup and overshoot. */
      // LaWheelSpeedCmnd[L_Index] = RampTo((L_WS[L_Index] * KV_SD_WheelGx[L_Index]), V_SD_WheelSpeedCmndPrev[L_Index], KV_SD_WheelSpeedRampRate);  ToDo: Remove once we know the auto correct works
      LaWheelSpeedCmnd[L_Index] = RampTo(L_WS[L_Index], V_SD_WheelSpeedCmndPrev[L_Index], KV_SD_WheelSpeedRampRate);

      V_SD_WheelSpeedCmndPrev[L_Index] = LaWheelSpeedCmnd[L_Index];

      if ((fabs(LaWheelSpeedCmnd[L_Index]) >= K_SD_WheelMinCmndSpeed))
        {
        /* Ok, so we have at least one wheel that is still trying to command a non zero speed. If not, we want to force it to a zero power 
           command to prevent locking of the wheels or swaying to try and hold a zero speed. */
        L_SD_DriveWheelsPowered = true;
        }
      }
 
    V_SD_DriveWheelsInPID = L_SD_DriveWheelsPowered;

    frc::SmartDashboard::PutNumber("L_WA[E_FrontLeft]", L_WA[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("V_SD_WheelAngleArb[E_FrontLeft]", V_SD_WheelAngleArb[E_FrontLeft]);
    frc::SmartDashboard::PutNumber("L_WheelAngleCmnd[E_FrontLeft]", L_WheelAngleCmnd[E_FrontLeft]);
  }
