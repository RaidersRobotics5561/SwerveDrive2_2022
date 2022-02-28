/*
 * Team 5561 2022 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive (beta 02/10/2020)
 * - Shooting balls (beta 02/15/2020)
 *
 * */

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>

#include "Encoders.hpp"
#include "Gyro.hpp"
#include "IO_Sensors.hpp"
// #include "vision.hpp"
#include "Driver_inputs.hpp"
#include "Odometry.hpp"
#include "DriveControl.hpp"
#include "Lift.hpp"
#include "BallHandler.hpp"
#include "LightControl.hpp"
#include "VisionV2.hpp"
#include "Auton.hpp"
#include "AutoTarget.hpp"

// nt::NetworkTableInstance inst;

T_RobotState                 V_RobotState        = E_Init;
frc::DriverStation::Alliance V_AllianceColor     = frc::DriverStation::Alliance::kInvalid;
double                       V_MatchTimeRemaining = 0;

/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit()
  {
  V_RobotState         = E_Init;
  V_AllianceColor      = frc::DriverStation::GetInstance().GetAlliance();
  V_MatchTimeRemaining = frc::Timer::GetMatchTime().value();

  EncodersInit(m_encoderFrontRightSteer,
               m_encoderFrontLeftSteer,
               m_encoderRearRightSteer,
               m_encoderRearLeftSteer,
               m_encoderFrontRightDrive,
               m_encoderFrontLeftDrive,
               m_encoderRearRightDrive,
               m_encoderRearLeftDrive,
               m_encoderLiftYD,
               m_encoderLiftXD,
               m_encoderrightShooter,
               m_encoderleftShooter);

  GyroInit();

  IO_SensorsInit();

  m_frontLeftSteerMotor.SetSmartCurrentLimit(K_SteerMotorCurrentLimit);
  m_frontRightSteerMotor.SetSmartCurrentLimit(K_SteerMotorCurrentLimit);
  m_rearLeftSteerMotor.SetSmartCurrentLimit(K_SteerMotorCurrentLimit);
  m_rearRightSteerMotor.SetSmartCurrentLimit(K_SteerMotorCurrentLimit);

  m_frontLeftSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_frontLeftDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_frontRightSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_frontRightDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearLeftSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearLeftDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearRightSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearRightDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_liftMotorYD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_liftMotorXD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_rightShooterMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_leftShooterMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  SwerveDriveMotorConfigsInit(m_frontLeftDrivePID,
                              m_frontRightDrivePID,
                              m_rearLeftDrivePID,
                              m_rearRightDrivePID);

  BallHandlerMotorConfigsInit(m_rightShooterpid,
                              m_leftShooterpid);

  LiftMotorConfigsInit(m_liftpidYD,
                       m_liftpidXD);

  VisionInit(V_AllianceColor);

  pc_Camera2.SetPipelineIndex(V_VisionBottomIndex);

  VisionDashboard();

  m_led.SetLength(K_LED_NumberOfLEDs);
  m_led.SetData(m_ledBuffer);
  m_led.Start();

  // inst = nt::NetworkTableInstance::Create();
  // inst.StartClient("10.55.61.24");
  // inst.StartDSClient();
  
// m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
// m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
// frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
// frc::SmartDashboard::PutNumber("cooler int", 1);
  }


/******************************************************************************
 * Function:     RobotPeriodic
 *
 * Description:  Function called periodically (not defined well as to what
 *               "periodically" means).
 ******************************************************************************/
void Robot::RobotPeriodic()
  {
  int L_Index = 0;

  V_MatchTimeRemaining = frc::Timer::GetMatchTime().value();

  Joystick_robot_mapping(c_joyStick2.GetRawButton(1),
                         c_joyStick2.GetRawButton(2),
                         c_joyStick2.GetRawButton(6), 
                         c_joyStick2.GetRawButton(7),
                         c_joyStick2.GetRawButton(8),
                         c_joyStick.GetRawButton(7),
                         c_joyStick.GetRawButton(8),
                         c_joyStick2.GetRawButton(3),
                         c_joyStick2.GetRawButton(4),
                         c_joyStick2.GetRawAxis(1),
                         c_joyStick2.GetRawAxis(5),
                         c_joyStick.GetRawAxis(1),
                         c_joyStick.GetRawAxis(0),
                         c_joyStick.GetRawAxis(4),
                         c_joyStick.GetRawAxis(3),
                         c_joyStick.GetRawButton(1),
                         c_joyStick.GetRawButton(3),
                         c_joyStick.GetRawButton(4),
                         c_joyStick2.GetPOV(),
                         c_joyStick.GetRawButton(6),
                         c_joyStick.GetRawButton(2));

  Read_Encoders(a_encoderWheelAngleFrontLeft.Get().value(),
                a_encoderWheelAngleFrontRight.Get().value(),
                a_encoderWheelAngleRearLeft.Get().value(),
                a_encoderWheelAngleRearRight.Get().value(),
                m_encoderFrontLeftDrive,
                m_encoderFrontRightDrive,
                m_encoderRearLeftDrive,
                m_encoderRearRightDrive,
                m_encoderrightShooter,
                m_encoderleftShooter,
                m_encoderLiftYD,
                m_encoderLiftXD);

  ReadGyro(V_Driver_zero_gyro);

  Read_IO_Sensors(di_IR_Sensor.Get(),
                  di_BallSensorLower.Get(),
                  di_XD_LimitSwitch.Get(),
                  di_XY_LimitSwitch.Get());

  VisionRun(pc_Camera1.GetLatestResult(),
            pc_Camera2.GetLatestResult());

  SwerveDriveMotorConfigsCal(m_frontLeftDrivePID,
                             m_frontRightDrivePID,
                             m_rearLeftDrivePID,
                             m_rearRightDrivePID);

  BallHandlerMotorConfigsCal(m_rightShooterpid,
                             m_leftShooterpid);

  LiftMotorConfigsCal(m_liftpidYD,
                      m_liftpidXD);

  LightControlMain( V_Driver_SwerveGoalAutoCenter,
                    V_Driver_auto_setspeed_shooter,
                    V_MatchTimeRemaining,
                    V_AllianceColor,
                    E_LauncherNotActive,
                    V_SwerveTargetLockingUpper,
                    V_Driver_CameraLight,
                    V_ShooterTargetSpeedReached,
                   &V_CameraLightCmndOn,
                   &V_VanityLED_Red,
                   &V_VanityLED_Green,
                   &V_VanityLED_Blue);

  for (L_Index = 0; L_Index < K_LED_NumberOfLEDs; L_Index++)
    {
    m_ledBuffer[L_Index].SetRGB(V_VanityLED_Red, V_VanityLED_Green, V_VanityLED_Blue);
    }

  m_led.SetData(m_ledBuffer);
  do_CameraLightControl.Set(V_CameraLightCmndOn);

  frc::SmartDashboard::PutBoolean("XD Limit Detected", V_XD_LimitDetected);
  frc::SmartDashboard::PutBoolean("YD Limit Detected", V_YD_LimitDetected);
  frc::SmartDashboard::PutBoolean("Ball Detected", V_BallDetectedUpper);

  frc::SmartDashboard::PutNumber("Lift postition YD", V_LiftPostitionYD);
  frc::SmartDashboard::PutNumber("Lift postition XD", V_LiftPostitionXD);

  frc::SmartDashboard::PutNumber("V_b_DriveStraight", V_b_DriveStraight);
  frc::SmartDashboard::PutNumber("V_RotateErrorCalc", V_RotateErrorCalc);
  frc::SmartDashboard::PutNumber("Speed Cmnd", V_ShooterRPM_Cmnd);

  frc::SmartDashboard::PutNumber("GYRO",                 V_GyroYawAngleDegrees);
  frc::SmartDashboard::PutBoolean("Ball Detected Lower", V_BallDetectedLower);

  frc::SmartDashboard::PutBoolean("Top Target?",    V_VisionTopTargetAquired);
  frc::SmartDashboard::PutNumber("Top Yaw",         V_VisionTopYaw);
  frc::SmartDashboard::PutNumber("Top Distance",    V_VisionTopTargetDistanceMeters);
  frc::SmartDashboard::PutNumber("Bottom Range",    V_VisionBottomTargetDistanceMeters);
  frc::SmartDashboard::PutBoolean("Bottom Target?", V_VisionBottomTargetAquired);
  frc::SmartDashboard::PutNumber("Bottom Yaw",      V_VisionBottomYaw);
  frc::SmartDashboard::PutNumber("Bottom Index",    V_VisionBottomIndex); 

  frc::SmartDashboard::PutNumber("Lift YD S0",  V_LiftMotorYD_MaxCurrent[E_S0_BEGONE]);
  frc::SmartDashboard::PutNumber("Lift YD S2",  V_LiftMotorYD_MaxCurrent[E_S2_lift_down_YD]);
  frc::SmartDashboard::PutNumber("Lift YD S3",  V_LiftMotorYD_MaxCurrent[E_S3_move_forward_XD]);
  frc::SmartDashboard::PutNumber("Lift YD S4",  V_LiftMotorYD_MaxCurrent[E_S4_stretch_up_YD]);
  frc::SmartDashboard::PutNumber("Lift YD S5",  V_LiftMotorYD_MaxCurrent[E_S5_more_forward_XD]);
  frc::SmartDashboard::PutNumber("Lift YD S6",  V_LiftMotorYD_MaxCurrent[E_S6_lift_up_more_YD]);
  frc::SmartDashboard::PutNumber("Lift YD S7",  V_LiftMotorYD_MaxCurrent[E_S7_move_back_XD]);
  frc::SmartDashboard::PutNumber("Lift YD S8",  V_LiftMotorYD_MaxCurrent[E_S8_more_down_some_YD]);
  frc::SmartDashboard::PutNumber("Lift YD S9",  V_LiftMotorYD_MaxCurrent[E_S9_back_rest_XD]);
  frc::SmartDashboard::PutNumber("Lift YD S10", V_LiftMotorYD_MaxCurrent[E_S10_final_YD]);
  
  frc::SmartDashboard::PutNumber("Lift XD S0",  V_LiftMotorXD_MaxCurrent[E_S0_BEGONE]);
  frc::SmartDashboard::PutNumber("Lift XD S2",  V_LiftMotorXD_MaxCurrent[E_S2_lift_down_YD]);
  frc::SmartDashboard::PutNumber("Lift XD S3",  V_LiftMotorXD_MaxCurrent[E_S3_move_forward_XD]);
  frc::SmartDashboard::PutNumber("Lift XD S4",  V_LiftMotorXD_MaxCurrent[E_S4_stretch_up_YD]);
  frc::SmartDashboard::PutNumber("Lift XD S5",  V_LiftMotorXD_MaxCurrent[E_S5_more_forward_XD]);
  frc::SmartDashboard::PutNumber("Lift XD S6",  V_LiftMotorXD_MaxCurrent[E_S6_lift_up_more_YD]);
  frc::SmartDashboard::PutNumber("Lift XD S7",  V_LiftMotorXD_MaxCurrent[E_S7_move_back_XD]);
  frc::SmartDashboard::PutNumber("Lift XD S8",  V_LiftMotorXD_MaxCurrent[E_S8_more_down_some_YD]);
  frc::SmartDashboard::PutNumber("Lift XD S9",  V_LiftMotorXD_MaxCurrent[E_S9_back_rest_XD]);
  frc::SmartDashboard::PutNumber("Lift XD S10", V_LiftMotorXD_MaxCurrent[E_S10_final_YD]);

  frc::SmartDashboard::PutNumber("Launcher Speed",    V_ShooterSpeedCurr);
  }


/******************************************************************************
 * Function:     AutonomousInit
 *S
 * Description:  Function called at init while in autonomous.  This is where we
 *               should zero out anything that we need to before autonomous mode.
 ******************************************************************************/
void Robot::AutonomousInit()
  { 
    V_RobotState = E_Auton;
    V_AllianceColor = frc::DriverStation::GetInstance().GetAlliance();
    
    DriveControlInit();
    BallHandlerInit();
    LiftControlInit();
    AutonDriveReset();
    OdometryInit();
    VisionInit(V_AllianceColor);
    pc_Camera2.SetPipelineIndex(V_VisionBottomIndex);
  }


/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Function called periodically in autonomous.  This is where we
 *               should place our primary autonomous control code.
 ******************************************************************************/
void Robot::AutonomousPeriodic()
  {
  double driveforward = 0;
  double strafe = 0;
  double speen = 0;
    
    DtrmnSwerveBotLocation(V_GyroYawAngleRad,
                           &V_Rad_WheelAngleFwd[0],
                           &V_M_WheelDeltaDistance[0]);
  
    AutonDriveMain();

    DriveControlMain( driveforward,
                      strafe,
                      speen,
                      0, // May need to add additional speed control/command
                      V_autonTargetCmd,
                      c_joyStick.GetRawButton(3),
                      c_joyStick.GetRawButton(4),
                      V_GyroYawAngleDegrees,
                      V_GyroYawAngleRad,
                      V_VisionTopTargetAquired,
                      V_VisionTopYaw,
                     &V_WheelAngleFwd[0],
                     &V_WheelAngleRev[0],
                     &V_WheelSpeedCmnd[0],
                     &V_WheelAngleCmnd[0],
                     &V_autonTargetFin,
                      V_RobotState,
                      V_Driver_AutoIntake,
                      V_VisionBottomTargetDistanceMeters,
                      V_VisionBottomTargetAquired,
                      V_VisionBottomYaw);

  // Motor output commands:
    #ifdef DriveMotorTest
    m_frontLeftDrivePID.SetReference(V_WheelSpeedCmnd[E_FrontLeft],   rev::ControlType::kSmartVelocity);
    m_frontRightDrivePID.SetReference(V_WheelSpeedCmnd[E_FrontRight], rev::ControlType::kSmartVelocity);
    m_rearLeftDrivePID.SetReference(V_WheelSpeedCmnd[E_RearLeft],     rev::ControlType::kSmartVelocity);
    m_rearRightDrivePID.SetReference(V_WheelSpeedCmnd[E_RearRight],   rev::ControlType::kSmartVelocity);
    #endif
    #ifndef DriveMotorTest
    m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);
    #endif

    m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft]);
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight]);
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft]);
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight]);

    if (V_BH_LauncherActive == true)
      {
      m_rightShooterpid.SetReference(V_ShooterRPM_Cmnd, rev::ControlType::kVelocity);
      m_leftShooterpid.SetReference(-V_ShooterRPM_Cmnd, rev::ControlType::kVelocity);
      }
    else
      {
      m_rightShooterMotor.Set(0);
      m_leftShooterMotor.Set(0);
      }

    m_intake.Set(ControlMode::PercentOutput, V_IntakePowerCmnd); //must be positive (don't be a fool)
    m_elevator.Set(ControlMode::PercentOutput, V_ElevatorPowerCmnd);

    m_liftpidYD.SetReference(V_lift_command_YD, rev::ControlType::kSmartMotion); // positive is up
    m_liftpidXD.SetReference(V_lift_command_XD, rev::ControlType::kSmartMotion); // This is temporary.  We actually want to use position, but need to force this off temporarily
  }


/******************************************************************************
 * Function:     TeleopInit
 *
 * Description:  Function called when starting out in teleop mode.
 *               We should zero out all of our global varibles.
 ******************************************************************************/
void Robot::TeleopInit()
  {
  V_RobotState = E_Teleop;
  V_AllianceColor = frc::DriverStation::GetInstance().GetAlliance();

  DriveControlInit();
  BallHandlerInit();
  LiftControlInit();
  OdometryInit();
  VisionInit(V_AllianceColor);
  m_encoderrightShooter.SetPosition(0);
  m_encoderleftShooter.SetPosition(0);
}


/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  Primary function called when in teleop mode.
 ******************************************************************************/
void Robot::TeleopPeriodic()
  {
  DtrmnSwerveBotLocation(V_GyroYawAngleRad,
                         &V_Rad_WheelAngleFwd[0],
                         &V_M_WheelDeltaDistance[0]);

  DriveControlMain( V_Driver_SwerveForwardBack,
                    V_Driver_SwerveStrafe,
                    V_Driver_SwerveRotate,
                    V_Driver_SwerveSpeed,
                    V_Driver_SwerveGoalAutoCenter,
                    V_Driver_SwerveRotateTo0,
                    V_Driver_SwerveRotateTo90,
                    V_GyroYawAngleDegrees,
                    V_GyroYawAngleRad,
                    V_VisionTopTargetAquired,
                    V_VisionTopYaw,
                   &V_WheelAngleFwd[0],
                   &V_WheelAngleRev[0],
                   &V_WheelSpeedCmnd[0],
                   &V_WheelAngleCmnd[0],
                   &V_autonTargetFin,
                    V_RobotState,
                    V_Driver_AutoIntake,
                    V_VisionBottomTargetDistanceMeters,
                    V_VisionBottomTargetAquired,
                    V_VisionBottomYaw);

  V_Lift_state = Lift_Control_Dictator(V_Driver_lift_control,
                                       V_Driver_StopShooterAutoClimbResetGyro,
                                       V_Driver_Lift_Cmnd_Direction,
                                       V_MatchTimeRemaining,
                                       V_Lift_state,
                                       V_LiftPostitionYD,
                                       V_LiftPostitionXD,
                                       &V_lift_command_YD,
                                       &V_lift_command_XD,
                                       V_GyroYawAngleDegrees,
                                       m_liftMotorYD.GetOutputCurrent(),
                                       m_liftMotorXD.GetOutputCurrent());

  BallHandlerControlMain( V_Driver_intake_in,
                          V_Driver_intake_out,
                          V_BallDetectedUpper,
                          V_Driver_elevator_up,
                          V_Driver_elevator_down,
                          V_Driver_StopShooterAutoClimbResetGyro,
                          V_Driver_auto_setspeed_shooter,
                          V_ShooterSpeedCurr,
                          V_Driver_manual_shooter_desired_speed,
                          0, // auto shoot command
                         &V_IntakePowerCmnd,
                         &V_ElevatorPowerCmnd,
                         &V_ShooterRPM_Cmnd);

  // Motor output commands:
    #ifdef DriveMotorTest
    if (V_SD_DriveWheelsInPID == true)
      {
      m_frontLeftDrivePID.SetReference(V_WheelSpeedCmnd[E_FrontLeft],   rev::ControlType::kVelocity);
      m_frontRightDrivePID.SetReference(V_WheelSpeedCmnd[E_FrontRight], rev::ControlType::kVelocity);
      m_rearLeftDrivePID.SetReference(V_WheelSpeedCmnd[E_RearLeft],     rev::ControlType::kVelocity);
      m_rearRightDrivePID.SetReference(V_WheelSpeedCmnd[E_RearRight],   rev::ControlType::kVelocity);
      }
    else
      {
      m_frontLeftDriveMotor.Set(0);
      m_frontRightDriveMotor.Set(0);
      m_rearLeftDriveMotor.Set(0);
      m_rearRightDriveMotor.Set(0);
      }
    #endif
    #ifndef DriveMotorTest
    m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);
    #endif

    m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft]);
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight]);
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft]);
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight]);

    if (V_BH_LauncherActive == true)
      {
      m_rightShooterpid.SetReference(V_ShooterRPM_Cmnd, rev::ControlType::kVelocity);
      m_leftShooterpid.SetReference(-V_ShooterRPM_Cmnd, rev::ControlType::kVelocity);
      }
    else
      {
      m_rightShooterMotor.Set(0);
      m_leftShooterMotor.Set(0);
      }


    m_intake.Set(ControlMode::PercentOutput, V_IntakePowerCmnd); //must be positive (don't be a fool)
    m_elevator.Set(ControlMode::PercentOutput, V_ElevatorPowerCmnd);

    m_liftpidYD.SetReference(V_lift_command_YD, rev::ControlType::kPosition); // positive is up
    m_liftpidXD.SetReference(V_lift_command_XD, rev::ControlType::kPosition); // This is temporary.  We actually want to use position, but need to force this off temporarily

    do_CameraLightControl.Set(V_CameraLightCmndOn);
}


/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  Called during the test phase initiated on the driver station.
 ******************************************************************************/
void Robot::TestPeriodic()
  {
  Lift_Control_ManualOverride(&V_LiftYD_TestPowerCmnd,
                              &V_LiftXD_TestPowerCmnd,
                               m_liftMotorYD.GetOutputCurrent(),
                               m_liftMotorXD.GetOutputCurrent(),
                               V_Driver_Lift_Cmnd_Direction,
                               V_YD_LimitDetected,
                               V_XD_LimitDetected);
 
  m_liftMotorYD.Set(V_LiftYD_TestPowerCmnd);
  m_liftMotorXD.Set(V_LiftXD_TestPowerCmnd);

  if (V_Driver_StopShooterAutoClimbResetGyro == true)
    {
    EncodersInit(m_encoderFrontRightSteer,
                 m_encoderFrontLeftSteer,
                 m_encoderRearRightSteer,
                 m_encoderRearLeftSteer,
                 m_encoderFrontRightDrive,
                 m_encoderFrontLeftDrive,
                 m_encoderRearRightDrive,
                 m_encoderRearLeftDrive,
                 m_encoderLiftYD,
                 m_encoderLiftXD,
                 m_encoderrightShooter,
                 m_encoderleftShooter);
    }

  m_frontLeftDriveMotor.Set(0);
  m_frontRightDriveMotor.Set(0);
  m_rearLeftDriveMotor.Set(0);
  m_rearRightDriveMotor.Set(0);

  m_frontLeftSteerMotor.Set(0);
  m_frontRightSteerMotor.Set(0);
  m_rearLeftSteerMotor.Set(0);
  m_rearRightSteerMotor.Set(0);

  m_rightShooterMotor.Set(0);
  m_leftShooterMotor.Set(0);

  m_intake.Set(ControlMode::PercentOutput, 0);
  m_elevator.Set(ControlMode::PercentOutput, 0);

  do_CameraLightControl.Set(V_CameraLightCmndOn); // I believe this is backwards, so true is off??
  }


#ifndef RUNNING_FRC_TESTS
/******************************************************************************
 * Function:     main
 *
 * Description:  This is the main calling function for the robot.
 ******************************************************************************/
int main()
  {
    return frc::StartRobot<Robot>();
  }
#endif
