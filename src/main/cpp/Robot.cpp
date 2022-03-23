/*
 * Team 5561 2022 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive (beta 02/10/2020)
 * - Shooting balls without vision (beta 02/26/2022)
 * - Climber active (beta 02/26/2022)
 *
 * */

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


#include "Encoders.hpp"
#include "Gyro.hpp"
#include "IO_Sensors.hpp"
#include "Driver_inputs.hpp"
#include "Odometry.hpp"
#include "DriveControl.hpp"
#include "Lift.hpp"
#include "BallHandler.hpp"
#include "LightControl.hpp"
#include "VisionV2.hpp"
#include "Auton.hpp"
#include "AutoTarget.hpp"
#include "ADAS.hpp"
#include "ADAS_BT.hpp"
#include "ADAS_UT.hpp"
#include "ADAS_DM.hpp"
 
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

  // frc::CameraServer::StartAutomaticCapture();  // For connecting a single USB camera directly to RIO

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

  m_frontLeftSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);
  m_frontRightSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);
  m_rearLeftSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);
  m_rearRightSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);

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

  ADAS_Main_Init();
  ADAS_Main_Reset();

  ADAS_DM_ConfigsInit();
  ADAS_UT_ConfigsInit();
  ADAS_BT_ConfigsInit();

  VisionRobotInit();

  VisionInit(V_AllianceColor);
  }


/******************************************************************************
 * Function:     RobotPeriodic
 *
 * Description:  Function called periodically (not defined well as to what
 *               "periodically" means).
 ******************************************************************************/
void Robot::RobotPeriodic()
  {
  V_MatchTimeRemaining = frc::Timer::GetMatchTime().value();

  Joystick_robot_mapping(c_joyStick2.GetRawButton(1),
                         c_joyStick2.GetRawButton(2),
                         c_joyStick2.GetRawButton(6), 
                         c_joyStick2.GetRawButton(5),
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
                         c_joyStick.GetRawButton(2),
                         c_joyStick.GetRawButton(5),
                         c_joyStick2.GetRawButton(7));

  Read_Encoders(a_encoderWheelAngleFrontLeft.Get().value(),
                a_encoderWheelAngleFrontRight.Get().value(),
                a_encoderWheelAngleRearLeft.Get().value(),
                a_encoderWheelAngleRearRight.Get().value(),
                a_encoderFrontLeftSteer.GetVoltage(),
                a_encoderFrontRightSteer.GetVoltage(),
                a_encoderRearLeftSteer.GetVoltage(),
                a_encoderRearRightSteer.GetVoltage(),
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

  DtrmnSwerveBotLocation( V_GyroYawAngleRad,
                         &V_Rad_WheelAngleFwd[0],
                         &V_M_WheelDeltaDistance[0]);

  ADAS_DetermineMode();

  V_ADAS_ActiveFeature = ADAS_ControlMain(&V_ADAS_Pct_SD_FwdRev,
                                          &V_ADAS_Pct_SD_Strafe,
                                          &V_ADAS_Pct_SD_Rotate,
                                          &V_ADAS_RPM_BH_Launcher,
                                          &V_ADAS_Pct_BH_Intake,
                                          &V_ADAS_Pct_BH_Elevator,
                                          &V_ADAS_CameraUpperLightCmndOn,
                                          &V_ADAS_CameraLowerLightCmndOn,
                                          &V_ADAS_SD_RobotOriented,
                                          &V_ADAS_Vision_RequestedTargeting,
                                           V_Driver_JoystickActive,
                                           V_Driver_StopShooterAutoClimbResetGyro,
                                           V_Driver_SwerveGoalAutoCenter,
                                           V_Driver_AutoIntake,
                                           V_GyroYawAngleDegrees,
                                           V_l_RobotDisplacementX,
                                           V_l_RobotDisplacementY,
                                           V_VisionTargetAquired[E_CamTop],
                                           V_VisionYaw[E_CamTop],
                                           V_VisionTargetDistanceMeters[E_CamTop],
                                           V_VisionTargetAquired[E_CamBottom],
                                           V_VisionYaw[E_CamBottom],
                                           V_VisionTargetDistanceMeters[E_CamBottom],
                                           V_RobotState,
                                           V_ShooterSpeedCurr,
                                           V_BallDetectedUpper,
                                           V_BallDetectedLower,
                                           V_Driver_elevator_up,
                                           V_Driver_elevator_down,
                                           V_Driver_intake_in,
                                           V_ADAS_ActiveFeature);

  DriveControlMain( V_Driver_SwerveForwardBack,  // swerve control forward/back
                    V_Driver_SwerveStrafe,  // swerve control strafe
                    V_Driver_SwerveRotate,  // rotate the robot joystick
                    V_Driver_SwerveSpeed,   // extra speed trigger
                    V_Driver_SwerveRotateTo0, // auto rotate to 0 degrees
                    V_Driver_SwerveRotateTo90, // auto rotate to 90 degrees
                    V_Driver_RobotFieldOrientedReq,
                    V_ADAS_ActiveFeature,
                    V_ADAS_Pct_SD_FwdRev,
                    V_ADAS_Pct_SD_Strafe,
                    V_ADAS_Pct_SD_Rotate,
                    V_ADAS_SD_RobotOriented,
                    V_GyroYawAngleDegrees,
                    V_GyroYawAngleRad,
                   &V_WheelAngleFwd[0],
                   &V_WheelAngleRev[0],
                   &V_SD_WheelSpeedCmnd[0],
                   &V_SD_WheelAngleCmnd[0]);

  BallHandlerControlMain( V_Driver_intake_in,
                          V_Driver_intake_out,
                          V_BallDetectedUpper,
                          V_BallDetectedLower,
                          V_Driver_elevator_up,
                          V_Driver_elevator_down,
                          V_Driver_StopShooterAutoClimbResetGyro,
                          V_Driver_auto_setspeed_shooter,
                          V_ShooterSpeedCurr,
                          V_Driver_manual_shooter_desired_speed,
                          V_ADAS_ActiveFeature,
                          V_ADAS_RPM_BH_Launcher,
                          V_ADAS_Pct_BH_Intake,
                          V_ADAS_Pct_BH_Elevator,
                         &V_IntakePowerCmnd,
                         &V_ElevatorPowerCmnd,
                         &V_ShooterRPM_Cmnd);

  LightControlMain(V_MatchTimeRemaining,
                   V_AllianceColor,
                   V_Driver_CameraLight,
                   V_ADAS_ActiveFeature,
                   V_ADAS_CameraUpperLightCmndOn,
                   V_ADAS_CameraLowerLightCmndOn,
                  &V_CameraLightCmndOn,
                  &V_VanityLightCmnd);

  VisionRun( pc_Camera1.GetLatestResult(),
             pc_Camera2.GetLatestResult(),
             V_ADAS_Vision_RequestedTargeting,
             V_Driver_VisionDriverModeOverride,
            &V_VisionDriverModeCmndFinal);

  pc_Camera1.SetDriverMode(V_VisionDriverModeCmndFinal);
  pc_Camera2.SetDriverMode(V_VisionDriverModeCmndFinal);

  if (V_VisionDriverModeCmndFinal == false)
    {
    // pc_Camera1.SetPipelineIndex(V_VisionCameraIndex[E_Cam1]);  // Shouldn't need this one so long as Cam1 remains as top
    pc_Camera2.SetPipelineIndex(V_VisionCameraIndex[E_Cam2]);  // Need to comment this out if Photon Vision is being calibrated/tweaked
    }

  V_Lift_state = Lift_Control_Dictator(V_Driver_lift_control,
                                       V_Driver_StopShooterAutoClimbResetGyro,
                                       V_Driver_Lift_Cmnd_Direction,
                                       V_MatchTimeRemaining,
                                       V_Lift_state,
                                       V_LiftPostitionYD,
                                       V_LiftPostitionXD,
                                       &V_lift_command_YD,
                                       &V_lift_command_XD,
                                       &V_LiftYD_TestPowerCmnd,
                                       &V_LiftXD_TestPowerCmnd,
                                       V_YD_LimitDetected,
                                       V_XD_LimitDetected,
                                       V_GyroYawAngleDegrees,
                                       m_liftMotorYD.GetOutputCurrent(),
                                       m_liftMotorXD.GetOutputCurrent(),
                                       m_encoderLiftYD,
                                       m_encoderLiftXD);

  /* These function calls are for test mode calibration. */
  SwerveDriveMotorConfigsCal(m_frontLeftDrivePID,
                             m_frontRightDrivePID,
                             m_rearLeftDrivePID,
                             m_rearRightDrivePID);

  BallHandlerMotorConfigsCal(m_rightShooterpid,
                             m_leftShooterpid);

  LiftMotorConfigsCal(m_liftpidYD,
                      m_liftpidXD);

  ADAS_UT_ConfigsCal();

  ADAS_BT_ConfigsCal();

/* Output all of the content to the dashboard here: */
  // frc::SmartDashboard::PutBoolean("XD Limit Detected", V_XD_LimitDetected);
  // frc::SmartDashboard::PutBoolean("YD Limit Detected", V_YD_LimitDetected);
  frc::SmartDashboard::PutBoolean("Ball Detected Upper", V_BallDetectedUpper);
  frc::SmartDashboard::PutBoolean("Ball Detected Lower", V_BallDetectedLower);
  frc::SmartDashboard::PutBoolean("Lift Ready to Advance", V_Lift_WaitingForDriverINS);
  frc::SmartDashboard::PutNumber("V_WheelAngleConverted[E_FrontRight]", V_WheelAngleConverted[E_FrontRight]);
  frc::SmartDashboard::PutNumber("V_ADAS_BT_State", V_ADAS_BT_State);
  
  // frc::SmartDashboard::PutNumber("Lift postition YD", V_LiftPostitionYD);
  // frc::SmartDashboard::PutNumber("Lift postition XD", V_LiftPostitionXD);

  // frc::SmartDashboard::PutNumber("Launcher Speed Cmnd",      V_ShooterRPM_Cmnd);
  // frc::SmartDashboard::PutNumber("Launcher Speed Actual",    V_ShooterSpeedCurr);
  // frc::SmartDashboard::PutNumber("Front Left Speed Cmnd", m_encoderFrontLeftDrive.GetVelocity());
  frc::SmartDashboard::PutNumber("GYRO",            V_GyroYawAngleDegrees);

  frc::SmartDashboard::PutBoolean("Top Target?",    V_VisionTargetAquired[E_CamTop]);
  frc::SmartDashboard::PutNumber("Top Yaw",         V_VisionYaw[E_CamTop]);
  frc::SmartDashboard::PutNumber("Top Distance",    V_VisionTargetDistanceMeters[E_CamTop]);
    // frc::SmartDashboard::PutNumber("Cam1 Index",    float(V_VisionCameraIndex[E_Cam1]));
  // frc::SmartDashboard::PutNumber("Bottom Range",    V_VisionTargetDistanceMeters[E_CamBottom]);
  frc::SmartDashboard::PutBoolean("Bottom Target?", V_VisionTargetAquired[E_CamBottom]);
  frc::SmartDashboard::PutNumber("Bottom Yaw",      V_VisionYaw[E_CamBottom]);
  frc::SmartDashboard::PutNumber("Cam2 Index",    float(V_VisionCameraIndex[E_Cam2])); 
// frc::SmartDashboard::PutNumber("V_VisionTopCamNumberTemp", V_VisionTopCamNumberTemp);

  // frc::SmartDashboard::PutNumber("Camera 1 Pipeline Index", float(pc_Camera1.GetPipelineIndex()));
  // frc::SmartDashboard::PutNumber("Camera 2 Pipeline Index", float(pc_Camera2.GetPipelineIndex()));

  // frc::SmartDashboard::PutNumber("ADAS ActiveFeature",     float(V_ADAS_ActiveFeature));
  // frc::SmartDashboard::PutNumber("ADAS SD_FwdRev",               V_ADAS_Pct_SD_FwdRev);
  // frc::SmartDashboard::PutNumber("ADAS SD_Strafe",               V_ADAS_Pct_SD_Strafe);
  // frc::SmartDashboard::PutNumber("ADAS SD_Rotate",               V_ADAS_Pct_SD_Rotate);
  // frc::SmartDashboard::PutNumber("ADAS RPM_BH_Launcher",         V_ADAS_RPM_BH_Launcher);
  // frc::SmartDashboard::PutNumber("ADAS BH_Intake",               V_ADAS_Pct_BH_Intake);
  // frc::SmartDashboard::PutNumber("ADAS BH_Elevator",             V_ADAS_Pct_BH_Elevator);
  // frc::SmartDashboard::PutBoolean("ADAS CameraUpperLightCmndOn", V_ADAS_CameraUpperLightCmndOn);
  // frc::SmartDashboard::PutBoolean("ADAS CameraLowerLightCmndOn", V_ADAS_CameraLowerLightCmndOn);
  // frc::SmartDashboard::PutBoolean("ADAS SD_RobotOriented",       V_ADAS_SD_RobotOriented);

  // frc::SmartDashboard::PutBoolean("SD Drive Wheels In PID",      V_SD_DriveWheelsInPID);

  // frc::SmartDashboard::PutNumber("JoystickY",               V_Driver_SwerveSpeed);
  
  // frc::SmartDashboard::PutNumber("Lift YD S0",  V_LiftMotorYD_MaxCurrent[E_S0_BEGONE]);
  // frc::SmartDashboard::PutNumber("Lift YD S2",  V_LiftMotorYD_MaxCurrent[E_S2_lift_down_YD]);
  // frc::SmartDashboard::PutNumber("Lift YD S3",  V_LiftMotorYD_MaxCurrent[E_S3_move_forward_XD]);
  // frc::SmartDashboard::PutNumber("Lift YD S4",  V_LiftMotorYD_MaxCurrent[E_S4_stretch_up_YD]);
  // frc::SmartDashboard::PutNumber("Lift YD S5",  V_LiftMotorYD_MaxCurrent[E_S5_more_forward_XD]);
  // frc::SmartDashboard::PutNumber("Lift YD S6",  V_LiftMotorYD_MaxCurrent[E_S6_lift_up_more_YD]);
  // frc::SmartDashboard::PutNumber("Lift YD S7",  V_LiftMotorYD_MaxCurrent[E_S7_move_back_XD]);
  // frc::SmartDashboard::PutNumber("Lift YD S8",  V_LiftMotorYD_MaxCurrent[E_S8_more_down_some_YD]);
  // frc::SmartDashboard::PutNumber("Lift YD S9",  V_LiftMotorYD_MaxCurrent[E_S9_back_rest_XD]);
  // frc::SmartDashboard::PutNumber("Lift YD S10", V_LiftMotorYD_MaxCurrent[E_S10_final_YD]);
  
  // frc::SmartDashboard::PutNumber("Lift XD S0",  V_LiftMotorXD_MaxCurrent[E_S0_BEGONE]);
  // frc::SmartDashboard::PutNumber("Lift XD S2",  V_LiftMotorXD_MaxCurrent[E_S2_lift_down_YD]);
  // frc::SmartDashboard::PutNumber("Lift XD S3",  V_LiftMotorXD_MaxCurrent[E_S3_move_forward_XD]);
  // frc::SmartDashboard::PutNumber("Lift XD S4",  V_LiftMotorXD_MaxCurrent[E_S4_stretch_up_YD]);
  // frc::SmartDashboard::PutNumber("Lift XD S5",  V_LiftMotorXD_MaxCurrent[E_S5_more_forward_XD]);
  // frc::SmartDashboard::PutNumber("Lift XD S6",  V_LiftMotorXD_MaxCurrent[E_S6_lift_up_more_YD]);
  // frc::SmartDashboard::PutNumber("Lift XD S7",  V_LiftMotorXD_MaxCurrent[E_S7_move_back_XD]);
  // frc::SmartDashboard::PutNumber("Lift XD S8",  V_LiftMotorXD_MaxCurrent[E_S8_more_down_some_YD]);
  // frc::SmartDashboard::PutNumber("Lift XD S9",  V_LiftMotorXD_MaxCurrent[E_S9_back_rest_XD]);
  // frc::SmartDashboard::PutNumber("Lift XD S10", V_LiftMotorXD_MaxCurrent[E_S10_final_YD]);
  frc::SmartDashboard::PutNumber("V_ADAS_UT_State",  float(V_ADAS_UT_State));
  frc::SmartDashboard::PutBoolean("Robot Oriented Control?",    V_SD_DriverRobotOrientedRequestedLatched);

  /* Set light control outputs here */
  do_CameraLightControl.Set(V_CameraLightCmndOn);
  m_vanityLightControler.Set(V_VanityLightCmnd);
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
    GyroInit();
    DriveControlInit();
    BallHandlerInit();
    LiftControlInit();
    ADAS_Main_Reset();
    OdometryInit();
    VisionInit(V_AllianceColor);
  }


/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Function called periodically in autonomous.  This is where we
 *               should place our primary autonomous control code.
 ******************************************************************************/
void Robot::AutonomousPeriodic()
  {
  // Motor output commands:
  if (V_SD_DriveWheelsInPID == true)
    {
    m_frontLeftDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_FrontLeft],   rev::ControlType::kVelocity);
    m_frontRightDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_FrontRight], rev::ControlType::kVelocity);
    m_rearLeftDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_RearLeft],     rev::ControlType::kVelocity);
    m_rearRightDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_RearRight],   rev::ControlType::kVelocity);
    }
  else
    {
    m_frontLeftDriveMotor.Set(0);
    m_frontRightDriveMotor.Set(0);
    m_rearLeftDriveMotor.Set(0);
    m_rearRightDriveMotor.Set(0);
    }

    m_frontLeftSteerMotor.Set(V_SD_WheelAngleCmnd[E_FrontLeft]);
    m_frontRightSteerMotor.Set(V_SD_WheelAngleCmnd[E_FrontRight]);
    m_rearLeftSteerMotor.Set(V_SD_WheelAngleCmnd[E_RearLeft]);
    m_rearRightSteerMotor.Set(V_SD_WheelAngleCmnd[E_RearRight]);

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

  if (V_LiftInitialized == false)
    {
    m_liftMotorYD.Set(V_LiftYD_TestPowerCmnd);
    m_liftMotorXD.Set(V_LiftXD_TestPowerCmnd);
    }
  else
    {
    m_liftpidYD.SetReference(V_lift_command_YD, rev::ControlType::kPosition); // positive is up
    m_liftpidXD.SetReference(V_lift_command_XD, rev::ControlType::kPosition); // This is temporary.  We actually want to use position, but need to force this off temporarily
    }
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

  ADAS_Main_Reset();
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
  // Motor output commands:
  if (V_SD_DriveWheelsInPID == true)
    {
    m_frontLeftDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_FrontLeft],   rev::ControlType::kVelocity);
    m_frontRightDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_FrontRight], rev::ControlType::kVelocity);
    m_rearLeftDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_RearLeft],     rev::ControlType::kVelocity);
    m_rearRightDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_RearRight],   rev::ControlType::kVelocity);
    }
  else
    {
    m_frontLeftDriveMotor.Set(0);
    m_frontRightDriveMotor.Set(0);
    m_rearLeftDriveMotor.Set(0);
    m_rearRightDriveMotor.Set(0);
    }

  m_frontLeftSteerMotor.Set(V_SD_WheelAngleCmnd[E_FrontLeft]);
  m_frontRightSteerMotor.Set(V_SD_WheelAngleCmnd[E_FrontRight]);
  m_rearLeftSteerMotor.Set(V_SD_WheelAngleCmnd[E_RearLeft]);
  m_rearRightSteerMotor.Set(V_SD_WheelAngleCmnd[E_RearRight]);

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

  if (V_LiftInitialized == false)
    {
    m_liftMotorYD.Set(V_LiftYD_TestPowerCmnd);
    m_liftMotorXD.Set(V_LiftXD_TestPowerCmnd);
    }
  else
    {
    m_liftpidYD.SetReference(V_lift_command_YD, rev::ControlType::kPosition); // positive is up
    m_liftpidXD.SetReference(V_lift_command_XD, rev::ControlType::kPosition); // This is temporary.  We actually want to use position, but need to force this off temporarily
    }
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

  m_liftMotorYD.Set(V_LiftYD_TestPowerCmnd);
  m_liftMotorXD.Set(V_LiftXD_TestPowerCmnd);

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
