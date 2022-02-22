/*
 * Team 5561 2022 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive (beta 02/10/2020)
 * - Shooting balls (beta 02/15/2020)
 *
 * */

// #define PHOTON
#define SPIKE

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>

#include "Encoders.hpp"
#include "BallHandler.hpp"
#include "LightControl.hpp"
#include "Enums.hpp"
#include "control_pid.hpp"
#include "Gyro.hpp"
#include "IO_Sensors.hpp"
#include "Lookup.hpp"
#include "vision.hpp"
#include "DriveControl.hpp"
#include "AutoTarget.hpp"
#include "Lift.hpp"
#include "Driver_inputs.hpp"
#include "Odometry.hpp"
#include "Auton.hpp"

nt::NetworkTableInstance inst;
nt::NetworkTableEntry driverMode0;
nt::NetworkTableEntry pipeline0;
nt::NetworkTableEntry targetYaw0;
nt::NetworkTableEntry targetPitch0;
nt::NetworkTableEntry targetPose0;
nt::NetworkTableEntry latency0;
nt::NetworkTableEntry driverMode1;
nt::NetworkTableEntry targetYaw1;
nt::NetworkTableEntry targetPitch1;
nt::NetworkTableEntry targetPose1;
nt::NetworkTableEntry latency1;
nt::NetworkTableEntry lidarDistance;
nt::NetworkTableEntry ledControl;

T_RobotState V_RobotState;

double       distanceTarget;
double       distanceBall;
double       distanceFromTargetCenter;
double       distanceFromBallCenter;
double       desiredVisionAngle0;
double       desiredVisionDistance0;
double       originalPosition;
bool         activeVisionAngle0;
bool         activeVisionDistance0;
bool         visionRequest;
bool         visionStart1;
bool         visionStart2;

int pipelineCounter;
bool V_pipelinecounterLatch;

bool V_autonTargetCmd = false;
bool V_autonTargetFin = false;


double V_M_RobotDisplacementX = 0;
double V_M_RobotDisplacementY = 0;

bool   LightOff; //the polarities are funny, true = off



/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit()
  {
  V_RobotState = E_Init;

  BallHandlerMotorConfigsInit(m_rightShooterpid,
                              m_leftShooterpid);

  m_liftMotorYD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_liftMotorXD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  LiftMotorConfigsInit(m_liftpidYD,
                       m_liftpidXD);

  VisionDashboard();

  V_M_RobotDisplacementY = 0;
  V_M_RobotDisplacementX = 0;

  GyroInit();
  
  inst = nt::NetworkTableInstance::Create();
  inst.StartClient("10.55.61.24");
  inst.StartDSClient();
  
// m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
// m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
// frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
// frc::SmartDashboard::PutNumber("cooler int", 1);

  #ifdef SPIKE
  LightOff = true; // light should be off on robot init
  #endif
  }


/******************************************************************************
 * Function:     RobotPeriodic
 *
 * Description:  Function called periodically (not defined well as to what
 *               "periodically" means).
 ******************************************************************************/
void Robot::RobotPeriodic()
  {
  BallHandlerMotorConfigsCal(m_rightShooterpid,
                             m_leftShooterpid);

  LiftMotorConfigsCal(m_liftpidYD,
                      m_liftpidXD);

    #ifdef PID_DEBUG
      // UpperShooterPIDConfig.Debug("Upper Shooter PID Control");
    #endif

  VisionRun();
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
    V_autonTargetCmd = false;
    V_autonTargetFin = false;
    
    EncodersInit(m_encoderFrontRightSteer,
                 m_encoderFrontLeftSteer,
                 m_encoderRearRightSteer,
                 m_encoderRearLeftSteer,
                 m_encoderLiftYD,
                 m_encoderLiftXD);

    DriveControlInit();
    BallHandlerInit();
    AutonDriveReset();
    V_M_RobotDisplacementX = 0;
    V_M_RobotDisplacementY = 0;

      // AutonDriveReset();  
  }


/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Function called periodically in autonomous.  This is where we
 *               should place our primary autonomous control code.
 ******************************************************************************/
void Robot::AutonomousPeriodic()
  {
    double L_timeleft = frc::Timer::GetMatchTime().value();
    double driveforward = 0;
    double strafe = 0;
    double speen = 0;

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
    
    ReadGyro(false);

    Read_IO_Sensors(di_IR_Sensor.Get(),
                    di_XD_LimitSwitch.Get(),
                    di_XY_LimitSwitch.Get());
 
    DtrmnSwerveBotLocation(V_GyroYawAngleRad,
                           &V_Rad_WheelAngleFwd[0],
                           &V_M_WheelDeltaDistance[0],
                           &V_M_RobotDisplacementX,
                           &V_M_RobotDisplacementY);
  
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
                      TopTargetAquired,
                      TopYaw,
                     &V_WheelAngleFwd[0],
                     &V_WheelAngleRev[0],
                     &V_WheelSpeedCmnd[0],
                     &V_WheelAngleCmnd[0],
                     &V_autonTargetFin,
                      V_RobotState);


    // m_rightShooterpid.SetReference(V_ShooterSpeedDesired[E_rightShooter], rev::ControlType::kVelocity);
    // m_leftShooterpid.SetReference(V_ShooterSpeedDesired[E_leftShooter], rev::ControlType::kVelocity);

    // m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    // m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    // m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    // m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);

    // m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft] * (-1));
    // m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight] * (-1));
    // m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft] * (-1));
    // m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight] * (-1));

    // m_elevator.Set(ControlMode::PercentOutput, 0);
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

  m_frontLeftSteerMotor.SetSmartCurrentLimit(K_SteerMotorCurrentLimit);
  m_frontRightSteerMotor.SetSmartCurrentLimit(K_SteerMotorCurrentLimit);
  m_rearLeftSteerMotor.SetSmartCurrentLimit(K_SteerMotorCurrentLimit);
  m_frontLeftSteerMotor.SetSmartCurrentLimit(K_SteerMotorCurrentLimit);

  EncodersInit(m_encoderFrontRightSteer,
               m_encoderFrontLeftSteer,
               m_encoderRearRightSteer,
               m_encoderRearLeftSteer,
               m_encoderLiftYD,
               m_encoderLiftXD);

  DriveControlInit();

  BallHandlerInit();

  LiftControlInit();

  V_M_RobotDisplacementX = 0;
  V_M_RobotDisplacementY = 0;
}


/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  Primary function called when in teleop mode.
 ******************************************************************************/
void Robot::TeleopPeriodic()
  {
  double L_timeleft = frc::DriverStation::GetInstance().GetMatchTime();

  frc::DriverStation::Alliance L_AllianceColor = frc::DriverStation::GetInstance().GetAlliance();

  Joystick_robot_mapping( c_joyStick2.GetRawButton(1),
                          c_joyStick2.GetRawButton(2),
                          c_joyStick2.GetRawButton(6), //change later
                          c_joyStick2.GetRawButton(7),
                          c_joyStick2.GetRawButton(8),
                          c_joyStick.GetRawButton(7),
                          c_joyStick2.GetRawButton(3),
                          c_joyStick2.GetRawAxis(1),
                          c_joyStick2.GetRawAxis(5),
                          c_joyStick.GetRawAxis(1),
                          c_joyStick.GetRawAxis(0),
                          c_joyStick.GetRawAxis(4),
                          c_joyStick.GetRawAxis(3),
                          c_joyStick.GetRawButton(1),
                          c_joyStick.GetRawButton(3),
                          c_joyStick.GetRawButton(4),
                          c_joyStick2.GetPOV());

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
                  di_XD_LimitSwitch.Get(),
                  di_XY_LimitSwitch.Get());

  LightControlMain( V_Driver_SwerveGoalAutoCenter,
                    V_Driver_auto_setspeed_shooter,
                    L_timeleft,
                    L_AllianceColor,
                    V_LauncherState,
                    V_SwerveTargetLocking,
                   &V_CameraLightCmndOn,
                   &V_VanityLightCmnd);

  DtrmnSwerveBotLocation(V_GyroYawAngleRad,
                         &V_Rad_WheelAngleFwd[0],
                         &V_M_WheelDeltaDistance[0],
                         &V_M_RobotDisplacementX,
                         &V_M_RobotDisplacementY);

  DriveControlMain( V_Driver_SwerveForwardBack,
                    V_Driver_SwerveStrafe,
                    V_Driver_SwerveRotate,
                    V_Driver_SwerveSpeed,
                    V_Driver_SwerveGoalAutoCenter,
                    V_Driver_SwerveRotateTo0,
                    V_Driver_SwerveRotateTo90,
                    V_GyroYawAngleDegrees,
                    V_GyroYawAngleRad,
                    TopTargetAquired,
                    TopYaw,
                   &V_WheelAngleFwd[0],
                   &V_WheelAngleRev[0],
                   &V_WheelSpeedCmnd[0],
                   &V_WheelAngleCmnd[0],
                   &V_autonTargetFin,
                    V_RobotState);

  V_Lift_state = Lift_Control_Dictator(V_Driver_lift_control,
                                       V_Driver_Lift_Cmnd_Direction,
                                       L_timeleft,
                                       V_Lift_state,
                                       V_LiftPostitionYD,
                                       V_LiftPostitionXD,
                                       &V_lift_command_YD,
                                       &V_lift_command_XD,
                                       V_GyroYawAngleDegrees);

  BallHandlerControlMain( V_Driver_intake_in,
                          V_BallDetectedRaw,
                          V_Driver_elevator_up,
                          V_Driver_elevator_down,
                          V_Driver_stops_shooter,
                          V_Driver_auto_setspeed_shooter,
                          V_autonTargetFin,
                          TopTargetAquired,
                          V_TopTargetDistanceMeters,
                          V_ShooterSpeedCurr,
                          V_Driver_manual_shooter_desired_speed,
                          V_CameraLightStatus,
                         &V_IntakePowerCmnd,
                         &V_ElevatorPowerCmnd,
                         &V_ShooterRPM_Cmnd);

  // Motor output commands:
    m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);

    // m_frontLeftDriveMotor.Set(0);
    // m_frontRightDriveMotor.Set(0);
    // m_rearLeftDriveMotor.Set(0);
    // m_rearRightDriveMotor.Set(0);

    m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft]);
    m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight]);
    m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft]);
    m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight]);

    // m_frontLeftSteerMotor.Set(0);
    // m_frontRightSteerMotor.Set(0);
    // m_rearLeftSteerMotor.Set(0);
    // m_rearRightSteerMotor.Set(0);

    m_rightShooterpid.SetReference(-V_ShooterRPM_Cmnd, rev::ControlType::kSmartVelocity);
    m_leftShooterpid.SetReference(V_ShooterRPM_Cmnd, rev::ControlType::kSmartVelocity);

    m_intake.Set(ControlMode::PercentOutput, V_IntakePowerCmnd); //must be positive (don't be a fool)
    m_elevator.Set(ControlMode::PercentOutput, V_ElevatorPowerCmnd);

    m_liftpidYD.SetReference(V_lift_command_YD, rev::ControlType::kSmartMotion); // positive is up
    m_liftpidXD.SetReference(V_lift_command_XD, rev::ControlType::kSmartMotion); // This is temporary.  We actually want to use position, but need to force this off temporarily

    do_CameraLightControl.Set(V_CameraLightCmndOn);
    m_vanityLightControler.Set(V_VanityLightCmnd);
}


/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  Called during the test phase initiated on the driver station.
 ******************************************************************************/
void Robot::TestPeriodic()
  {
  double L_LiftYD_Power = 0;
  double L_LiftXD_Power = 0;

  Read_IO_Sensors(di_IR_Sensor.Get(),
                  di_XD_LimitSwitch.Get(),
                  di_XY_LimitSwitch.Get());

    if (c_joyStick2.GetPOV() == 0)
      {
      L_LiftYD_Power = 1.0;
      }
    else if (c_joyStick2.GetPOV() == 180)
      {
        if (V_YD_LimitDetected == false)
          {
          L_LiftYD_Power = -0.25;
          }
      }
    else if (c_joyStick2.GetPOV() == 270)
      {
        if (V_XD_LimitDetected == false)
          {
          L_LiftXD_Power = -0.15;
          }
      }
    else if (c_joyStick2.GetPOV() == 90)
      {
      L_LiftXD_Power = 0.15;
      }

    m_liftMotorXD.Set(L_LiftXD_Power);
    m_liftMotorYD.Set(L_LiftYD_Power);
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
