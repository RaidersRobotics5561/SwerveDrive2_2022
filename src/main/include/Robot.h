#ifndef ROBOT
#define ROBOT

#pragma once

#include <string>

#include <frc/AnalogInput.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>


#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/motorcontrol/Spark.h>
#include "Const.hpp"

#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include <networktables/NetworkTable.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  frc::AnalogInput a_encoderFrontLeftSteer{2};
  frc::AnalogInput a_encoderFrontRightSteer{1};
  frc::AnalogInput a_encoderRearLeftSteer{3};
  frc::AnalogInput a_encoderRearRightSteer{0};

  rev::CANSparkMax m_frontLeftSteerMotor {frontLeftSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontLeftDriveMotor {frontLeftDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRightSteerMotor{frontRightSteerDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_frontRightDriveMotor{frontRightDriveDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeftSteerMotor  {rearLeftSteerDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearLeftDriveMotor  {rearLeftDriveDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRightSteerMotor {rearRightSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rearRightDriveMotor {rearRightDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightShooterMotor     {rightShooterID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftShooterMotor  {leftShooterID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_liftMotor           {liftID,                  rev::CANSparkMax::MotorType::kBrushless};

  VictorSPX m_intake {14};
  VictorSPX m_elevateDaBalls {15};

  ctre::phoenix::motorcontrol::can::TalonSRX m_intake2 {1};
  ctre::phoenix::motorcontrol::can::TalonSRX m_elevateDaBalls2 {6};

  
   rev::SparkMaxPIDController m_rightShooterpid = m_rightShooterMotor.GetPIDController();
   rev::SparkMaxPIDController m_leftShooterpid = m_leftShooterMotor.GetPIDController();
  // rev::SparkMaxPIDController m_liftpid          = m_liftMotor.GetPIDController();

  rev::SparkMaxRelativeEncoder m_encoderFrontLeftSteer  = m_frontLeftSteerMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive  = m_frontLeftDriveMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderFrontRightSteer = m_frontRightSteerMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive = m_frontRightDriveMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderRearLeftSteer   = m_rearLeftSteerMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive   = m_rearLeftDriveMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderRearRightSteer  = m_rearRightSteerMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderRearRightDrive  = m_rearRightDriveMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderrightShooter      = m_rightShooterMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderleftShooter   = m_leftShooterMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_encoderLift            = m_liftMotor.GetEncoder();

  frc::Joystick c_joyStick{0};
  frc::Joystick c_joyStick2{1};

  frc::PowerDistribution PDP {0, frc::PowerDistribution::ModuleType::kCTRE};

  double Upper_P_Gx = 0, Upper_I_Gx = 0, Upper_D_Gx = 0, Upper_I_Zone = 0, Upper_FF = 0, Upper_Max = 1, Upper_Min = -1;
  double Lower_P_Gx = 0, Lower_I_Gx = 0, Lower_D_Gx = 0, Lower_I_Zone = 0, Lower_FF = 0, Lower_Max = 1, Lower_Min = -1;
  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  
};

#endif