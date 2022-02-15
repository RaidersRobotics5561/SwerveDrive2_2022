/*
 * Team 5561 2022 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive (beta 02/10/2020)
 * - Shooting balls (beta 02/15/2020)
 *
 * */

//NOTE: Set this to TEST for testing of speeds and PID gains.  Set to COMP for competion
#define COMP
//NOTE: Set this to allow Shuffleboard configuration of PIDConfig objects (Will override defaults)
#define PID_DEBUG

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
#include "Lookup.hpp"
#include "vision.hpp"
#include "DriveControl.hpp"
#include "AutoTarget.hpp"
#include "Lift.hpp"
#include "Driver_inputs.hpp"

#include "Utils/PIDConfig.hpp"
#include "Odometry.hpp"
#include "Auton.hpp"
#include <units/length.h>

std::shared_ptr<nt::NetworkTable> vision0;
std::shared_ptr<nt::NetworkTable> vision1;
std::shared_ptr<nt::NetworkTable> lidar;
std::shared_ptr<nt::NetworkTable> ledLight;

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

frc::DriverStation::Alliance V_AllianceColor;

double V_ShooterSpeedDesired[E_RoboShooter];

double V_AutoTargetAngle;
double V_AutoTargetUpperRollerSpd;
double V_AutoTargetLowerRollerSpd;
double V_AutoTargetBeltPower;

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

bool         V_AutoShootEnable;




bool V_autonTargetCmd = false;
bool V_autonTargetFin = false;


double V_M_RobotDisplacementX = 0;
double V_M_RobotDisplacementY = 0;

T_RobotState V_RobotState;



// PIDConfig UpperShooterPIDConfig {0.0008, 0.000001, 0.0006};

/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit() {
    V_RobotState = E_Init;
//  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
//  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
//  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    m_liftMotorYD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_liftMotorXD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    V_M_RobotDisplacementY = 0;
    V_M_RobotDisplacementX = 0;

    GyroRobotInit();

    inst = nt::NetworkTableInstance::Create();
    inst.StartClient("10.55.61.24");
    inst.StartDSClient();

    vision0  = inst.GetTable("chameleon-vision/goal");
    vision1  = inst.GetTable("chameleon-vision/ColorWheel");
    lidar    = inst.GetTable("lidar");
    ledLight = inst.GetTable("ledLight");


    driverMode0           = vision0->GetEntry("driverMode");
    pipeline0             = vision0->GetEntry("pipeline");
    targetPitch0          = vision0->GetEntry("targetPitch");
    targetYaw0            = vision0->GetEntry("targetYaw");
    targetPose0           = vision0->GetEntry("targetpose");
    latency0              = vision0->GetEntry("latency");

    driverMode1           = vision1->GetEntry("driverMode");
    targetPitch1          = vision1->GetEntry("targetPitch");
    targetYaw1            = vision1->GetEntry("targetYaw");
    targetPose1           = vision1->GetEntry("targetpose");
    latency1              = vision1->GetEntry("latency");

    ledControl            = ledLight->GetEntry("ledControl");
    lidarDistance         = lidar->GetEntry("lidarDistance");
  
 #ifdef COMP
    // V_testIntake = 0;
    // V_testElevator = 0;
    // V_testspeed = 0;
    // frc::SmartDashboard::PutNumber("Intake Power",V_testIntake);
    // frc::SmartDashboard::PutNumber("Elevator Power",V_testElevator);
    // frc::SmartDashboard::PutNumber("Speed Desired", V_testspeed);

    // frc::SmartDashboard::PutNumber("P_Gx", V_P_Gx);
    // frc::SmartDashboard::PutNumber("I_Gx", V_I_Gx);
    // frc::SmartDashboard::PutNumber("D_Gx", V_D_Gx);
    // frc::SmartDashboard::PutNumber("I_Zone", V_I_Zone);
    // frc::SmartDashboard::PutNumber("FF", V_FF);
    // frc::SmartDashboard::PutNumber("Max_Limit", V_Max);
    // frc::SmartDashboard::PutNumber("Min_Limit", V_Min);

    // frc::SmartDashboard::PutNumber("P_Steer_Gx", V_Steer_P_Gx);
    // frc::SmartDashboard::PutNumber("I_Steer_Gx", V_Steer_I_Gx);
    // frc::SmartDashboard::PutNumber("D_Steer_Gx", V_Steer_D_Gx);
    // frc::SmartDashboard::PutNumber("I_Steer_Zone", V_Steer_I_Zone);
    // frc::SmartDashboard::PutNumber("FF_Steer", V_Steer_FF);
    // frc::SmartDashboard::PutNumber("Max_Limit_Steer", V_Steer_Max);
    // frc::SmartDashboard::PutNumber("Min_Limit_Steer", V_Steer_Min);

    // frc::SmartDashboard::PutNumber("P_Gx_Drive", V_Drive_P_Gx);
    // frc::SmartDashboard::PutNumber("I_Gx_Drive", V_Drive_I_Gx);
    // frc::SmartDashboard::PutNumber("D_Gx_Drive", V_Drive_D_Gx);
    // frc::SmartDashboard::PutNumber("I_Zone_Drive", V_Drive_I_Zone);
    // frc::SmartDashboard::PutNumber("FF_Drive", V_Drive_FF);
    // frc::SmartDashboard::PutNumber("Max_Limit_Drive", V_Drive_Max);
    // frc::SmartDashboard::PutNumber("Min_Limit_Drive", V_Drive_Min);
#endif

    // frc::SmartDashboard::PutNumber("Speed Desired Right", 0);
    // frc::SmartDashboard::PutNumber("Speed Desired Left", 0);

    frc::SmartDashboard::PutNumber("cooler int", 1);

    V_ShooterSpeedDesired[E_rightShooter] = 0;
    V_ShooterSpeedDesired[E_leftShooter] = 0;

    // double lower_P_Gx = .0008;
    // double lower_I_Gx = .000001;
    // double lower_D_Gx = .0006;
    // double lower_I_Zone = 0;
    // double lower_FF = 0;
    // double lower_Max = 1;
    // double lower_Min = -1;

    // m_rightShooterpid.SetP(V_P_Gx);
    // m_rightShooterpid.SetI(V_I_Gx);
    // m_rightShooterpid.SetD(V_D_Gx);
    // m_rightShooterpid.SetIZone(V_I_Zone);
    // m_rightShooterpid.SetFF(V_FF);
    // m_rightShooterpid.SetOutputRange(V_Min, V_Max);

    // m_leftShooterpid.SetP(V_P_Gx);
    // m_leftShooterpid.SetI(V_I_Gx);
    // m_leftShooterpid.SetD(V_D_Gx);
    // m_leftShooterpid.SetIZone(V_I_Zone);
    // m_leftShooterpid.SetFF(V_FF);
    // m_leftShooterpid.SetOutputRange(V_Min, V_Max);

    // m_liftpidYD.SetP(kP);
    // m_liftpidYD.SetI(kI);
    // m_liftpidYD.SetD(kD);
    // m_liftpidYD.SetIZone(kIz);
    // m_liftpidYD.SetFF(kFF);
    // m_liftpidYD.SetOutputRange(kMinOutput, kMaxOutput);

    // m_liftpidXD.SetP(kP);
    // m_liftpidXD.SetI(kI);
    // m_liftpidXD.SetD(kD);
    // m_liftpidXD.SetIZone(kIz);
    // m_liftpidXD.SetFF(kFF);
    // m_liftpidXD.SetOutputRange(kMinOutput, kMaxOutput);

    // frc::SmartDashboard::PutNumber("P Gain", kP);
    // frc::SmartDashboard::PutNumber("I Gain", kI);
    // frc::SmartDashboard::PutNumber("D Gain", kD);
    // frc::SmartDashboard::PutNumber("I Zone", kIz);
    // frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    // frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    // frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    // frc::SmartDashboard::PutNumber("Desired Level", 0);


    // frc::SmartDashboard::PutNumber("Blinkin code", 0);
}


/******************************************************************************
 * Function:     RobotPeriodic
 *
 * Description:  Function called periodically (not defined well as to what
 *               "periodically" means).
 ******************************************************************************/
void Robot::RobotPeriodic()
{
  // frc::SmartDashboard::PutNumber("Postion_YD", m_encoderLiftYD.GetPosition());
  // frc::SmartDashboard::PutNumber("Postion_XD", m_encoderLiftXD.GetPosition());

    #ifdef PID_DEBUG
      // UpperShooterPIDConfig.Debug("Upper Shooter PID Control");
    #endif

    //Run Gyro readings when the robot starts
    // Gyro();
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
    
    V_ShooterSpeedDesired[E_rightShooter] = 0;
    V_ShooterSpeedDesired[E_leftShooter] = 0;
      
      
      GyroZero();
      EncodersInit(a_encoderWheelAngleFrontLeft.Get().value(),
                   a_encoderWheelAngleFrontRight.Get().value(),
                   a_encoderWheelAngleRearLeft.Get().value(),
                   a_encoderWheelAngleRearRight.Get().value());
      DriveControlInit();
      BallHandlerInit();
      AutonDriveReset();
      gyro_yawangledegrees = 0;
      gyro_yawanglerad = 0;
      V_M_RobotDisplacementX = 0;
      V_M_RobotDisplacementY = 0;

      
      // visionInit(vision0, ledLight, inst);
      originalPosition = targetYaw0.GetDouble(0);
      vision0->PutNumber("pipeline", 0);
      vision1->PutBoolean("driverMode", true);
      inst.Flush();    
  }


/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Function called periodically in autonomous.  This is where we
 *               should place our primary autonomous control code.
 ******************************************************************************/
void Robot::AutonomousPeriodic()
  {
    double timeleft = frc::Timer::GetMatchTime().value();
    double driveforward = 0;
    double strafe = 0;
    double speen = 0;

    Read_Encoders(m_encoderFrontLeftSteer,
                  m_encoderFrontRightSteer,
                  m_encoderRearLeftSteer,
                  m_encoderRearRightSteer,
                  m_encoderFrontLeftDrive,
                  m_encoderFrontRightDrive,
                  m_encoderRearLeftDrive,
                  m_encoderRearRightDrive,
                  m_encoderrightShooter,
                  m_encoderleftShooter);
 
    DtrmnSwerveBotLocation(gyro_yawanglerad,
                           &V_Rad_WheelAngleFwd[0],
                           &V_M_WheelDeltaDistance[0],
                           &V_M_RobotDisplacementX,
                           &V_M_RobotDisplacementY);

    AutonDriveMain();

    DriveControlMain( driveforward,
                      strafe,
                      speen,
                      c_joyStick.GetRawAxis(3),
                      V_autonTargetCmd,
                      c_joyStick.GetRawButton(3),
                      c_joyStick.GetRawButton(4),
                      c_joyStick.GetRawButton(5),
                      gyro_yawangledegrees,
                      gyro_yawanglerad,
                      targetYaw0.GetDouble(0),
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

  m_encoderFrontRightSteer.SetPosition(0);
  m_encoderFrontLeftSteer.SetPosition(0);
  m_encoderRearRightSteer.SetPosition(0);
  m_encoderRearLeftSteer.SetPosition(0);

  EncodersInit(a_encoderWheelAngleFrontLeft.Get().value(),
               a_encoderWheelAngleFrontRight.Get().value(),
               a_encoderWheelAngleRearLeft.Get().value(),
               a_encoderWheelAngleRearRight.Get().value());

  DriveControlInit();
  BallHandlerInit();
  gyro_yawangledegrees = 0;
  gyro_yawanglerad = 0;
  V_AutoShootEnable = false;
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
  bool L_Driver_lift_control = false;
  bool L_Driver_zero_gyro = false;
  bool L_Driver_stops_shooter = false;
  bool L_Driver_auto_setspeed_shooter = false;
  bool L_Driver_elevator_up = false;
  bool L_Driver_elevator_down = false;
  bool L_Driver_right_shooter_desired_speed = false;
  bool L_Driver_left_shooter_desired_speed = false;

  double timeleft = frc::DriverStation::GetInstance().GetMatchTime();

  Joystick_robot_mapping(    c_joyStick2.GetRawButton(1),
                            &L_Driver_elevator_up,
                             c_joyStick2.GetRawButton(2),
                            &L_Driver_elevator_down,
                             c_joyStick2.GetRawButton(3), //change later
                            &L_Driver_lift_control,
                             c_joyStick2.GetRawButton(7),
                            &L_Driver_stops_shooter,
                             c_joyStick2.GetRawButton(8),
                            &L_Driver_auto_setspeed_shooter,
                             c_joyStick.GetRawButton(7),
                            &L_Driver_zero_gyro,
                             c_joyStick2.GetRawAxis(1),
                            &L_Driver_right_shooter_desired_speed,
                             c_joyStick2.GetRawAxis(5),
                            &L_Driver_left_shooter_desired_speed);

  Read_Encoders(m_encoderFrontLeftSteer,
                m_encoderFrontRightSteer,
                m_encoderRearLeftSteer,
                m_encoderRearRightSteer,
                m_encoderFrontLeftDrive,
                m_encoderFrontRightDrive,
                m_encoderRearLeftDrive,
                m_encoderRearRightDrive,
                m_encoderrightShooter,
                m_encoderleftShooter);

  DtrmnSwerveBotLocation(gyro_yawanglerad,
                         &V_Rad_WheelAngleFwd[0],
                         &V_M_WheelDeltaDistance[0],
                         &V_M_RobotDisplacementX,
                         &V_M_RobotDisplacementY);

  DriveControlMain( c_joyStick.GetRawAxis(1),
                    c_joyStick.GetRawAxis(0),
                    c_joyStick.GetRawAxis(4),
                    c_joyStick.GetRawAxis(3),
                    c_joyStick.GetRawButton(1),
                    c_joyStick.GetRawButton(3),
                    c_joyStick.GetRawButton(4),
                    c_joyStick.GetRawButton(5),
                    gyro_yawangledegrees,
                    gyro_yawanglerad,
                    targetYaw0.GetDouble(0),
                   &V_WheelAngleFwd[0],
                   &V_WheelAngleRev[0],
                   &V_WheelSpeedCmnd[0],
                   &V_WheelAngleCmnd[0],
                   &V_autonTargetFin,
                    V_RobotState);

  V_Lift_state = Lift_Control_Dictator(L_Driver_lift_control,
                                       timeleft,
                                       V_Lift_state,
                                       V_lift_measured_position_YD,
                                       V_lift_measured_position_XD,
                                       &V_lift_command_YD,
                                       &V_lift_command_XD,
                                       V_gyro_yawangledegrees);

  BallHandlerControlMain();

  LED_VanityLights();
  
  
  frc::SmartDashboard::PutNumber("wheelAngleCmd Front Left", V_WheelAngleCmnd[E_FrontLeft]);

  // Motor output commands:
    m_frontLeftDriveMotor.Set(V_WheelSpeedCmnd[E_FrontLeft]);
    // m_frontRightDriveMotor.Set(V_WheelSpeedCmnd[E_FrontRight]);
    // m_rearLeftDriveMotor.Set(V_WheelSpeedCmnd[E_RearLeft]);
    // m_rearRightDriveMotor.Set(V_WheelSpeedCmnd[E_RearRight]);

    // m_frontLeftDriveMotor.Set(0);
    m_frontRightDriveMotor.Set(0);
    m_rearLeftDriveMotor.Set(0);
    m_rearRightDriveMotor.Set(0);

    m_frontLeftSteerMotor.Set(V_WheelAngleCmnd[E_FrontLeft]);
    // m_frontRightSteerMotor.Set(V_WheelAngleCmnd[E_FrontRight]);
    // m_rearLeftSteerMotor.Set(V_WheelAngleCmnd[E_RearLeft]);
    // m_rearRightSteerMotor.Set(V_WheelAngleCmnd[E_RearRight]);

    // m_frontLeftSteerMotor.Set(0);
    m_frontRightSteerMotor.Set(0);
    m_rearLeftSteerMotor.Set(0);
    m_rearRightSteerMotor.Set(0);

    m_rightShooterpid.SetReference(0, rev::ControlType::kVelocity);
    m_leftShooterpid.SetReference(-0, rev::ControlType::kVelocity);

    m_intake.Set(ControlMode::PercentOutput, 0);
    m_elevator.Set(ControlMode::PercentOutput, 0);

    // m_liftpidYD.SetReference(V_lift_command_YD, rev::ControlType::kPosition);
    // m_liftpidXD.SetReference(V_lift_command_XD, rev::ControlType::kPosition);
    m_liftpidYD.SetReference(0, rev::ControlType::kVelocity); // This is temporary.  We actually want to use position, but need to force this off temporarily
    m_liftpidXD.SetReference(0, rev::ControlType::kVelocity); // This is temporary.  We actually want to use position, but need to force this off temporarily
}


/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  Called during the test phase initiated on the driver station.
 ******************************************************************************/
void Robot::TestPeriodic()
  {

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
