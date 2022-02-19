/*
 * Team 5561 2022 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive (beta 02/10/2020)
 * - Shooting balls (beta 02/15/2020)
 *
 * */

//NOTE: Set this to TEST for testing of speeds and PID gains.  Set to COMP for competion
#define TEST
// #define PHOTON
//NOTE: Set this to allow Shuffleboard configuration of PIDConfig objects (Will override defaults)
#define PID_DEBUG
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

#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include "Lift.hpp"
#include "Driver_inputs.hpp"
#include "Lift.hpp"
#include "Driver_inputs.hpp"

#include <units/length.h>
#include "Utils/PIDConfig.hpp"
#include "Odometry.hpp"
#include "Auton.hpp"
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>



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

double V_XD_Test = 0;
double V_YD_Test = 0;
double V_Intake_Test = 0;
double V_Elevator_Test = 0;

double V_P_Gx = 0;
double V_I_Gx = 0;
double V_D_Gx = 0;
double V_I_Zone = 0;
double V_FF = 0;
double V_Max = 0;
double V_Min = 0;

double K_MaxVel = 0;
double K_MinVel = 0;
double K_MaxAcc = 0;
double K_AllErr = 0;

bool   LightOff; //the polarities are funny, true = off

T_RobotState V_RobotState;



// PIDConfig UpperShooterPIDConfig {0.0008, 0.000001, 0.0006};

/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit() {

    VisionDashboard();

    V_RobotState = E_Init;

//  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
//  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
//  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    m_liftMotorYD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_liftMotorXD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    V_M_RobotDisplacementY = 0;
    V_M_RobotDisplacementX = 0;

    GyroInit();

    inst = nt::NetworkTableInstance::Create();
    inst.StartClient("10.55.61.24");
    inst.StartDSClient();

    frc::SmartDashboard::PutNumber("P_Gx", V_P_Gx);
    frc::SmartDashboard::PutNumber("I_Gx", V_I_Gx);
    frc::SmartDashboard::PutNumber("D_Gx", V_D_Gx);
    frc::SmartDashboard::PutNumber("I_Zone", V_I_Zone);
    frc::SmartDashboard::PutNumber("FF", V_FF);
    frc::SmartDashboard::PutNumber("Max_Limit", V_Max);
    frc::SmartDashboard::PutNumber("Min_Limit", V_Min);
    frc::SmartDashboard::PutNumber("XD Lift", V_XD_Test);
  frc::SmartDashboard::PutNumber("YD Lift", V_YD_Test);
  frc::SmartDashboard::PutNumber("Intake", V_Intake_Test);
  frc::SmartDashboard::PutNumber("Elevator", V_Elevator_Test);

  frc::SmartDashboard::PutNumber("max velocity", K_MaxVel);
  frc::SmartDashboard::PutNumber("min velocity", K_MinVel);
  frc::SmartDashboard::PutNumber("max acceleration", K_MaxAcc);
  frc::SmartDashboard::PutNumber("kAllErr", K_AllErr);
  
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

    #ifdef PID_DEBUG
      // UpperShooterPIDConfig.Debug("Upper Shooter PID Control");
    #endif

    //Run Gyro readings when the robot starts


  void VisionRun();
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
    double timeleft = frc::Timer::GetMatchTime().value();
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
                      c_joyStick.GetRawAxis(3),
                      V_autonTargetCmd,
                      c_joyStick.GetRawButton(3),
                      c_joyStick.GetRawButton(4),
                      V_GyroYawAngleDegrees,
                      V_GyroYawAngleRad,
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

  EncodersInit(m_encoderFrontRightSteer,
               m_encoderFrontLeftSteer,
               m_encoderRearRightSteer,
               m_encoderRearLeftSteer,
               m_encoderLiftYD,
               m_encoderLiftXD);

  DriveControlInit();

  BallHandlerInit();

  V_AutoShootEnable = false;
  V_M_RobotDisplacementX = 0;
  V_M_RobotDisplacementY = 0;

  // V_P_Gx = frc::SmartDashboard::GetNumber("P_Gx", V_P_Gx);
  // V_I_Gx = frc::SmartDashboard::GetNumber("I_Gx", V_I_Gx);
  // V_D_Gx = frc::SmartDashboard::GetNumber("D_Gx", V_D_Gx);
  // V_I_Zone = frc::SmartDashboard::GetNumber("I_Zone", V_I_Zone);
  // V_FF = frc::SmartDashboard::GetNumber("FF", V_FF);
  // V_Max = frc::SmartDashboard::GetNumber("Max_Limit", V_Max);
  // V_Min = frc::SmartDashboard::GetNumber("Min_Limit", V_Min);

  // K_WheelSpeedPID_Gx[E_P_Gx] = V_P_Gx;
  // K_WheelSpeedPID_Gx[E_I_Gx] = V_I_Gx;
  // K_WheelSpeedPID_Gx[E_D_Gx] = V_D_Gx;
  // K_WheelSpeedPID_Gx[E_I_Ul] = V_I_Zone;
  // K_WheelSpeedPID_Gx[E_I_Ll] = -V_I_Zone;
  // K_WheelSpeedPID_Gx[E_Max_Ul] = V_Max;
  // K_WheelSpeedPID_Gx[E_Max_Ll] = V_Min;


  K_MaxVel = frc::SmartDashboard::GetNumber("max velocity", K_MaxVel);
  K_MinVel = frc::SmartDashboard::GetNumber("min velocity", K_MinVel);
  K_MaxAcc = frc::SmartDashboard::GetNumber("max acceleration", K_MaxAcc);
  K_AllErr = frc::SmartDashboard::GetNumber("kAllErr", K_AllErr);

  m_liftpidYD.SetP(V_P_Gx);
  m_liftpidYD.SetI(V_I_Gx);
  m_liftpidYD.SetD(V_D_Gx);
  m_liftpidYD.SetIZone(V_I_Zone);
  m_liftpidYD.SetFF(V_FF);
  m_liftpidYD.SetOutputRange(V_Min, V_Max);

  m_liftpidXD.SetP(V_P_Gx);
  m_liftpidXD.SetI(V_I_Gx);
  m_liftpidXD.SetD(V_D_Gx);
  m_liftpidXD.SetIZone(V_I_Zone);
  m_liftpidXD.SetFF(V_FF);
  m_liftpidXD.SetOutputRange(V_Min, V_Max);



  m_liftpidYD.SetSmartMotionMaxVelocity(K_MaxVel);
  m_liftpidYD.SetSmartMotionMinOutputVelocity(K_MinVel);
  m_liftpidYD.SetSmartMotionMaxAccel(K_MaxAcc);
  m_liftpidYD.SetSmartMotionAllowedClosedLoopError(K_AllErr);
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
  double L_Driver_right_shooter_desired_speed = 0;
  double L_Driver_left_shooter_desired_speed = 0;
  bool L_Driver_intake_in = false;
  //bool L_driver_intake_out = false;

  double timeleft = frc::DriverStation::GetInstance().GetMatchTime();

  Joystick_robot_mapping(    c_joyStick2.GetRawButton(1),
                            &L_Driver_elevator_up,
                             c_joyStick2.GetRawButton(2),
                            &L_Driver_elevator_down,
                             c_joyStick2.GetRawButton(6), //change later
                            &L_Driver_lift_control,
                             c_joyStick2.GetRawButton(7),
                            &L_Driver_stops_shooter,
                             c_joyStick2.GetRawButton(8),
                            &L_Driver_auto_setspeed_shooter,
                             c_joyStick.GetRawButton(7),
                            &L_Driver_zero_gyro,
                            c_joyStick2.GetRawButton(3),
                            &L_Driver_intake_in,
                             c_joyStick2.GetRawAxis(1),
                            &L_Driver_right_shooter_desired_speed,
                             c_joyStick2.GetRawAxis(5),
                            &L_Driver_left_shooter_desired_speed);


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

  ReadGyro(L_Driver_zero_gyro);

  Read_IO_Sensors(di_IR_Sensor.Get(),
                  di_XD_LimitSwitch.Get(),
                  di_XY_LimitSwitch.Get());

  DtrmnSwerveBotLocation(V_GyroYawAngleRad,
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
                    V_GyroYawAngleDegrees,
                    V_GyroYawAngleRad,
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
                                       V_GyroYawAngleDegrees);

  // BallHandlerControlMain();

  double L_Intake = 0;
  double L_Elevator = 0;
  double L_Shooter = 0;
  BallHandlerControlMain(   L_Driver_intake_in,
                            false,
                            L_Driver_elevator_up,
                            L_Driver_elevator_down,
                            L_Driver_right_shooter_desired_speed,
                            &L_Intake,
                            &L_Elevator,
                            &L_Shooter);

  LED_VanityLights();
  
  
  V_XD_Test = frc::SmartDashboard::GetNumber("XD Lift", V_XD_Test);
  V_YD_Test = frc::SmartDashboard::GetNumber("YD Lift", V_YD_Test);
  V_Intake_Test = frc::SmartDashboard::GetNumber("Intake", V_Intake_Test);
  V_Elevator_Test = frc::SmartDashboard::GetNumber("Elevator", V_Elevator_Test);

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


    // m_rightShooterpid.SetReference(L_Shooter, rev::ControlType::kVelocity);
    // m_leftShooterpid.SetReference(-L_Shooter, rev::ControlType::kVelocity);

double L_ELE = 0;
if (c_joyStick2.GetRawButton(1) == true)
{
  L_ELE = 0.9;
}
else if (c_joyStick2.GetRawButton(2) == true)
{
  L_ELE = -0.9;
}

m_rightShooterMotor.Set(c_joyStick2.GetRawAxis(1));
    m_leftShooterMotor.Set(-c_joyStick2.GetRawAxis(1));

    m_intake.Set(ControlMode::PercentOutput, L_Intake); //must be positive (don't be a fool)
    m_elevator.Set(ControlMode::PercentOutput, L_ELE);

    // m_liftpidYD.SetReference(V_lift_command_YD, rev::ControlType::kPosition);
    // m_liftpidXD.SetReference(V_lift_command_XD, rev::ControlType::kPosition);

    m_liftpidYD.SetReference(V_YD_Test, rev::ControlType::kPosition); // positive is up
    m_liftpidXD.SetReference(V_XD_Test, rev::ControlType::kPosition); // This is temporary.  We actually want to use position, but need to force this off temporarily
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

    if (c_joyStick2.GetPOV() == 0)
      {
      L_LiftYD_Power = 1.0;
      }
    else if (c_joyStick2.GetPOV() == 180)
      {
      L_LiftYD_Power = -0.25;
      }
    else if (c_joyStick2.GetPOV() == 270)
      {
      L_LiftXD_Power = -0.15;
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
