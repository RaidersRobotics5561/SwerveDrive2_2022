#include "Enums.hpp"
#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>

// Define the desired test state here: COMP (no test), BallHandlerTest, LiftXY_Test, DriveMotorTest, WheelAngleTest, ADAS_UT_Test, ADAS_BT_Test
#define ADAS_UT_Test 
// Define the version of vision to use: VISION1 VISION2
#define VISION2

// RoboRio controller execution time
const double C_ExeTime = 0.02; // Set to match the the default controller loop time of 20 ms
const units::second_t C_ExeTime_t = 0.02_s; // Set to match the the default controller loop time of 20 ms

// Amount of time for end game
const double C_End_game_time = 30;

// Numerical constants
const double C_RadtoDeg = 57.2958;
const double C_Deg2Rad = 0.017453292519943295;
const double C_PI = 3.14159265358979;
const double C_Tau = 6.28318530717958647;

static const double C_EncoderToAngle = 360; // Raw output of PWM encoder to degrees

// CAN Device IDs:
static const int C_PDP_ID = 0;
static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
static const int rearLeftSteerDeviceID  = 5, rearLeftDriveDeviceID  = 6, rearRightSteerDeviceID  = 7, rearRightDriveDeviceID  = 8;
static const int rightShooterID = 10, leftShooterID = 9;
static const int C_liftYD_ID = 11;
static const int C_liftXD_ID = 12;
static const int C_elevatorID = 13;
static const int C_intakeID = 14;

// DIO IDs:
static const int C_MagEncoderFL_ID = 2, C_MagEncoderFR_ID = 1, C_MagEncoderRL_ID = 3, C_MagEncoderRR_ID = 0;
static const int C_XY_LimitSwitch_ID = 4, C_XD_LimitSwitch_ID = 6, C_IR_Sensor_ID = 9, C_CameraLightControl_ID = 7;
static const int C_LowerBallSensorID = 5;

// PWM IDs:
static const int C_VanityLight_ID = 0;


// Motor specific cals
const double K_SteerMotorCurrentLimit = 25;


// Vision Cals:
// cals for top target cam
const units::meter_t K_VisionHeight1 = 0.725_m; // 725 mm to camera lense
const units::meter_t K_VisionTargetHeight1 = 2.58_m; // bottom of tape to carpet 
const units::radian_t K_VisionCameraPitch1 = 45_deg; // camera on a 45 degree tilt

// cals for bottom ball cam
const units::meter_t K_VisionHeight2 = 0.367_m;
const units::meter_t K_VisionTargetHeight2 = .12_m; // radius of the ball in cm
const units::radian_t K_VisionCameraPitch2 = 50_deg;


// Cals / constants for Light Control
const double K_CameraLightDelay = 0.01; // Delay time between enabling the camera light and allowing the data feed to be used. [seconds]
const double K_CameraLightMaxOnTime = 10.0; // Max amount of time to have the camera light enabled. [seconds]
const double C_BlinkinLED_SolidWhite = 0.93;
const double C_BlinkinLED_BreathRed = -0.17;
const double C_BlinkinLED_BreathBlue = -0.15;
const double C_BlinkinLED_LightChaseGray = -0.27;
const double C_BlinkinLED_RainbowWithGlitter = -0.89;



// Encoder / speed calculation related cals
const double K_ReductionRatio = 8.31;
const double K_WheelCircufrence = 12.566; // Circumferance of wheel, in inches



/* Lift specific calibrations: */
const double K_lift_S2_YD = 8; //initial lift of the robot
const double K_lift_S3_YD = 8; //stays the same
const double K_lift_S4_YD = 26; //Move YD off of hooks
const double K_lift_S5_YD = 26; //stays the same
const double K_lift_S6_YD = 38; //lower YD below the rung
const double K_lift_S7_YD = 165; //
const double K_lift_S8_YD = 210; //
const double K_lift_S9_YD = 210; //stays the same
const double K_lift_S10_YD = 170; //
const double K_lift_S11_YD = 140; //

const double K_lift_max_YD = 210; //distance from floor to mid rung (60.25 inches)
const double K_lift_min_YD = 0; //it crunch
const double K_lift_enable_auto_YD = 150; //distance the lift must be above to allow the driver to enable the auto climb
const double K_lift_deadband_YD = 1.1; //it's a deadband for the y lift yeah
const double K_lift_driver_up_rate_YD = 1.2; // This is the amount of traversal added per loop (0.02 sec)
const double K_lift_driver_down_rate_YD = 0.3; // This is the amount of traversal added per loop (0.02 sec)
const double K_lift_driver_manual_up_YD = 0.25; // Manual override power
const double K_lift_driver_manual_down_YD = -0.25; // Manual override power

const double K_lift_S3_XD = 30; //move XD onto the rungs
const double K_lift_S4_XD = 30; //stays the same
const double K_lift_S5_XD = 32; //tilt the robot
const double K_lift_S6_XD = 34; //connect YD with the upper rungs
const double K_lift_S7_XD = 133; //
const double K_lift_S8_XD = 133; //stays the same
const double K_lift_S9_XD = 122; //
const double K_lift_S10_XD = 122; //stays the same
const double K_lift_S11_XD = 0; //

const double K_lift_max_XD = 133; //distance between bars (24 inches)
const double K_lift_min_XD = 0; //we don't want XD to past this or it crunch
const double K_lift_deadband_XD = 0.7; //it's a deadband for the x lift yeah
const double K_lift_driver_manual_forward_XD = 0.15; // Manual override power
const double K_lift_driver_manual_back_XD = -0.15; // Manual override power

const double K_Lift_deadband_timer = 0.035; //keep the deadband for a certain amount of time [seconds]

const double K_LiftPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,      // kP
                                                   0.000001, // kI
                                                   0.002000, // kD
                                                   0.0,      // kIz
                                                   0.0,      // kFF
                                                   1.0,      // kMaxOutput
                                                  -1.0,      // kMinOutput
                                                   1.35,     // kMaxVel - 0.93
                                                   0.5,      // kMinVel
                                                   0.0,      // kMaxAcc
                                                   0.0};     // kAllErr



/* Ball handler (BH) cals: */
const double K_IntakePower = 0.7; // Amount of power to apply to intake wheels.  Must be 0 to 1.

const double K_ElevatorPowerUp = 0.9; // Amount of power to apply to elevator band when commanded up.  Must be 0 to 1.

const double K_ElevatorPowerDwn = -0.9; // Amount of power to apply to elevator band when commanded down.  Must be -1 to 0.

/* K_BH_LauncherMinCmndSpd: Min speed for launcher control.  Below this speed, launcher will transition to 0 power. */
const double K_BH_LauncherMinCmndSpd = 10;

/* K_BH_LauncherPID_Gx: PID gains for the launcher. */
const double K_BH_LauncherPID_Gx[E_PID_SparkMaxCalSz] = { 0.00055,  // kP
                                                          0.000001, // kI
                                                          0.0,      // kD
                                                          0.0,      // kIz
                                                          0.0,      // kFF
                                                          1.0,      // kMaxOutput
                                                         -1.0,      // kMinOutput
                                                          0.0,      // kMaxVel
                                                          0.0,      // kMinVel
                                                         55.0,      // kMaxAcc
                                                          0.0};     // kAllErr

/* K_BH_LauncherSpeedAxis: Launcher speed axis for K_BH_LauncherSpeed.  Distance is in the unit from the camera.  Comments reflect actual measured distance. */
const double K_BH_LauncherSpeedAxis[10] = {415,   // 3 ft
                                           644,   // 5 ft
                                           840,   // 7 ft
                                           975,   // 9 ft
                                           1077,  // 11 ft
                                           1090,  // 13 ft
                                           1100,  // 15 ft
                                           1120,  // 17 ft
                                           1130,  // 19 ft
                                           1150}; // 21 ft

/* K_BH_LauncherSpeed: Launcher speed necessary for ball to reach target based on the estimated distance from the camera. */
const double K_BH_LauncherSpeed[10] = {3300,  // 3 ft 
                                       3300,  // 5 ft
                                       3550,  // 7 ft
                                       3750,  // 9 ft
                                       4000,  // 11 ft
                                       4300,  // 13 ft
                                       4550,  // 15 ft
                                       4700,  // 17 ft
                                       5000,  // 19 ft
                                       5300}; // 21 ft

/* K_BH_LauncherManualDb: Deadband around the manual ball launcher axis. */
const double K_BH_LauncherManualDb = 0.1;

/* K_BH_LauncherManualSpeed: Manual launcher speed control values. */
const double K_BH_LauncherManualSpeed[5] = {0,
                                            3300,
                                            4000,
                                            4500,
                                            5300};

/* K_BH_LauncherManualSpeedAxis: Axis for K_BH_LauncherManualSpeed. */
const double K_BH_LauncherManualSpeedAxis[5] = {0.00,
                                                0.25,
                                                0.50,
                                                0.75,
                                                1.00};

/* K_BH_LauncherSpeedDb: Deadband around the commanded launcher speed (in RPM).  
                         Used to indicate when a ball can be launched. */
const double K_BH_LauncherSpeedDb = 50;



// Constants and cals for Swerve Drive (SD) control:
/* C_SD_L: Robot wheelbase. [meters] */
const double C_SD_L = 0.5969;

/* C_SD_W: Robot track width. [meters] */
const double C_SD_W = 0.5969;

/* C_SD_R: Constant composed of the C_SD_W and C_SD_L constants: R = sqrt(L^2 + W^2) [meters]*/
const double C_SD_R = 0.8441;

/* K_SD_WheelOffsetAngle: Offset angle for each respective corder of the swerve drive wheel.  This is the angle 
   reading from the absolute encoder that is indicated in order for the wheel to point straight. */
const double K_SD_WheelOffsetAngle[E_RobotCornerSz] = {169.527239,   // E_FrontLeft
                                                       128.487963,   // E_FrontRight 
                                                        33.112801,   // E_RearLeft
                                                       246.813891};  // E_RearRight 

/* K_SD_MinGain: Min gain applied to the wheel speed for swerve drive. */
const double K_SD_MinGain = 0.1;

/* K_SD_MaxGain: Max gain allowed for swerve drive control. */
const double K_SD_MaxGain = 0.75;

/* K_SD_AutoRotateGx: Gain applied to the rotate command for auto functionality. */
const double K_SD_AutoRotateGx = 0.1;

/* K_SD_WheelMaxSpeed: Max in/sec speed of the swerve drive wheel.*/
const double K_SD_WheelMaxSpeed = 225;

/* K_SD_WheelMinCmndSpeed: Min in/sec speed of the swerve drive wheel to keep it under PID control.  
  If the absolute value of the command, wheels will transition to 0 power (but still in brake 
  mode).  There is a corresponding actual speed threshold. [in/sec] */
const double K_SD_WheelMinCmndSpeed = 0.2;

/* K_SD_WheelMinActualSpeed: Min in/sec speed of the swerve drive wheel to keep it under PID control.  
  If the absolute value of the actual speed, wheels will transition to 0 power (but still in brake 
  mode).  There is a corresponding commanded speed threshold. [in/sec] */
const double K_SD_WheelMinActualSpeed = 3;

/* K_SD_WheelSpeedPID_V2_Gx: PID gains for the driven wheels that is within the motor controllers. */
const double K_SD_WheelSpeedPID_V2_Gx[E_PID_SparkMaxCalSz] = { 0.00055,  // kP
                                                               0.000001, // kI
                                                               0.0001,   // kD
                                                               0.0,      // kIz
                                                               0.0,      // kFF
                                                               1.0,      // kMaxOutput
                                                              -1.0,      // kMinOutput
                                                               0.0,      // kMaxVel
                                                               0.0,      // kMinVel
                                                              45.0,      // kMaxAcc
                                                               0.0};     // kAllErr

/* K_SD_WheelSpeedPID_Gx: PID gains for the driven wheels.   PID control is within the RoboRio.  */
double const K_SD_WheelSpeedPID_Gx[E_PID_CalSz] = { 0.009,     // P Gx
                                                    0.0009,     // I Gx
                                                    0.00000005, // D Gx
                                                    1.0,        // P UL
                                                   -1.0,        // P LL
                                                    0.5,        // I UL
                                                   -0.5,        // I LL
                                                    0.2,        // D UL
                                                   -0.2,        // D LL
                                                    1.0,        // Max upper
                                                   -1.0};       // Max lower

/* K_SD_WheelAnglePID_Gx: PID gains for the angle of the swerve drive wheels.  PID control is within the RoboRio.  */
const double K_SD_WheelAnglePID_Gx[E_PID_CalSz] = { 0.007,     // P Gx
                                                    0.0005,    // I Gx
                                                    0.0000005, // D Gx
                                                    1.0,       // P UL - 0.4
                                                   -1.0,       // P LL - -0.4
                                                    0.1000,      // I UL - 0.1
                                                   -0.1000,      // I LL - -0.1
                                                    1.0,       // D UL 0.5
                                                   -1.0,       // D LL -0.5
                                                    1.0,       // Max upper 0.9
                                                   -1.0};      // Max lower -0.9

/* K_SD_DesiredDriveSpeedAxis: Joystick scale axis for K_SD_DesiredDriveSpeed.  */
const double K_SD_DesiredDriveSpeedAxis[20] = {-0.95,
                                            -0.85,
                                            -0.75,
                                            -0.65,
                                            -0.55,
                                            -0.45,
                                            -0.35,
                                            -0.25,
                                            -0.15,
                                            -0.10,
                                             0.10,
                                             0.15,
                                             0.25,
                                             0.35,
                                             0.45,
                                             0.55,
                                             0.65,
                                             0.75,
                                             0.85,
                                             0.95};

/* K_SD_DesiredDriveSpeed: Joystick scaled output for swerve drive control.  Used as debouncing and to help limit speeds at lower joystick inputs values.  */
const double K_SD_DesiredDriveSpeed[20] = {-1.00,  //-0.95
                                        -0.88,  //-0.85
                                        -0.77,  //-0.75
                                        -0.66,  //-0.65
                                        -0.55,  //-0.55
                                        -0.44,  //-0.45
                                        -0.33,  //-0.35
                                        -0.22,  //-0.25
                                        -0.11,  //-0.15
                                         0.00,  //-0.10
                                         0.00,  // 0.10
                                         0.11,  // 0.15
                                         0.22,  // 0.25
                                         0.33,  // 0.35
                                         0.44,  // 0.45
                                         0.55,  // 0.55
                                         0.66,  // 0.65
                                         0.77,  // 0.75
                                         0.88,  // 0.85
                                         1.00}; // 0.95



/* ADAS Cals */
/* Upper Targeting Cals (UT) */
/* K_ADAS_UT_LightDelayTIme - Amount of time wait for the camera to have sufficent light before proceeding. [Seconds] */
const double K_ADAS_UT_LightDelayTIme = 0.060;

/* K_ADAS_UT_LostTargetGx - When the camera has lost the target, the previous error value will be used,
   but multiplied against this gain so that we don't go too far before getting another good value. */
const double K_ADAS_UT_LostTargetGx = 0.25;

/* K_ADAS_UT_NoTargetError - When we haven't seen anything from the camera, take a guess.  This will 
   be the percieved error value until we see something good. */
const double K_ADAS_UT_NoTargetError = 20;

/* K_ADAS_UT_DebounceTime - Debounce time to hold a given state before preceding to next step. [Seconds] */
const double K_ADAS_UT_DebounceTime = 0.080;

/* K_ADAS_UT_AllowedLauncherError - Amount of error allowed in launcher speed before attempting to launch balls. [RPM] */
const double K_ADAS_UT_AllowedLauncherError = 100;

/* K_ADAS_UT_AllowedLauncherTime - Amount of time to remain in auto elevator mode.  For auton only. [Seconds] */
const double K_ADAS_UT_AllowedLauncherTime = 5;

/* K_ADAS_UT_RotateDeadbandAngle: Deadband angle for upper targeting */
const double K_ADAS_UT_RotateDeadbandAngle = 0.5;

/* K_ADAS_UT_TargetVisionAngle: This is the desired target angle for the auto vision targeting.  This is due to the offset of the camera. For 2020 - 3.3 */
const double K_ADAS_UT_TargetVisionAngle = 0.0;

/* K_ADAS_BT_LightDelayTIme - Amount of time wait for the camera to have sufficent light before proceeding. [Seconds] */
const double K_ADAS_BT_LightDelayTIme = 0.060;

/* K_ADAS_BT_LostTargetGx - When the camera has lost the target, the previous error value will be used,
   but multiplied against this gain so that we don't go too far before getting another good value. */
const double K_ADAS_BT_LostTargetGx = 0.25;

/* K_ADAS_BT_NoTargetError - When we haven't seen anything from the camera, take a guess.  This will 
   be the percieved error value until we see something good. */
const double K_ADAS_BT_NoTargetError = 20;

/* K_ADAS_BT_DebounceTime - Debounce time to hold a given state before preceding to next step. [Seconds] */
const double K_ADAS_BT_DebounceTime = 0.080;

/* K_ADAS_BT_RotateDeadbandAngle: Deadband angle for ball targeting */
const double K_ADAS_BT_RotateDeadbandAngle = 0.5;

/* K_ADAS_BT_TargetVisionAngle: This is the desired target angle for the auto ball vision targeting.  This is due to the offset of the camera. */
const double K_ADAS_BT_TargetVisionAngle = 2.0;

/* K_ADAS_BT_DriveTimeAxis: This is the estimated distance from the camera that will be scaled against the drive time table K_ADAS_BT_DriveTime. [meters] */
const double K_ADAS_BT_DriveTimeAxis[6] = {0,
                                          1,
                                          2,
                                          3,
                                          4,
                                          5};

/* K_ADAS_BT_DriveTime: This is the amount of time to drive forward to capture the ball based on the estimated distance. [seconds] */
const double K_ADAS_BT_DriveTime[6] = {0.8,
                                       1.5,
                                       2.0,
                                       2.5,
                                       3.0,
                                       3.4};

/* K_ADAS_BT_MaxTimeToWaitForCamera: This is the max amount of time we will wait for a valid distance from the camera. [Seconds] */
const double K_ADAS_BT_MaxTimeToWaitForCamera = 0.5;

/* K_ADAS_BT_TimedOutDriveForward: This is the default drive forward time when we have waited too long for the camera. [Seconds] */
const double K_ADAS_BT_TimedOutDriveForward = 1.0;

/* K_ADAS_BT_DriveForwardPct: This is the percent of swerve drive control to go forward to pickup the ball. */
const double K_ADAS_BT_DriveForwardPct = 0.8;



/*  Rotation calibrations */
/* K_DesiredRotateSpeedAxis - This is the effective command axis, function of error calculation, in degrees */
const double K_DesiredRotateSpeedAxis[10] = {-20.0,
                                              -4.0,
                                              -2.0,
                                              -1.0,
                                              -0.2,
                                               0.2,
                                               1.0,
                                               2.0,
                                               4.0,
                                              20.0};

/* K_DesiredRotateSpeed - This is the effective command, equivalent to the rotate joystick */
const double K_DesiredRotateSpeed[10] = {-0.5,  // -20.0
                                         -0.2,  //  -4.0
                                         -0.06,  //  -2.0
                                         -0.05,  //  -1.0
                                          0.0,  //  -0.2
                                          0.0,  //   0.2
                                          0.05,  //   1.0
                                          0.06,  //   2.0
                                          0.2,  //   4.0
                                          0.5}; //  20.0

/* K_DesiredAutoRotateSpeedAxis - This is the effective command axis, function of error calculation, in degrees */
const double K_DesiredAutoRotateSpeedAxis[10] = {-4.0,
                                              -3.0,
                                              -2.0,
                                              -1.0,
                                              -0.2,
                                               0.2,
                                               1.0,
                                               2.0,
                                               3.0,
                                              4.0};

/* K_DesiredRotateSpeed - This is the effective command, equivalent to the rotate joystick */
const double K_DesiredAutoRotateSpeed[10] = {0,  // -4.0
                                         0,  //  -3.0
                                         0,  //  -2.0
                                         0,  //  -1.0
                                         0,  //  -0.2
                                          0.0,  //   0.2
                                          0.0,  //   1.0
                                          0.0,  //   2.0
                                          0,  //   4.0
                                          0.0}; //  20.0

                                        //   const double K_DesiredAutoRotateSpeed[10] = {-0.09,  // -4.0
                                        //  -0.08,  //  -3.0
                                        //  -0.07,  //  -2.0
                                        //  -0.06,  //  -1.0
                                        //  -0.05,  //  -0.2
                                        //   0.05,  //   0.2
                                        //   0.06,  //   1.0
                                        //   0.07,  //   2.0
                                        //   0.08,  //   4.0
                                        //   0.09}; //  20.0




// This is the amount of time that we will wait to make sure we are at the correct location
const double K_RotateDebounceTime = 0.06;  

// This is the amount of error allowed when in auto rotate / auto target
const double K_RotateDeadbandAngle = 0.5;  

// This is the desired target angle for the auto vision targeting.  This is due to the offset of the camera. For 2020 - 3.3
const double K_TargetVisionAngleUpper = 0.0;

// This is the desired target angle for the auto ball vision targeting.  This is due to the offset of the camera.
const double K_TargetVisionAngleLower = 2.0;



const double K_TargetVisionAngleMin = 10;

const double K_TargetVisionAngleMax = 50;

const double K_TargetVisionDistanceMin = 50;

const double K_TargetVisionDistanceMax = 50;

const double K_TargetVisionAngleErrorMax = 2;

const double K_TargetVisionUpperRollerErrorMax = 200;

const double K_TargetVisionLowerRollerErrorMax = 200;





#define K_BallLauncherDistanceSz 5
#define K_BallLauncherAngleSz 3

const double K_BallLauncherDistanceAxis[K_BallLauncherDistanceSz] = {400, 700, 968, 1300, 1660};

const double K_BallLauncherAngleAxis[K_BallLauncherAngleSz] = {-45, 0, 45};

const double K_BallLauncherRobotAngle[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {45, 0, -45},
    {45, 0, -45},
    {45, 0, -45},
    {45, 0, -45},
    {45, 0, -45}
  };

const double K_BallLauncherUpperSpeed[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {-1800, -1800, -1800},
    {-1800, -1800, -1800},
    {-1800, -1800, -1800},
    {-2000, -2000, -2000},
    {-2850, -2850, -2850}
  };

const double K_BallLauncherLowerSpeed[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {-2100, -2100, -2100},
    {-2100, -2100, -2100},
    {-2100, -2100, -2100},
    {-2500, -2500, -2500},
    {-2900, -2900, -2900}
  };



/* Auton specific cals */
const double K_k_AutonX_PID_Gx[E_PID_CalSz] = { 0.095,       // P Gx
                                                0.000001,    // I Gx
                                                0.00012,      // D Gx
                                                0.8,       // P UL
                                               -0.8,       // P LL
                                                0.05,      // I UL
                                               -0.05,      // I LL
                                                0.5,       // D UL
                                               -0.5,       // D LL
                                                1.0,       // Max upper
                                               -1.0};      // Max lower

const double K_k_AutonY_PID_Gx[E_PID_CalSz] = { 0.095,       // P Gx
                                                0.000001,    // I Gx
                                                0.00012,      // D Gx
                                                0.8,       // P UL
                                               -0.8,       // P LL
                                                0.05,      // I UL
                                               -0.05,      // I LL
                                                0.5,       // D UL
                                               -0.5,       // D LL
                                                1.0,       // Max upper
                                               -1.0};      // Max lower

#include "MotionProfiles/K_BarrelRacing_V55A25_T.hpp"
#include "MotionProfiles/K_BarrelRacing_V55A25_X.hpp"
#include "MotionProfiles/K_BarrelRacing_V55A25_Y.hpp"
#include "MotionProfiles/K_BarrelRacing_V75A30_T.hpp"
#include "MotionProfiles/K_BarrelRacing_V75A30_X.hpp"
#include "MotionProfiles/K_BarrelRacing_V75A30_Y.hpp"
#include "MotionProfiles/K_BarrelRacing_V95A35_T.hpp"
#include "MotionProfiles/K_BarrelRacing_V95A35_X.hpp"
#include "MotionProfiles/K_BarrelRacing_V95A35_Y.hpp"

#include "MotionProfiles/K_Bounce_V55A25_T.hpp"
#include "MotionProfiles/K_Bounce_V55A25_X.hpp"
#include "MotionProfiles/K_Bounce_V55A25_Y.hpp"
#include "MotionProfiles/K_Bounce_V75A30_T.hpp"
#include "MotionProfiles/K_Bounce_V75A30_X.hpp"
#include "MotionProfiles/K_Bounce_V75A30_Y.hpp"
#include "MotionProfiles/K_Bounce_V95A35_T.hpp"
#include "MotionProfiles/K_Bounce_V95A35_X.hpp"
#include "MotionProfiles/K_Bounce_V95A35_Y.hpp"

#include "MotionProfiles/K_Slalom_V55A25_T.hpp"
#include "MotionProfiles/K_Slalom_V55A25_X.hpp"
#include "MotionProfiles/K_Slalom_V55A25_Y.hpp"
#include "MotionProfiles/K_Slalom_V75A30_T.hpp"
#include "MotionProfiles/K_Slalom_V75A30_X.hpp"
#include "MotionProfiles/K_Slalom_V75A30_Y.hpp"
#include "MotionProfiles/K_Slalom_V95A35_T.hpp"
#include "MotionProfiles/K_Slalom_V95A35_X.hpp"
#include "MotionProfiles/K_Slalom_V95A35_Y.hpp"
#include "MotionProfiles/K_Slalom_V125A50_T.hpp"
#include "MotionProfiles/K_Slalom_V125A50_X.hpp"
#include "MotionProfiles/K_Slalom_V125A50_Y.hpp"