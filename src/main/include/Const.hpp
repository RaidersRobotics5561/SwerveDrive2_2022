#include "Enums.hpp"
#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>

// Define the desired test state here: COMP (no test), BallHandlerTest, LiftXY_Test, DriveMotorTest
#define DriveMotorTest

const double C_ExeTime = 0.02; // Set to match the the default controller loop time of 20 ms
const units::second_t C_ExeTime_t = 0.02_s; // Set to match the the default controller loop time of 20 ms

const double C_End_game_time = 30;

const double C_RadtoDeg = 57.2958;
const double C_Deg2Rad = 0.017453292519943295;
const double C_PI = 3.14159265358979;
const double C_Tau = 6.28318530717958647;

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
static const int C_Voltage_Woman = 8; //gENDER?!?!?!?!?!??!

// PWM IDs:
static const int C_VanityLight_ID = 1;

const double K_SteerMotorCurrentLimit = 25;
static const double C_EncoderToAngle = 360; // Raw output of PWM encoder to degrees




const double K_SteerDriveReductionRatio = 30; //30:1
const double K_ReductionRatio = 8.31;
const double K_WheelCircufrence = 12.566; // Circumferance of wheel, in inches

const double C_L = 0.5969;
const double C_W = 0.5969;
const double C_R = 0.8441;

const double K_lift_max_YD = 207; //distance from floor to mid rung (60.25 inches)
const double K_lift_enable_auto_YD = 180; //distance the lift must be above to allow the driver to enable the auto climb
const double K_lift_mid_YD = 60; //lift YD is aligned with lift XD
const double K_lift_min_traversal_YD = 15; //lift YD commanded value for start of handoff to XD
const double K_lift_min_YD = 0; //it crunch
// const double K_lift_rungs_YD = 15.375; //distance from rung to rung (15.375 inches)
const double K_lift_rate_up_YD = 0.001; //RampTo slope for lift up
const double K_lift_rate_down_YD = -0.001; //RampTo slope for lift down
const double K_lift_deadband_YD = 0.5; //it's a deadband for the y lift yeah
const double K_lift_driver_up_rate_YD = 0.52; // This is the amount added per loop (0.02 sec)
const double K_lift_driver_down_rate_YD = 0.25; // This is the amount added per loop (0.02 sec)
const double K_lift_driver_manual_up_YD = 1.0; // Manual override power
const double K_lift_driver_manual_down_YD = -0.25; // Manual override power

const double K_lift_max_XD = 135; //distance between bars (24 inches)
const double K_lift_travel_for_YD_handoff_XD = 90; //lift XD position to allow for robot to rotate to enage YD hook
const double K_lift_mid_XD = 30; //lift XD is aligned with lift YD
const double K_lift_min_XD = 0; //we don't want XD to move cuz it's a loser
const double K_lift_rate_forward_XD = 0.001; //RampTo slope for lift forward
const double K_lift_rate_backward_XD = -0.001; //RampTo slope for lift backward
const double K_lift_deadband_XD = 0.5; //it's a deadband for the x lift yeah
const double K_lift_driver_manual_forward_XD = 0.15; // Manual override power
const double K_lift_driver_manual_back_XD = -0.15; // Manual override power

const double K_gyro_angle_lift = -10; //robert is tilting
const double K_gyro_deadband = 2;
const double K_deadband_timer = 0.5; //keep the deadband for a certain amount of time

const double K_CameraLightDelay = 0.01; // Delay time between enabling the camera light and allowing the data feed to be used. [seconds]
const double K_CameraLightMaxOnTime = 5.0; // Max amount of time to have the camera light enabled. [seconds]

const double K_IntakePower = 0.7; // Amount of power to apply to intake wheels.  Must be 0 to 1.
const double K_ElevatorPowerUp = 0.9; // Amount of power to apply to elevator band when commanded up.  Must be 0 to 1.
const double K_ElevatorPowerDwn = -0.9; // Amount of power to apply to elevator band when commanded down.  Must be -1 to 0.

const double K_WheelOffsetAngle[E_RobotCornerSz] = {169.527239,   // E_FrontLeft
                                                    128.487963,   // E_FrontRight 152  104.6 
                                                    33.112801,   // E_RearLeft
                                                    246.813891}; // E_RearRight 180.703106  144.580063

const double K_WheelMaxSpeed = 225; // This is the max allowed speed for the wheels

const double K_WheelAnglePID_Gx[E_PID_CalSz] = { 0.007,     // P Gx
                                                 0.0005,    // I Gx
                                                 0.0000005, // D Gx
                                                 0.4,       // P UL
                                                -0.4,       // P LL
                                                 0.1000,      // I UL
                                                -0.1000,      // I LL
                                                 0.5,       // D UL
                                                -0.5,       // D LL
                                                 0.9,       // Max upper
                                                -0.9};      // Max lower

double const K_WheelSpeedPID_Gx[E_PID_CalSz] = { 0.009,     // P Gx
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

const double K_WheelSpeedPID_V2_Gx[E_PID_SparkMaxCalSz] = { 0.01,   // kP
                                                            0.0001, // kI
                                                            1.0,    // kD
                                                            0.0,    // kIz
                                                            0.0,    // kFF
                                                            1.0,    // kMaxOutput
                                                           -1.0,    // kMinOutput
                                                          200.0,    // kMaxVel
                                                         -200.0,    // kMinVel
                                                          100.0,    // kMaxAcc
                                                            0.0};   // kAllErr

const double K_RobotRotationPID_Gx[E_PID_CalSz] = { 0.07,   // P Gx
                                                    0.0,   // I Gx
                                                    0.0,    // D Gx
                                                    0.9,    // P UL
                                                   -0.9,    // P LL
                                                    0.5,    // I UL
                                                   -0.5,    // I LL
                                                    0.2,    // D UL
                                                   -0.2,    // D LL
                                                    1.0,    // Max upper
                                                   -1.0};   // Max lower

const double K_LauncherPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,    // kP
                                                       0.0001, // kI
                                                       1.0,    // kD
                                                       0.0,    // kIz
                                                       0.0,    // kFF
                                                       1.0,    // kMaxOutput
                                                      -1.0,    // kMinOutput
                                                     200.0,    // kMaxVel
                                                    -200.0,    // kMinVel
                                                      10.0,    // kMaxAcc
                                                       0.0};   // kAllErr

const double K_LiftPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,    // kP
                                                   0.0001, // kI
                                                   1.0,    // kD
                                                   0.0,    // kIz
                                                   0.0,    // kFF
                                                   1.0,    // kMaxOutput
                                                  -1.0,    // kMinOutput
                                                  20.0,    // kMaxVel
                                                 -20.0,    // kMinVel
                                                  10.0,    // kMaxAcc
                                                   0.0};   // kAllErr

const double K_DesiredDriveSpeedAxis[20] = {-0.95,
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

const double K_DesiredDriveSpeed[20] = {-1.00,  //-0.95
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

const double K_DesiredDistanceAxis[6] = {415,
                                     644,
                                     840,
                                     975,
                                     1077,
                                     1667};

const double K_DesiredSpeedUpperBeam[6] = {-1200,
                                           -1200,
                                           -1435,
                                           -1815,
                                           -1965,
                                           -3015};

const double K_DesiredSpeedLowerBeam[6] = {-1150,
                                           -1350,
                                           -1880,
                                           -2100,
                                           -2400,
                                           -3100};

const double K_DesiredLauncherSpeed[6] = {3300,
                                          3500,
                                          3700,
                                          3800,
                                          4000,
                                          4200};



const double K_DesiredLauncherManualAxis[5] = {0.00,
                                     0.25,
                                     0.50,
                                     0.75,
                                     1.00};

/* K_DesiredLauncherManualDb: Deadband around the manual ball launcher axis. */
const double K_DesiredLauncherManualDb = 0.1;

const double K_DesiredLauncherManualSpeed[5] = {0,
                                                1000,
                                                2000,
                                                3000,
                                                4000};

/* K_DesiredLauncherSpeedDb: Deadband around the desired launcher speed (in RPM).  
                             Used to indicate when a ball can be launched. */
const double K_DesiredLauncherSpeedDb = 10;

const double K_LiftYD_PID[E_PID_CalSz] = { 0.1,   // P Gx
                                           0.000002,   // I Gx
                                           0.0,    // D Gx
                                           1.0,    // P UL
                                          -1.0,    // P LL
                                           0.5,    // I UL
                                          -0.5,    // I LL
                                           0.0,    // D UL
                                           0.0,    // D LL
                                           1.0,    // Max upper
                                          -1.0};   // Max lower


const double K_LiftXD_PID[E_PID_CalSz] = { 0.1,   // P Gx
                                           0.000002,   // I Gx
                                           0.0,    // D Gx
                                           1.0,    // P UL
                                          -1.0,    // P LL
                                           0.5,    // I UL
                                          -0.5,    // I LL
                                           0.0,    // D UL
                                           0.0,    // D LL
                                           1.0,    // Max upper
                                          -1.0};   // Max lower



// This is the amount of time that we will wait to make sure we are at the correct location
const double K_RotateDebounceTime = 0.06;  

// This is the amount of error allowed when in auto rotate / auto target
const double K_RotateDeadbandAngle = 0.5;  

// This is the desired target angle for the auto vision targeting.  This is due to the offset of the camera. For 2020 - 3.3
const double K_TargetVisionAngle = 0.0;




const double K_TargetVisionAngleMin = 10;

const double K_TargetVisionAngleMax = 50;

const double K_TargetVisionDistanceMin = 50;

const double K_TargetVisionDistanceMax = 50;

const double K_TargetVisionAngleErrorMax = 2;

const double K_TargetVisionUpperRollerErrorMax = 200;

const double K_TargetVisionLowerRollerErrorMax = 200;

const double K_RotateDebounceThreshold = 0.1;

const double K_MaxGain = 0.75;

const double K_AutoRotateGx = 0.1;

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