/*
  Enums.hpp

   Created on: Jan 3, 2020
   Author: 5561
 */

#pragma once

#ifndef ENUMS
#define ENUMS

typedef enum T_RobotCorner
{
  E_FrontLeft,
  E_FrontRight,
  E_RearLeft,
  E_RearRight,
  E_RobotCornerSz
} T_RobotCorner;


typedef enum T_RoboShooter
{
  E_rightShooter,
  E_leftShooter,
  E_RoboShooter
} T_RoboShooter;


typedef enum T_PID_Cal
{
  E_P_Gx,
  E_I_Gx,
  E_D_Gx,
  E_P_Ul,
  E_P_Ll,
  E_I_Ul,
  E_I_Ll,
  E_D_Ul,
  E_D_Ll,
  E_Max_Ul,
  E_Max_Ll,
  E_PID_CalSz
} T_PID_Cal;


typedef enum T_PID_SparkMaxCal
{
  E_kP,
  E_kI,
  E_kD,
  E_kIz,
  E_kFF,
  E_kMaxOutput,
  E_kMinOutput,
  E_kMaxVel,
  E_kMinVel,
  E_kMaxAcc,
  E_kAllErr,
  E_PID_SparkMaxCalSz
} T_PID_SparkMaxCal;


typedef enum T_AutoTargetStates
{
  E_NotActive, //not doing anything
  E_TargetFoundRotateBot, //target locked
  E_RollerSpinUp, //we movin
  E_MoveBallsToRollers, // get ready
  E_AutoTargetStatesSz // 
} T_AutoTargetStates;

typedef enum T_LauncherStates
{
  E_LauncherNotActive,
  E_LauncherAutoTargetActive,
  E_LauncherManualActive,
} T_LauncherStates;

typedef enum T_CameraLightStatus
{
  E_LightTurnedOff,
  E_LightOnWaitingForTarget,
  E_LightOnTargetingReady,
  E_LightForcedOffDueToOvertime
} T_CameraLightStatus;

typedef enum T_LiftCmndDirection
{
  E_LiftCmndNone,
  E_LiftCmndUp,
  E_LiftCmndDown,
  E_LiftCmndForward,
  E_LiftCmndBack
} T_LiftCmndDirection;

typedef enum T_LED_LightCmnd
{
  E_LED_Red,
  E_LED_Blue,
  E_LED_Green,
  E_LED_Orange
} T_LED_LightCmnd;

typedef enum T_Lift_State
{
  E_S0_BEGONE,
  E_S2_lift_down_YD,
  E_S3_move_forward_XD,
  E_S4_stretch_up_YD,
  E_S5_more_forward_XD,
  E_S6_lift_up_more_YD,
  E_S7_move_back_XD,
  E_S8_more_down_some_YD,
  E_S9_back_rest_XD,
  E_S10_final_YD,
  E_S11_final_OWO,
  E_Lift_State_Sz
} T_Lift_State;


typedef enum T_RobotState
{
  E_Init,
  E_Auton,
  E_Teleop
} T_RobotState;

typedef enum T_ADAS_BT_BallTarget /* aka GetDaBalls */
{
  E_ADAS_BT_Disabled,
  E_ADAS_BT_CameraLightOn,
  E_ADAS_BT_AutoCenter,
  E_ADAS_BT_IntakeAndRun
} T_ADAS_BT_BallTarget;

typedef enum T_ADAS_UT_UpperTarget
{
  E_ADAS_UT_Disabled,
  E_ADAS_UT_CameraLightOn,
  E_ADAS_UT_AutoCenter,
  E_ADAS_UT_LauncherSpeed,
  E_ADAS_UT_ElevatorControl
} T_ADAS_UT_UpperTarget;

typedef enum T_ADAS_DM_DriveManuvering
{
  E_ADAS_DM_Disabled,
  E_ADAS_DM_DriveStraight,
  E_ADAS_DM_Rotate
} T_ADAS_DM_DriveManuvering;

typedef enum T_ADAS_ActiveFeature
{
  E_ADAS_Disabled,
  E_ADAS_AutoUpperTarget,
  E_ADAS_AutoBallTarget,
  E_ADAS_DriveStraight, // Drive straight, robot oriented
  E_ADAS_Rotate    // Rotate the robot
} T_ADAS_ActiveFeature;

#endif
