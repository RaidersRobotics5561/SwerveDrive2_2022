/*
  BallHandler.cpp

  Created on: Feb 15, 2022
  Author: Biggs

  This file contains functions related to processing of balls, cargo, etc.
  This can include but is not limited to:
   - Intake
   - Elevator
   - Launcher
   - Targeting
 */

#include "Const.hpp"
#include <math.h>

double PDP_Current_UpperShooter = 0;
double PDP_Current_LowerShooter = 0;
double PDP_Current_UpperShooter_last = 0;
double PDP_Current_LowerShooter_last = 0;
double BallsShot = 0;
double V_elevatorValue = 0;

double       V_ShooterSpeedDesiredFinalUpper;
double       V_ShooterSpeedDesiredFinalLower;

double V_testspeed = 0;
double V_testIntake = 0;
double V_testElevator = 0;
double V_P_Gx = 0.00005;
double V_I_Gx = 0.000001;
double V_D_Gx = 0.000002;
double V_I_Zone = 0;
double V_FF = 0;
double V_Max = 1;
double V_Min = -1;


/******************************************************************************
 * Function:     BallLauncherInit
 *
 * Description:  Initialization function for the drive control.
 ******************************************************************************/
void BallHandlerInit()
  {
    int L_Index;
    V_elevatorValue = 0;
    BallsShot = 0;

      for (L_Index = E_FrontLeft;
           L_Index < E_RobotCornerSz;
           L_Index = T_RobotCorner(int(L_Index) + 1))
      {

      }
  }

/******************************************************************************
 * Function:     BallLauncher
 *
 * Description:  Contains the functionality for controlling the launch 
 *               mechanism.
 ******************************************************************************/
void BallLauncher()
  {
//    if (L_Driver_stops_shooter)
//    {
//      V_AutoShootEnable = false;
//      V_ShooterSpeedDesiredFinalUpper = 0;
//      V_ShooterSpeedDesiredFinalLower = 0;
//    }
    
//     if ((c_joyStick2.GetPOV() == 180) || 
//         (c_joyStick2.GetPOV() == 270) || 
//         (c_joyStick2.GetPOV() == 0)   || 
//         (L_Driver_auto_setspeed_shooter) || 
//         (V_AutoShootEnable == true))
//     {
//       if ((c_joyStick2.GetPOV() == 180))
//       {
//        V_ShooterSpeedDesiredFinalUpper = (-1312.5); //-1312.5
//        V_ShooterSpeedDesiredFinalLower = (-1400 * .8); //-1400
//       }
//       else if ((c_joyStick2.GetPOV() == 270))
//       {
//        V_ShooterSpeedDesiredFinalUpper = -2350;
//        V_ShooterSpeedDesiredFinalLower = -3325;
//       }
//       else if ((c_joyStick2.GetPOV() == 0))
//       {
//        V_ShooterSpeedDesiredFinalUpper = -200;
//        V_ShooterSpeedDesiredFinalLower = -200;
//       }
//       else if (L_Driver_auto_setspeed_shooter)
//       {
//         V_ShooterSpeedDesiredFinalUpper = DesiredUpperBeamSpeed(distanceTarget);
//         V_ShooterSpeedDesiredFinalLower = DesiredLowerBeamSpeed(distanceTarget);
//       }

//       V_AutoShootEnable = true;
//       V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 50);
//       V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 50);
//     }
//     else if(fabs(c_joyStick2.GetRawAxis(5)) > .05 || fabs(c_joyStick2.GetRawAxis(1)) > .05)
//     {
//       V_ShooterSpeedDesired[E_rightShooter] = L_Driver_right_shooter_desired_speed;
//       V_ShooterSpeedDesired[E_leftShooter] =  L_Driver_left_shooter_desired_speed;
//     } 
//     else 
//     {
//       V_ShooterSpeedDesired[E_rightShooter] = 0;
//       V_ShooterSpeedDesired[E_leftShooter] = 0;
//     }

//     V_testspeed = frc::SmartDashboard::GetNumber("Speed Desired",V_testspeed);
//     V_ShooterSpeedDesiredFinalUpper = V_testspeed;
//     V_ShooterSpeedDesiredFinalLower = -V_testspeed;
//     V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 40);
//     V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 40);

//     // V_P_Gx = frc::SmartDashboard::GetNumber("P_Gx", V_P_Gx);
//     // V_I_Gx = frc::SmartDashboard::GetNumber("I_Gx", V_I_Gx);
//     // V_D_Gx = frc::SmartDashboard::GetNumber("D_Gx", V_D_Gx);
//     // V_I_Zone = frc::SmartDashboard::GetNumber("I_Zone", V_I_Zone);
//     // V_FF = frc::SmartDashboard::GetNumber("FF", V_FF);
//     // V_Max = frc::SmartDashboard::GetNumber("Max_Limit", V_Max);
//     // V_Min = frc::SmartDashboard::GetNumber("Min_Limit", V_Min);

//     // V_Steer_P_Gx = frc::SmartDashboard::GetNumber("P_Steer_Gx", V_Steer_P_Gx);
//     // V_Steer_I_Gx = frc::SmartDashboard::GetNumber("I_Steer_Gx", V_Steer_I_Gx);
//     // V_Steer_D_Gx = frc::SmartDashboard::GetNumber("D_Steer_Gx", V_Steer_D_Gx);
//     // V_Steer_I_Zone = frc::SmartDashboard::GetNumber("I_Steer_Zone", V_Steer_I_Zone);
//     // V_Steer_FF = frc::SmartDashboard::GetNumber("FF_Steer", V_Steer_FF);
//     // V_Steer_Max = frc::SmartDashboard::GetNumber("Max_Limit_Steer", V_Steer_Max);
//     // V_Steer_Min = frc::SmartDashboard::GetNumber("Min_Limit_Steer", V_Steer_Min);

//     // V_Drive_P_Gx = frc::SmartDashboard::GetNumber("P_Gx_Drive", V_Drive_P_Gx);
//     // V_Drive_I_Gx = frc::SmartDashboard::GetNumber("I_Gx_Drive", V_Drive_I_Gx);
//     // V_Drive_D_Gx = frc::SmartDashboard::GetNumber("D_Gx_Drive", V_Drive_D_Gx);
//     // V_Drive_I_Zone = frc::SmartDashboard::GetNumber("I_Zone_Drive", V_Drive_I_Zone);
//     // V_Drive_FF = frc::SmartDashboard::GetNumber("FF_Drive", V_Drive_FF);
// V_Drive_Max = frc::SmartDashboard::GetNumber("Max_Limit_Drive", V_Drive_Max);
// V_Drive_Min = frc::SmartDashboard::GetNumber("Min_Limit_Drive", V_Drive_Min);

//     V_P_Gx = 0;
//     V_I_Gx = 0;
//     V_D_Gx = 0;
//     V_I_Zone = 0;
//     V_FF = 0;
//     V_Max = 0;
//     V_Min = 0;

//     V_Steer_P_Gx = 0;
//     V_Steer_I_Gx = 0;
//     V_Steer_D_Gx = 0;
//     V_Steer_I_Zone = 0;
//     V_Steer_FF = 0;
//     V_Steer_Max = 0;
//     V_Steer_Min = 0;

//     V_Drive_P_Gx = 0;
//     V_Drive_I_Gx = 0;
//     V_Drive_D_Gx = 0;
//     V_Drive_I_Zone = 0;
//     V_Drive_FF = 0;
//     V_Drive_Max = 0;
//     V_Drive_Min = 0;


//     m_rightShooterpid.SetP(V_P_Gx);
//     m_rightShooterpid.SetI(V_I_Gx);
//     m_rightShooterpid.SetD(V_D_Gx);
//     m_rightShooterpid.SetIZone(V_I_Zone);
//     m_rightShooterpid.SetFF(V_FF);
//     m_rightShooterpid.SetOutputRange(V_Min, V_Max);

//     m_leftShooterpid.SetP(V_P_Gx);
//     m_leftShooterpid.SetI(V_I_Gx);
//     m_leftShooterpid.SetD(V_D_Gx);
//     m_leftShooterpid.SetIZone(V_I_Zone);
//     m_leftShooterpid.SetFF(V_FF);
//     m_leftShooterpid.SetOutputRange(V_Min, V_Max);

// V_testElevator = frc::SmartDashboard::GetNumber("Elevator Power",V_testElevator);
  }

/******************************************************************************
 * Function:     BallIntake
 *
 * Description:  Contains the functionality for controlling the intake 
 *               mechanism.
 ******************************************************************************/
void BallIntake()
  {
    //   V_testIntake = frc::SmartDashboard::GetNumber("Intake Power",V_testIntake);
  }

/******************************************************************************
 * Function:     BallElevator
 *
 * Description:  Contains the functionality for controlling the elevator 
 *               mechanism.
 ******************************************************************************/
void BallElevator()
  {
    // bool activeBeamSensor = ir_sensor.Get();
    // // frc::SmartDashboard::PutBoolean("ir beam", activeBeamSensor);

    // if(L_Driver_elevator_up)
    // {
    //   if(activeBeamSensor)
    //   {
    //     m_elevator.Set(ControlMode::PercentOutput, 1);
    //   }
    //   else
    //   {
    //     m_elevator.Set(ControlMode::PercentOutput, 1);
    //   }    
    // }
    // else if(L_Driver_elevator_down)
    // {
    //   m_elevator.Set(ControlMode::PercentOutput, -0.420);
    // }
    // else
    // {
    //   m_elevator.Set(ControlMode::PercentOutput, 0);
    // }
  }

/******************************************************************************
 * Function:     BallHandlerControlMain
 *
 * Description:  Contains the functionality for controlling the launch 
 *               mechanism.
 ******************************************************************************/
void BallHandlerControlMain()
  { 
    BallIntake();
    BallElevator();
    BallLauncher();
  }