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

double V_ShooterSpeedDesiredFinalUpper;
double V_ShooterSpeedDesiredFinalLower;
double V_ShooterSpeedDesired[E_RoboShooter];

double V_testspeed = 0;
double V_testIntake = 0;
double V_testElevator = 0;
// double V_P_Gx = 0.00005;
// double V_I_Gx = 0.000001;
// double V_D_Gx = 0.000002;
// double V_I_Zone = 0;
// double V_FF = 0;
// double V_Max = 1;
// double V_Min = -1;


/******************************************************************************
 * Function:     BallHandlerInit
 *
 * Description:  Initialization function for the drive control.
 ******************************************************************************/
void BallHandlerInit()
  {
    int L_Index;

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
double BallLauncher(bool L_AutoShootReq)
  {
//    if (L_Driver_stops_shooter)
//    {
//      L_AutoShootReq = false;
//      V_ShooterSpeedDesiredFinalUpper = 0;
//      V_ShooterSpeedDesiredFinalLower = 0;
//    }
    
//     if ((c_joyStick2.GetPOV() == 180) || 
//         (c_joyStick2.GetPOV() == 270) || 
//         (c_joyStick2.GetPOV() == 0)   || 
//         (L_Driver_auto_setspeed_shooter) || 
//         (L_AutoShootReq == true))
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

//       L_AutoShootReq = true;
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
    return (0);
  }

/******************************************************************************
 * Function:     BallIntake
 *
 * Description:  Contains the functionality for controlling the intake 
 *               mechanism.
 ******************************************************************************/
double BallIntake(bool L_DriverIntakeCmnd)
  {
    double L_IntakeMotorCmnd = 0;

    if (L_DriverIntakeCmnd == true)
    {
      L_IntakeMotorCmnd = K_IntakePower;
    }
    // Otherwise, leave at 0

    //   V_testIntake = frc::SmartDashboard::GetNumber("Intake Power",V_testIntake);
    return (L_IntakeMotorCmnd);
  }

/******************************************************************************
 * Function:     BallElevator
 *
 * Description:  Contains the functionality for controlling the elevator 
 *               mechanism.
 *
 *               ToDo: What do we want to do with the IR sensor?
 ******************************************************************************/
double BallElevator(bool L_BallDetected,
                    bool L_ElevatorCmndUp,
                    bool L_ElevatorCmndDwn)
  {
    double L_ElevatorPowerCmnd = 0;

    if(L_ElevatorCmndUp == true)
      {
      L_ElevatorPowerCmnd = K_ElevatorPowerUp;
      }
    else if(L_ElevatorCmndDwn == true)
      {
      L_ElevatorPowerCmnd = K_ElevatorPowerDwn;
      }
    // otherwise leave at 0

    return (L_ElevatorPowerCmnd);
  }


/******************************************************************************
 * Function:     BallHandlerControlMain
 *
 * Description:  Contains the functionality for controlling the launch 
 *               mechanism.
 ******************************************************************************/
void BallHandlerControlMain(bool L_IntakeCmnd,
                            bool L_BallDetected,
                            bool L_ElevatorCmndUp,
                            bool L_ElevatorCmndDwn,
                            double L_ManualShooter,
                            double *L_Intake,
                            double *L_Elevator,
                            double *L_Shooter)
  {
    double L_LauncherRPM       = 0;
    double L_IntakePowerCmnd   = 0;
    double L_ElevatorPowerCmnd = 0;

    L_IntakePowerCmnd = BallIntake(L_IntakeCmnd);
    
    L_ElevatorPowerCmnd = BallElevator(L_BallDetected,
                                       L_ElevatorCmndUp,
                                       L_ElevatorCmndDwn);

    L_LauncherRPM = BallLauncher(false);

    *L_Intake = L_IntakePowerCmnd;

    *L_Elevator = L_ElevatorPowerCmnd;

    *L_Shooter = L_LauncherRPM;
  }