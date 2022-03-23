/*
  Auton.cpp

  Created on: Feb 28, 2021
  Author: 5561

  Changes:
  2021-02-28 -> Beta
 */

#include <math.h>

#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Const.hpp"
#include <frc/DriverStation.h>

// #include "BallHandler.hpp"

double distanceTarget;
int    theCoolerInteger;
double V_autonTimer = 0;
int    V_autonState = 0;
bool V_autonTargetCmd = false;
bool V_autonTargetFin = false;

/******************************************************************************
 * Function:     AutonMainController
 *
 * Description:  Main calling function for the auton control.
 *
 ******************************************************************************/
void AutonDriveMain()
  {          
    
    double timeleft = frc::DriverStation::GetInstance().GetMatchTime();

      switch (theCoolerInteger){
        case 1:
          if ((timeleft > 13) && (timeleft <= 15)){
              //drive forward
          }
          else if ((timeleft > 9) && timeleft <= 13) {
            //auto pick up da balls function (+ some elevator)
          }
          else if ((timeleft > 7) && timeleft <= 9) {
            //rotate robot 180 degrees
          }
          else if ((timeleft > 5) && timeleft <= 7) {
            //auto align to shooter
          }
          else if ((timeleft > 0) && timeleft <= 5) {
            //shoot the balls (shooter + elevator)
          }


      }

        //   switch (theCoolerInteger)
    //   {
    //     case 1:
    //       if(timeleft > 8)
    //       {
    //         V_ShooterSpeedDesiredFinalUpper = -1400;
    //         V_ShooterSpeedDesiredFinalLower = -1250;
    //         // V_ShooterSpeedDesiredFinalUpper = DesiredUpperBeamSpeed(distanceTarget);
    //         // V_ShooterSpeedDesiredFinalLower = DesiredLowerBeamSpeed(distanceTarget);
    //       }
    //       else
    //       {
    //         V_ShooterSpeedDesiredFinalUpper = 0;
    //         V_ShooterSpeedDesiredFinalLower = 0;
    //       }
    //       V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 50);
    //       V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 50);

    //       if(timeleft < 13 && timeleft > 8)
    //       {
    //         m_elevator.Set(ControlMode::PercentOutput, 0.420);
    //       }
    //       else
    //       {
    //         m_elevator.Set(ControlMode::PercentOutput, 0);
    //       }

    //       if(timeleft < 8 && timeleft > 4)
    //       {
    //         driveforward = (0.85);
    //       }

    //       break;

    //     case 2:
    //       if(timeleft > 10)
    //       {
    //         V_ShooterSpeedDesiredFinalUpper = DesiredUpperBeamSpeed(distanceTarget);
    //         V_ShooterSpeedDesiredFinalLower = DesiredLowerBeamSpeed(distanceTarget);
    //       }
    //       else
    //       {
    //         V_ShooterSpeedDesiredFinalUpper = 0;
    //         V_ShooterSpeedDesiredFinalLower = 0;
    //       }
    //       V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 50);
    //       V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 50);

    //       if(timeleft < 14.5 && timeleft > 10)
    //       {
    //         m_elevator.Set(ControlMode::PercentOutput, 0.8);
    //       }
    //       else
    //       {
    //         m_elevator.Set(ControlMode::PercentOutput, 0);
    //       }

    //       if(timeleft < 15 && timeleft > 10)
    //       {
    //         driveforward = (0.420);
    //       }

    //       break;
        
    //   case 3:
      
    //   if(V_autonState == 0)
    //       {
    //         driveforward = (1);
    //         V_autonTimer += C_ExeTime;
    //         if (V_autonTimer >= 2){
    //           driveforward = (0);
    //           V_autonState++;
    //           V_autonTimer = 0;
    //         }
    //       }
    //   else if(V_autonState == 1)
    //       {
    //         V_autonTargetCmd = true;
    //         if (V_autonTargetFin == true){
    //             V_autonTargetCmd = false;
    //             V_autonTargetFin = false;
    //             V_autonState++;
    //         }
    //       }
      
    //   else if(V_autonState == 2)
    //       {
    //         V_ShooterSpeedDesiredFinalUpper = DesiredUpperBeamSpeed(distanceTarget);
    //         V_ShooterSpeedDesiredFinalLower = DesiredLowerBeamSpeed(distanceTarget);
    //         V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 50);
    //         V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 50);
  
    //           V_autonTimer += C_ExeTime;
    //           if (V_autonTimer >= 1){
    //           V_autonState++;
    //           V_autonTimer = 0;
    //           }
    //       }
    //   else if (V_autonState == 3){
    //     V_elevatorValue = 0.8;
    //     V_autonTimer += C_ExeTime;
    //         if (V_autonTimer >= 1.5){
    //           V_elevatorValue = 0;
    //           V_autonState++;
    //           V_autonTimer = 0;
    //         }
    //     }
    //   if(V_autonState == 4)
    //       {
    //         strafe = (-1.0);
    //         V_ShooterSpeedDesiredFinalUpper = 0;
    //         V_ShooterSpeedDesiredFinalLower = 0;
    //         V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 50);
    //         V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 50);
    //         V_autonTimer += C_ExeTime;
    //         if (V_autonTimer >= 2){
    //           strafe = (0);
    //           V_autonState++;
    //           V_autonTimer = 0;
    //         }
    //       }
    //   else if(V_autonState == 5)
    //       {
    //         V_autonTargetCmd = true;
    //         if (V_autonTargetFin == true){
    //             V_autonTargetCmd = false;
    //             V_autonTargetFin = false;
    //             V_autonState++;
    //         }
    //       }
      
    //   else if(V_autonState == 6)
    //       {
    //         V_ShooterSpeedDesiredFinalUpper = DesiredUpperBeamSpeed(distanceTarget);
    //         V_ShooterSpeedDesiredFinalLower = DesiredLowerBeamSpeed(distanceTarget);
    //         V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 50);
    //         V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 50);
  
    //           V_autonTimer += C_ExeTime;
    //           if (V_autonTimer >= 1){
    //           V_autonState++;
    //           V_autonTimer = 0;
    //           }
    //       }
    //   else if (V_autonState == 7){
    //     V_elevatorValue = 0.8;
    //     V_autonTimer += C_ExeTime;
    //         if (V_autonTimer >= 1.5){
    //           V_elevatorValue = 0;
    //           V_autonState++;
    //           V_autonTimer = 0;
    //         }
    //   }
    //   if(V_autonState == 8)
    //       {
    //         V_ShooterSpeedDesiredFinalUpper = 0;
    //         V_ShooterSpeedDesiredFinalLower = 0;
    //         V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 50);
    //         V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 50);
    //         strafe = (1.0);
    //         V_autonTimer += C_ExeTime;
    //         if (V_autonTimer >= 2){
    //           strafe = (0);
    //           V_autonState++;
    //           V_autonTimer = 0;
    //         }
    //       }
    //   else if(V_autonState == 9)
    //       {
    //         V_autonTargetCmd = true;
    //         if (V_autonTargetFin == true){
    //             V_autonTargetCmd = false;
    //             V_autonTargetFin = false;
    //             V_autonState++;
    //         }
    //       }
      
    //   else if(V_autonState == 10)
    //       {
    //         V_ShooterSpeedDesiredFinalUpper = DesiredUpperBeamSpeed(distanceTarget);
    //         V_ShooterSpeedDesiredFinalLower = DesiredLowerBeamSpeed(distanceTarget);
    //         V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 50);
    //         V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 50);
  
    //           V_autonTimer += C_ExeTime;
    //           if (V_autonTimer >= 1){
    //           V_autonState++;
    //           V_autonTimer = 0;
    //           }
    //       }
    //   else if (V_autonState == 11){
    //     V_elevatorValue = 0.8;
    //     V_autonTimer += C_ExeTime;
    //         if (V_autonTimer >= 1.5){
    //           V_elevatorValue = 0;
    //           V_autonState++;
    //           V_autonTimer = 0;
    //         }
    //      }
    //  else if(V_autonState == 12)
    //       {
    //         V_ShooterSpeedDesiredFinalUpper = 0;
    //         V_ShooterSpeedDesiredFinalLower = 0;
    //         V_ShooterSpeedDesired[E_rightShooter] = RampTo(V_ShooterSpeedDesiredFinalUpper, V_ShooterSpeedDesired[E_rightShooter], 50);
    //         V_ShooterSpeedDesired[E_leftShooter] = RampTo(V_ShooterSpeedDesiredFinalLower, V_ShooterSpeedDesired[E_leftShooter], 50);
    //         V_autonTimer += C_ExeTime;
    //         if (V_autonTimer >= 1){
    //           V_autonState++;
    //           V_autonTimer = 0;
    //         }
    //       }
    //  else if(V_autonState == 13){
    //       speen = 1.0;
    //       V_autonTimer += C_ExeTime;
    //     if (V_autonTimer >= 3){
    //           speen = 0.0;
    //           V_autonState++;
    //           V_autonTimer = 0;
    //         }
    //   }
    //   break;

    //   case 4:
    //     AutonDriveMain(&driveforward,
    //                    &strafe,
    //                    &speen,
    //                     V_M_RobotDisplacementY,
    //                     V_M_RobotDisplacementX,
    //                     V_GyroYawAngleDegrees,
    //                     0);

    //   // case 5 is expirimental, do not use for any real driving
    //   // probably works
    //   case 5 :
    //     GyroZero();
    //     if(V_autonState == 0)
    //       {
    //         driveforward = (.7);
    //         if (V_M_RobotDisplacementY <= -30.0){
    //           driveforward = (0.0);
    //           V_autonState++;
              
    //         }
    //       }
    //     else if(V_autonState == 1)
    //       {
    //         strafe = (.7);
    //         if (V_M_RobotDisplacementX <= -30.0){
    //           strafe = (0.0);
    //           V_autonState++;
    //         }
    //       }
    //     else if(V_autonState == 2)
    //       {
    //         driveforward = (-.7);
    //         if (V_M_RobotDisplacementY >= 0.0){
    //           driveforward = (0.0);
    //           V_autonState++;
    //         }
    //       }
    //     else if(V_autonState == 3)
    //       {
    //         strafe = (-.7);
    //         if (V_M_RobotDisplacementX >= 0.0){
    //           strafe = (0.0);
    //         }
    //       }
    //   break;
    //   }
  }









  