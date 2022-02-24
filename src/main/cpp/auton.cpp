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
// #include "BallHandler.hpp"

double V_t_AutonTime;
double V_L_X_ErrorPrev;
double V_L_Y_ErrorPrev;
double V_L_X_Integral;
double V_L_Y_Integral;
bool   V_b_RecordStartPosition;
double V_L_X_StartPosition;
double V_L_Y_StartPosition;
double distanceTarget;
int    theCoolerInteger;
double V_autonTimer = 0;
int    V_autonState = 0;
bool V_autonTargetCmd = false;
bool V_autonTargetFin = false;

/******************************************************************************
 * Function:     AutonDriveReset
 *
 * Description:  Reset all applicable Auton variables.
 ******************************************************************************/
void AutonDriveReset(void)
  {
      V_t_AutonTime = 0.0;
      V_L_X_ErrorPrev = 0.0;
      V_L_Y_ErrorPrev = 0.0;
      V_L_X_Integral = 0.0;
      V_L_Y_Integral = 0.0;
      V_b_RecordStartPosition = true;
      V_L_X_StartPosition = 0.0;
      V_L_Y_StartPosition = 0.0;
      V_autonTimer = 0;
      V_autonState = 0;
      V_autonTargetCmd = false;
      V_autonTargetFin = false;
  }

/******************************************************************************
 * Function:     AutonDriveMain
 *
 * Description:  Main calling function for the auton drive control.
 *               We take in the current field position, then lookup the desired 
 *               position based on the current time.  Based on this "error"
 *               between the current position and the desired position, we
 *               command either the robot to move foward/backward or to strafe
 *               to try and reach the desired position
 ******************************************************************************/
void AutonDriveMain(double *L_Pct_JoyStickFwdRev,
                    double *L_Pct_JoyStickStrafe,
                    double *L_Pct_JoyStickRotate,
                    double  L_L_X_FieldPos,
                    double  L_L_Y_FieldPos,
                    double  L_Deg_GyroAngleDeg,
                    int     L_int_AutonSelection,
                    bool    L_b_RobotInit)
  {
    double L_L_X_Location = 0.0;
    double L_L_Y_Location = 0.0;
    double L_L_FwdRevPosition = 0.0;
    double L_L_Strafe = 0.0;

    if (L_b_RobotInit == false)
      {
        DesiredAutonLocation( V_t_AutonTime,
                             &L_L_X_Location,
                             &L_L_Y_Location);
        
        if (V_b_RecordStartPosition == true)
          {
            V_L_X_StartPosition = L_L_X_Location;
            V_L_Y_StartPosition = L_L_Y_Location;
            V_b_RecordStartPosition = false;
          }

        /* We need to offset the position by the start position since the odometry will 
           start at zero, but the lookup table will not */
        L_L_FwdRevPosition = L_L_X_Location - V_L_X_StartPosition;
        L_L_Strafe = L_L_Y_Location - V_L_Y_StartPosition;

        *L_Pct_JoyStickFwdRev =  Control_PID( L_L_FwdRevPosition,
                                             -L_L_X_FieldPos,
                                             &V_L_X_ErrorPrev,
                                             &V_L_X_Integral,
                                              K_k_AutonX_PID_Gx[E_P_Gx],
                                              K_k_AutonX_PID_Gx[E_I_Gx],
                                              K_k_AutonX_PID_Gx[E_D_Gx],
                                              K_k_AutonX_PID_Gx[E_P_Ul],
                                              K_k_AutonX_PID_Gx[E_P_Ll],
                                              K_k_AutonX_PID_Gx[E_I_Ul],
                                              K_k_AutonX_PID_Gx[E_I_Ll],
                                              K_k_AutonX_PID_Gx[E_D_Ul],
                                              K_k_AutonX_PID_Gx[E_D_Ll],
                                              K_k_AutonX_PID_Gx[E_Max_Ul],
                                              K_k_AutonX_PID_Gx[E_Max_Ll]);

        *L_Pct_JoyStickStrafe =  Control_PID( L_L_Strafe,
                                             -L_L_Y_FieldPos,
                                             &V_L_Y_ErrorPrev,
                                             &V_L_Y_Integral,
                                              K_k_AutonY_PID_Gx[E_P_Gx],
                                              K_k_AutonY_PID_Gx[E_I_Gx],
                                              K_k_AutonY_PID_Gx[E_D_Gx],
                                              K_k_AutonY_PID_Gx[E_P_Ul],
                                              K_k_AutonY_PID_Gx[E_P_Ll],
                                              K_k_AutonY_PID_Gx[E_I_Ul],
                                              K_k_AutonY_PID_Gx[E_I_Ll],
                                              K_k_AutonY_PID_Gx[E_D_Ul],
                                              K_k_AutonY_PID_Gx[E_D_Ll],
                                              K_k_AutonY_PID_Gx[E_Max_Ul],
                                              K_k_AutonY_PID_Gx[E_Max_Ll]);

        V_t_AutonTime += C_ExeTime;
      }
  }

/******************************************************************************
 * Function:     AutonMainController
 *
 * Description:  Main calling function for the auton control.
 *
 ******************************************************************************/
void AutonDriveMain()
  {          
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