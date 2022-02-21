/*
  BallHandler.hpp

  Created on: Feb 15, 2022
  Author: Biggs

  This header file contains functions related to processing of balls, cargo, etc.
  This can include but is not limited to:
   - Intake
   - Elevator
   - Launcher
   - Targeting
 */

extern double V_IntakePowerCmnd;
extern double V_ElevatorPowerCmnd;
extern double V_ShooterRPM_Cmnd;

void BallHandlerControlMain(bool L_IntakeCmnd,
                            bool L_BallDetected,
                            bool L_ElevatorCmndUp,
                            bool L_ElevatorCmndDwn,
                            bool L_DisableShooter,
                            bool L_AutoShootReq,
                            bool L_AutoRotateComplete,
                            bool L_TopTargetAquired,
                            double L_TopTargetDistanceMeters,
                            double L_LauncherCurrentSpeed,
                            double L_ManualShooter,
                            double *L_Intake,
                            double *L_Elevator,
                            double *L_Shooter);

void BallHandlerInit(void);
