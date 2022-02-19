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

void BallHandlerControlMain(bool L_IntakeCmnd,
                            bool L_BallDetected,
                            bool L_ElevatorCmndUp,
                            bool L_ElevatorCmndDwn,
                            double L_ManualShooter,
                            double *L_Intake,
                            double *L_Elevator,
                            double *L_Shooter);

void BallHandlerInit(void);
