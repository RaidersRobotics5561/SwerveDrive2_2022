/*
  Lift.cpp

   Created on: Feb 01, 2022
   Author: 5561

   The lift control state machine. This controls the robat to move the x and y hooks. It automously controls the robot to climb

   lift go brrrrrrrrrrrrrrrrrrr -chloe
 */


// #include "Robot.h"
#include <iostream>
// #include <frc/smartdashboard/SmartDashboard.h>
// #include <frc/DriverStation.h>
// #include <frc/livewindow/LiveWindow.h>
#include <frc/DigitalInput.h>
// #include <units/length.h>
#include "Const.hpp"

int L_stateControl = 0;
int urMom = 0;

/******************************************************************************
 * Function:     Lift_Control_Dictator
 *
 * Description:  Main calling function for lift control.
 ******************************************************************************/
 T_Lift_State Lift_Control_Dictator(bool          L_driver_button,
                                    double        L_game_time,
                                    T_Lift_State  L_current_state,
                                    bool          L_criteria_met)
{
    T_Lift_State L_Commanded_State = L_current_state;
switch (L_current_state)
      {
        case E_S0_BEGONE:
            if (L_game_time <= C_End_game_time && L_driver_button == true) {
                L_Commanded_State = E_S1_initialize_Up_YD;
            }
        break;

      }

      return(L_Commanded_State);

}