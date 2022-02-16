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
#include "Lookup.hpp"
#include "Gyro.hpp"
#include "Lift.hpp"
#include "Lift_sub_functions.hpp"

T_Lift_State V_Lift_state = E_S0_BEGONE;
int V_lift_counter = 0;
bool V_init_state = false;
double V_stop_positon_XD = 0;
double V_timer_owo = 0;
int L_stateControl = 0;
bool urMomGay = true;
bool V_criteria_met = false;

double V_lift_measured_position_YD = 0;
double V_lift_measured_position_XD = 0;
double V_lift_command_YD = 0;
double V_lift_command_XD = 0;
double V_gyro_yawangledegrees = 0;

/******************************************************************************
 * Function:     Lift_Control_Dictator
 *
 * Description:  Main calling function for lift control.
 ******************************************************************************/
 T_Lift_State Lift_Control_Dictator(bool          L_driver_button,
                                    double        L_game_time,
                                    T_Lift_State  L_current_state,                                
                                    double        L_lift_measured_position_YD,
                                    double        L_lift_measured_position_XD,
                                    double       *L_lift_command_YD,
                                    double       *L_lift_command_XD,
                                    double        L_gyro_yawangledegrees)
{
    T_Lift_State L_Commanded_State = L_current_state;
switch (L_current_state)
      {
        case E_S0_BEGONE:
            if (L_game_time <= C_End_game_time && L_driver_button == true) {
                L_Commanded_State = E_S1_initialize_Up_YD;
            }
        break;

        case E_S1_initialize_Up_YD:
            V_criteria_met = S1_initialize_Up_YD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true){
              L_Commanded_State = E_S2_lift_down_YD;
            }
        break; 

        case E_S2_lift_down_YD:
            V_criteria_met = S2_lift_down_YD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S3_move_forward_XD;
            }
        break;

        case E_S3_move_forward_XD:
            V_criteria_met = S3_move_forward_XD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S4_stretch_up_YD;
            }
        break;

        case E_S4_stretch_up_YD:
            V_criteria_met = S4_stretch_up_YD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S5_more_forward_XD;
            }
        break;

        case E_S5_more_forward_XD:
            V_criteria_met = S5_more_forward_XD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S6_lift_up_more_YD;
            }
        break;

        case E_S6_lift_up_more_YD:
            V_criteria_met = S6_lift_up_more_YD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S7_move_back_XD;
            }
        break;

        case E_S7_move_back_XD:
            V_criteria_met = S7_move_back_XD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_gyro_yawangledegrees, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S7_move_back_XD;
            }
        break;

        case E_S8_more_down_some_YD:
            V_criteria_met = S8_more_down_some_YD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S9_back_rest_XD;
            }
        break;

        case E_S9_back_rest_XD:
            V_criteria_met = S9_back_rest_XD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S10_final_YD;
            }
        break;

        case E_S10_final_YD:
            V_criteria_met = S10_final_YD(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
            if(V_criteria_met == true && V_lift_counter < 1){
              L_Commanded_State = E_S3_move_forward_XD;
              V_lift_counter++;
            }
            else if(V_criteria_met == true && V_lift_counter >= 1){
              L_Commanded_State = E_S11_Stop;
            }
        break;

        case E_S11_Stop:
            V_criteria_met = S11_Stop(L_lift_measured_position_YD, L_lift_measured_position_XD, L_lift_command_YD, L_lift_command_XD);
        break;
      }

      return(L_Commanded_State);

}

/******************************************************************************
 * Function:     S1_initialize_Up_YD
 *
 * Description:  State Won: the beginning of the Y lift moters movement
 ******************************************************************************/
 bool S1_initialize_Up_YD(double         L_lift_measured_position_YD,
                          double         L_lift_measured_position_XD,
                          double        *L_lift_command_YD,
                          double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;

  *L_lift_command_YD = RampTo( K_lift_max_YD,
                               L_lift_measured_position_YD,
                               K_lift_rate_up_YD);
  
  *L_lift_command_XD = K_lift_min_XD;

  if (L_lift_measured_position_YD >= K_lift_max_YD) {
    L_criteria_met = true;
  }

  return(L_criteria_met);
}


/******************************************************************************
 * Function:     S2_lift_down_YD
 *
 * Description:  State 2: moving robert up by moving y-lift down
 ******************************************************************************/
 bool S2_lift_down_YD(double         L_lift_measured_position_YD,
                      double         L_lift_measured_position_XD,
                      double        *L_lift_command_YD,
                      double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;

  *L_lift_command_YD = RampTo( K_lift_min_YD,
                               L_lift_measured_position_YD,
                               K_lift_rate_down_YD);
  
  *L_lift_command_XD = K_lift_min_XD;

  if (L_lift_measured_position_YD <= K_lift_min_YD) {
    L_criteria_met = true;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S3_move_forward_XD,
 *
 * Description:  State 3: moving x lift haha it has to do its job
 ******************************************************************************/
 bool S3_move_forward_XD(double         L_lift_measured_position_YD,
                         double         L_lift_measured_position_XD,
                         double        *L_lift_command_YD,
                         double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;

  *L_lift_command_XD = RampTo( K_lift_mid_XD,
                               L_lift_measured_position_XD,
                               K_lift_rate_forward_XD);
  
  *L_lift_command_YD = K_lift_min_YD;

  if (L_lift_measured_position_XD >= (K_lift_mid_XD + K_lift_deadband_XD) && L_lift_measured_position_XD <= (K_lift_mid_XD - K_lift_deadband_XD)) {
    V_timer_owo += C_ExeTime;
    if (V_timer_owo >= K_deadband_timer){
          L_criteria_met = true;
          V_timer_owo = 0;
    }
  }
  else {
    V_timer_owo = 0;
  }

  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S4_stretch_up_YD,
 *
 * Description:  State 4: x lift no move, y lift go
 ******************************************************************************/
 bool S4_stretch_up_YD(double         L_lift_measured_position_YD,
                       double         L_lift_measured_position_XD,
                       double        *L_lift_command_YD,
                       double        *L_lift_command_XD)  
{
   bool L_criteria_met = false;

  *L_lift_command_YD = RampTo( K_lift_mid_YD,
                               L_lift_measured_position_YD,
                               K_lift_rate_up_YD);
  
  *L_lift_command_XD = K_lift_mid_XD;

  if (L_lift_measured_position_YD >= (K_lift_mid_YD + K_lift_deadband_YD) && L_lift_measured_position_YD <= (K_lift_mid_YD - K_lift_deadband_YD)) {
    V_timer_owo += C_ExeTime;
    if (V_timer_owo >= K_deadband_timer){
          L_criteria_met = true;
          V_timer_owo = 0;
    }
  }
  else {
    V_timer_owo = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S5_more_forward_XD,
 *
 * Description:  State 5: y lift no move, x lift go
 ******************************************************************************/
 bool S5_more_forward_XD(double         L_lift_measured_position_YD,
                         double         L_lift_measured_position_XD,
                         double        *L_lift_command_YD,
                         double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;

  *L_lift_command_XD = RampTo( K_lift_max_XD,
                               L_lift_measured_position_XD,
                               K_lift_rate_forward_XD);
  
  *L_lift_command_YD = K_lift_mid_YD;

  if (L_lift_measured_position_XD >= (K_lift_max_XD + K_lift_deadband_XD) && L_lift_measured_position_XD <= (K_lift_max_XD - K_lift_deadband_XD)) {
    V_timer_owo += C_ExeTime;
    if (V_timer_owo >= K_deadband_timer){
          L_criteria_met = true;
          V_timer_owo = 0;
    }
  }
  else {
    V_timer_owo = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S6_lift_up_more_YD,
 *
 * Description:  State 6: y lift go down, x lift bad stop what's in your mouth no get back here doN'T EAT IT
 ******************************************************************************/
 bool S6_lift_up_more_YD(double         L_lift_measured_position_YD,
                         double         L_lift_measured_position_XD,
                         double        *L_lift_command_YD,
                         double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;

  *L_lift_command_YD = RampTo( K_lift_rungs_YD,
                               L_lift_measured_position_YD,
                               K_lift_rate_down_YD);
  
  *L_lift_command_XD = K_lift_max_XD;

  if (L_lift_measured_position_YD >= (K_lift_rungs_YD + K_lift_deadband_YD) && L_lift_measured_position_YD <= (K_lift_rungs_YD - K_lift_deadband_YD)) {
    V_timer_owo += C_ExeTime;
    if (V_timer_owo >= K_deadband_timer){
          L_criteria_met = true;
          V_timer_owo = 0;
    }
  }
  else {
    V_timer_owo = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S7_move_back_XD,
 *
 * Description:  State 7: X go back-aroni, we look at gyro to make sure we aren't tilted too much
 ******************************************************************************/
 bool S7_move_back_XD(double         L_lift_measured_position_YD,
                      double         L_lift_measured_position_XD,
                      double         L_gyro_yawangledegrees,
                      double        *L_lift_command_YD,
                      double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;

  *L_lift_command_XD = RampTo( K_lift_mid_XD,
                               L_lift_measured_position_XD,
                               K_lift_rate_backward_XD);
  
  *L_lift_command_YD = K_lift_rungs_YD;

  if (L_lift_measured_position_XD >= (K_lift_mid_XD + K_lift_deadband_XD)  && L_lift_measured_position_XD <= (K_lift_mid_XD - K_lift_deadband_XD) || 
      L_gyro_yawangledegrees >= (K_gyro_angle_lift + K_gyro_deadband) && L_gyro_yawangledegrees <= (K_gyro_angle_lift - K_gyro_deadband)) {
    V_timer_owo += C_ExeTime;
     if (V_timer_owo >= K_deadband_timer){
          L_criteria_met = true;
          V_timer_owo = 0;
    }
  }
  else {
    V_timer_owo = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S8_more_down_some_YD,
 *
 * Description:  State 8: me when the lift go down more
 ******************************************************************************/
 bool S8_more_down_some_YD(double         L_lift_measured_position_YD,
                           double         L_lift_measured_position_XD,
                           double        *L_lift_command_YD,
                           double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;
  if (V_init_state == false){
    V_init_state = true;
    V_stop_positon_XD = L_lift_measured_position_XD;
  }

  *L_lift_command_YD = RampTo( K_lift_mid_YD,
                               L_lift_measured_position_YD,
                               K_lift_rate_down_YD);
  
  *L_lift_command_XD = V_stop_positon_XD;

  if (L_lift_measured_position_YD >= (K_lift_mid_YD + K_lift_deadband_YD) && L_lift_measured_position_YD <= (K_lift_mid_YD - K_lift_deadband_YD)) {
    V_timer_owo += C_ExeTime;
    if (V_timer_owo >= K_deadband_timer){
          L_criteria_met = true;
          V_timer_owo = 0;
    }
  }
  else {
    V_timer_owo = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S9_back_rest_XD
 *
 * Description:  State 9: reset it to initial x position (we aren't fixing my back  :(  )
 ******************************************************************************/
 bool S9_back_rest_XD(double         L_lift_measured_position_YD,
                      double         L_lift_measured_position_XD,
                      double        *L_lift_command_YD,
                      double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;

  *L_lift_command_XD = RampTo( K_lift_min_XD,
                               L_lift_measured_position_XD,
                               K_lift_rate_backward_XD);
  
  *L_lift_command_YD = K_lift_mid_YD;

  if (L_lift_measured_position_XD >= (K_lift_min_XD + K_lift_deadband_XD) && L_lift_measured_position_XD <= (K_lift_min_XD - K_lift_deadband_XD)) {
    V_timer_owo += C_ExeTime;
    if (V_timer_owo >= K_deadband_timer){
          L_criteria_met = true;
          V_timer_owo = 0;
    }
  }
  else {
    V_timer_owo = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S10_final_YD
 *
 * Description:  State 10: y move down, robert move up (what a chad)
 ******************************************************************************/
 bool S10_final_YD(double         L_lift_measured_position_YD,
                   double         L_lift_measured_position_XD,
                   double        *L_lift_command_YD,
                   double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;

  *L_lift_command_YD = RampTo( K_lift_min_YD,
                               L_lift_measured_position_YD,
                               K_lift_rate_down_YD);
  
  *L_lift_command_XD = K_lift_min_XD;

  if (L_lift_measured_position_YD >= (K_lift_min_YD + K_lift_deadband_YD) && L_lift_measured_position_YD <= (K_lift_min_YD - K_lift_deadband_YD)) {
    V_timer_owo += C_ExeTime;
    if (V_timer_owo >= K_deadband_timer){
          L_criteria_met = true;
          V_timer_owo = 0;
    }
  }
  else {
    V_timer_owo = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S11_Stop
 *
 * Description:  State 11: Hold the robot at it's final postion as state machine ends.
 ******************************************************************************/
 bool S11_Stop(double         L_lift_measured_position_YD,
                   double         L_lift_measured_position_XD,
                   double        *L_lift_command_YD,
                   double        *L_lift_command_XD)  
{
  bool L_criteria_met = false;

  *L_lift_command_YD = K_lift_min_YD;
  
  *L_lift_command_XD = K_lift_min_XD;
  
  return(L_criteria_met);
}