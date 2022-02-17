/*
  Lift.hpp

   Created on: Feb 01, 2022
   Author: 5561

   The lift control state machine. This controls the robat to move the x and y hooks. It automously controls the robot to climb

   lift STATE machine? another government scam smh -chloe
 */

#include "Enums.hpp"

extern double       V_lift_measured_position_YD;
extern double       V_lift_measured_position_XD;
extern double       V_lift_command_YD;
extern double       V_lift_command_XD;
extern T_Lift_State V_Lift_state;

 T_Lift_State Lift_Control_Dictator(bool          L_driver_button,
                                    double        L_game_time,
                                    T_Lift_State  L_current_state,
                                    double        L_lift_measured_position_YD,
                                    double        L_lift_measured_position_XD,
                                    double       *L_lift_command_YD,
                                    double       *L_lift_command_XD,
                                    double        L_gyro_yawangledegrees);