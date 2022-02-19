/*
  Driver_inputs.cpp

   Created on: Feb 05, 2022
   Author: Lauren

  Function that maps the driver inputs to the robot controls. 
 */

/******************************************************************************
 * Function:     Joystick_robot_mapping
 *
 * Description:  Main calling function for lift control.
 ******************************************************************************/

void Joystick_robot_mapping(bool  L_Driver2_buttonA,
                            bool *L_elevator_up,    
                            bool  L_Driver2_buttonB,
                            bool *L_elevator_down,
                            bool  L_Driver2_buttonRB,
                            bool *L_Lift_control,
                            bool  L_Driver2_buttonback,
                            bool *L_stops_shooter,
                            bool  L_Driver2_buttonstart,
                            bool *L_auto_setspeed_shooter,
                            bool  L_Driver1_buttonback,
                            bool *L_zero_gyro,
                            bool  L_Driver2_ButtonX,
                            bool *L_Driver_intake_in,
                            double  L_Driver2_left_Axis_y,
                            double *L_right_shooter_desired_speed,
                            double  L_Driver2_right_Axis_y,
                            double *L_left_shooter_desired_speed
                            )
{
    *L_elevator_up = L_Driver2_buttonA;
    *L_elevator_down = L_Driver2_buttonB;
    *L_zero_gyro = L_Driver1_buttonback;
    *L_stops_shooter = L_Driver2_buttonback;
    *L_auto_setspeed_shooter = L_Driver2_buttonstart;
    *L_right_shooter_desired_speed = L_Driver2_left_Axis_y;
    *L_left_shooter_desired_speed = L_Driver2_right_Axis_y;
    *L_Lift_control = L_Driver2_buttonRB,
    *L_Driver_intake_in = L_Driver2_ButtonX;
}

