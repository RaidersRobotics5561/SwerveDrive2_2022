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

void Joystick_robot_mapping(bool L_Driver2_buttonX,
                            bool *L_Lift_control) 
{
    *L_Lift_control = L_Driver2_buttonX;
}

