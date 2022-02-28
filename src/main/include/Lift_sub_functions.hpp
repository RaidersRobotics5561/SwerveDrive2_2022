/*
  Lift_sub_functions.hpp

   Created on: Feb 05, 2022
   Author: 5561

   These are the sub functions for our lift control states machine.
 */

  bool S2_lift_down_YD(double         L_lift_measured_position_YD,
                        double         L_lift_measured_position_XD,
                        double        *L_lift_command_YD,
                        double        *L_lift_command_XD);
              
  bool S3_move_forward_XD(double         L_lift_measured_position_YD,
                          double         L_lift_measured_position_XD,
                          double        *L_lift_command_YD,
                          double        *L_lift_command_XD);  

  
  bool S4_stretch_up_YD(double         L_lift_measured_position_YD,
                        double         L_lift_measured_position_XD,
                        double        *L_lift_command_YD,
                        double        *L_lift_command_XD);   

  bool S5_more_forward_XD(double         L_lift_measured_position_YD,
                          double         L_lift_measured_position_XD,
                          double        *L_lift_command_YD,
                          double        *L_lift_command_XD);

  bool S6_lift_up_more_YD(double         L_lift_measured_position_YD,
                          double         L_lift_measured_position_XD,
                          double        *L_lift_command_YD,
                           double        *L_lift_command_XD);

  bool S7_move_back_XD(double         L_lift_measured_position_YD,
                       double         L_lift_measured_position_XD,
                       double         L_gyro_yawangledegrees,
                       double        *L_lift_command_YD,
                       double        *L_lift_command_XD);

  bool S8_more_down_some_YD(double         L_lift_measured_position_YD,
                            double         L_lift_measured_position_XD,
                            double        *L_lift_command_YD,
                            double        *L_lift_command_XD); 

  bool S9_back_rest_XD(double         L_lift_measured_position_YD,
                       double         L_lift_measured_position_XD,
                       double        *L_lift_command_YD,
                       double        *L_lift_command_XD);

  bool S10_final_YD(double         L_lift_measured_position_YD,
                    double         L_lift_measured_position_XD,
                    double        *L_lift_command_YD,
                    double        *L_lift_command_XD);    

   bool S11_final_OWO(double         L_lift_measured_position_YD,
                      double         L_lift_measured_position_XD,
                      double        *L_lift_command_YD,
                      double        *L_lift_command_XD);                             