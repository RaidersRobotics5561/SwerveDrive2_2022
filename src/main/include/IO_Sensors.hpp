/*
  IO_Sensors.hpp

  Created on: Feb 17, 2022
  Author: Biggs

  Contains the code related to the reading and processing of the gyro output.
 */

extern bool V_BallDetectedUpper;
extern bool V_BallDetectedLower;
extern bool V_XD_LimitDetected;
extern bool V_YD_LimitDetected;

void IO_SensorsInit();

void Read_IO_Sensors(bool L_IR_SensorDetect,
                     bool L_BallSensorLower,
                     bool L_XD_LimitSwitch,
                     bool L_XY_LimitSwitch,
                     bool L_TurretLimitDetected);