/*
  IO_Sensors.cpp

   Created on: Feb 17, 2022
   Author: Biggs

   Contains the code related to the reading and processing of the IO sensors:
   - IR ball detection
   - XY limit detection
   - XD limit detection

 */

bool V_BallDetectedRaw;   // Detection of a ball based on the IR sensor.
bool V_XD_LimitDetected;  // XD travel is at the limit switch.
bool V_YD_LimitDetected;  // YD travel is at the limit switch.

/******************************************************************************
 * Function:     BallDetectionSensor
 *
 * Description:  IR sensor that detects the presence of a ball in the elevator.
 *
 ******************************************************************************/
void BallDetectionSensor(bool L_IR_SensorDetect)
  {
    V_BallDetectedRaw = L_IR_SensorDetect;
  }


/******************************************************************************
 * Function:     XD_YD_LimitSwitch
 *
 * Description:  Limit switches indicating end of travel for XD/YD lifts.
 *
 ******************************************************************************/
void XD_YD_LimitSwitch(bool L_XD_LimitSwitch,
                       bool L_YD_LimitSwitch)
  {
    V_XD_LimitDetected = L_XD_LimitSwitch;
    V_YD_LimitDetected = L_YD_LimitSwitch;
  }


/******************************************************************************
 * Function:     Read_IO_Sensors
 *
 * Description:  Main calling funciton for the IO sensors.
 *
 ******************************************************************************/
void Read_IO_Sensors(bool L_IR_SensorDetect,
                     bool L_XD_LimitSwitch,
                     bool L_XY_LimitSwitch)
  {
    BallDetectionSensor(L_IR_SensorDetect);

    XD_YD_LimitSwitch(L_XD_LimitSwitch,
                      L_XY_LimitSwitch);
  }
