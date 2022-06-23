/*
  IO_Sensors.cpp

   Created on: Feb 17, 2022
   Author: Biggs

   Contains the code related to the reading and processing of the IO sensors:
   - IR ball detection
   - XY limit detection
   - XD limit detection

 */

bool V_BallDetectedUpper;   // Detection of a ball based on the IR sensor.
bool V_BallDetectedLower; // Detection of a ball in the lower part of elevator.
bool V_XD_LimitDetected;  // XD travel is at the limit switch.
bool V_YD_LimitDetected;  // YD travel is at the limit switch.
bool V_Turret_LimitDetected;  // Turret limit detected.

/******************************************************************************
 * Function:     IO_SensorsInit
 *
 * Description:  Init calling funciton for the IO sensors.
 *
 ******************************************************************************/
void IO_SensorsInit()
  {
  V_BallDetectedUpper = false;
  V_XD_LimitDetected = false;
  V_YD_LimitDetected = false;
  V_BallDetectedLower = false;
  V_Turret_LimitDetected = false;
  }

/******************************************************************************
 * Function:     BallDetectionSensor
 *
 * Description:  IR sensor that detects the presence of a ball in the elevator.
 *
 ******************************************************************************/
void BallDetectionSensor(bool L_IR_SensorDetect,
                         bool L_BallSensorLower)
  {
    bool L_BallDetected = false;
    bool L_BallDetectedLower = false;

    if (L_IR_SensorDetect == false)
      {
      L_BallDetected = true;
      }

    if (L_BallSensorLower == false)
      {
      L_BallDetectedLower = true;
      }
    
    V_BallDetectedUpper = L_BallDetected;

    V_BallDetectedLower = L_BallDetectedLower;
  }



/******************************************************************************
 * Function:     ReadLimitSwitchs
 *
 * Description:  Limit switches for the following indications:
 *               - end of travel for XD/YD lifts
 *               - end of travel for turret
 *
 ******************************************************************************/
void ReadLimitSwitchs(bool L_XD_LimitSwitch,
                      bool L_YD_LimitSwitch,
                      bool L_TurretLimitDetected)
  {
    V_XD_LimitDetected = L_XD_LimitSwitch;
    V_YD_LimitDetected = L_YD_LimitSwitch;
    V_Turret_LimitDetected = L_TurretLimitDetected;
  }


/******************************************************************************
 * Function:     Read_IO_Sensors
 *
 * Description:  Main calling funciton for the IO sensors.
 *
 ******************************************************************************/
void Read_IO_Sensors(bool L_IR_SensorDetect,
                     bool L_BallSensorLower,
                     bool L_XD_LimitSwitch,
                     bool L_XY_LimitSwitch,
                     bool L_TurretLimitDetected)
  {
    BallDetectionSensor(L_IR_SensorDetect,
                        L_BallSensorLower);

    ReadLimitSwitchs(L_XD_LimitSwitch,
                     L_XY_LimitSwitch,
                     L_TurretLimitDetected);
  }
