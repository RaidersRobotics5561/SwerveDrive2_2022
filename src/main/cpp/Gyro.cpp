/*
  Gyro.cpp

   Created on: Feb 01, 2020
   Author: 5561

   Contains the code related to the reading and processing of the gyro output.

 */

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "AHRS.h"
#include "Const.hpp"

AHRS *NavX;

using namespace frc;

double V_GyroYawAngleDegrees;
double V_GyroYawAngleRad;

/******************************************************************************
 * Function:     GyroInit
 *
 * Description:  Initialization of the gyro.
 ******************************************************************************/
void GyroInit()
  {
    try
      {
      NavX = new AHRS(SPI::Port::kMXP);
      }
    catch(const std::exception e)
      {
      // std::string err_string = "Error instantiating navX-MXP:  ";
      // err_string += e.what();
      // DriverStation::ReportError(err_string.c_str());
      }

    V_GyroYawAngleDegrees = 0;
    V_GyroYawAngleRad = 0;
  }


/******************************************************************************
 * Function:     ReadGyro
 *
 * Description:  Contains the code to read the gyro.
 ******************************************************************************/
void ReadGyro(bool L_DriverZeroGyroCmnd)
  {
  if(L_DriverZeroGyroCmnd)
    {
    NavX->ZeroYaw();
    }

  V_GyroYawAngleDegrees = (double)NavX->GetYaw();

  V_GyroYawAngleRad = V_GyroYawAngleDegrees / C_RadtoDeg;

  frc::SmartDashboard::PutNumber("GYRO", V_GyroYawAngleDegrees);
  }
