/*
  Encoders.cpp

  Created on: Jan 3, 2020
  Author: 5561
 */

#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Const.hpp"

double V_WheelAngle[E_RobotCornerSz];
double V_WheelAngleConverted[E_RobotCornerSz]; // This is the wheel angle coming from the NEO and processed to only be from 0 - 360
double V_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in degrees
double V_Rad_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in radians
double V_WheelAngleRev[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a reverse direction

double V_WheelVelocity[E_RobotCornerSz]; // Velocity of drive wheels, in in/sec
double V_M_WheelDeltaDistance[E_RobotCornerSz]; // Distance wheel moved, loop to loop, in inches
double V_Cnt_WheelDeltaDistanceCurr[E_RobotCornerSz]; // Prev distance wheel moved, loop to loop, in Counts
double V_Cnt_WheelDeltaDistancePrev[E_RobotCornerSz]; // Prev distance wheel moved, loop to loop, in Counts
double V_ShooterSpeedCurr;
double V_Cnt_WheelDeltaDistanceInit[E_RobotCornerSz];
double V_Delta_Angle[E_RobotCornerSz]; // The delta of the angle needed to align the wheels when the robot inits
double V_LiftPostitionYD; // Position of the YD lift, in revolutions of the motor
double V_LiftPostitionXD; // Position of the XD lift, in revolutions of the motor


/******************************************************************************
 * Function:     EncodersInit
 *
 * Description:  Initialize all of the applicable encoder variables.
 ******************************************************************************/
void EncodersInit(rev::SparkMaxRelativeEncoder m_encoderFrontRightSteer,
                  rev::SparkMaxRelativeEncoder m_encoderFrontLeftSteer,
                  rev::SparkMaxRelativeEncoder m_encoderRearRightSteer,
                  rev::SparkMaxRelativeEncoder m_encoderRearLeftSteer,
                  rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                  rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                  rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                  rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                  rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                  rev::SparkMaxRelativeEncoder m_encoderLiftXD,
                  rev::SparkMaxRelativeEncoder m_encoderrightShooter,
                  rev::SparkMaxRelativeEncoder m_encoderleftShooter)
  {
    T_RobotCorner L_Index;

    for (L_Index = E_FrontLeft;
         L_Index < E_RobotCornerSz;
         L_Index = T_RobotCorner(int(L_Index) + 1))
      {
        V_WheelAngleFwd[L_Index] = 0;
        V_Rad_WheelAngleFwd[L_Index] = 0;
        V_WheelVelocity[L_Index] = 0;
        V_M_WheelDeltaDistance[L_Index] = 0;
        V_Cnt_WheelDeltaDistanceCurr[L_Index] = 0;
        V_Cnt_WheelDeltaDistancePrev[L_Index] = 0;
      }

    m_encoderFrontRightSteer.SetPosition(0);
    m_encoderFrontLeftSteer.SetPosition(0);
    m_encoderRearRightSteer.SetPosition(0);
    m_encoderRearLeftSteer.SetPosition(0);

    m_encoderFrontRightDrive.SetPosition(0);
    m_encoderFrontLeftDrive.SetPosition(0);
    m_encoderRearRightDrive.SetPosition(0);
    m_encoderRearLeftDrive.SetPosition(0);

    m_encoderLiftYD.SetPosition(0);
    m_encoderLiftXD.SetPosition(0);

    m_encoderrightShooter.SetPosition(0);
    m_encoderleftShooter.SetPosition(0);

    V_ShooterSpeedCurr = 0;
  }

/******************************************************************************
 * Function:     Read_Encoders
 *
 * Description:  Run all of the encoder decoding logic.
 ******************************************************************************/
void Read_Encoders(double                       L_encoderWheelAngleFrontLeftRaw,
                   double                       L_encoderWheelAngleFrontRightRaw,
                   double                       L_encoderWheelAngleRearLeftRaw,
                   double                       L_encoderWheelAngleRearRightRaw,
                   rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                   rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                   rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                   rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                   rev::SparkMaxRelativeEncoder m_encoderrightShooter,
                   rev::SparkMaxRelativeEncoder m_encoderleftShooter,
                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                   rev::SparkMaxRelativeEncoder m_encoderLiftXD)
  {
  T_RobotCorner index;

  V_LiftPostitionYD = m_encoderLiftYD.GetPosition();
  V_LiftPostitionXD = m_encoderLiftXD.GetPosition();

  V_WheelAngleConverted[E_FrontLeft]  = std::fmod((L_encoderWheelAngleFrontLeftRaw  * C_EncoderToAngle), 360) - K_WheelOffsetAngle[E_FrontLeft];
  V_WheelAngleConverted[E_FrontRight] = std::fmod((L_encoderWheelAngleFrontRightRaw * C_EncoderToAngle), 360) - K_WheelOffsetAngle[E_FrontRight];
  V_WheelAngleConverted[E_RearLeft]   = std::fmod((L_encoderWheelAngleRearLeftRaw   * C_EncoderToAngle), 360) - K_WheelOffsetAngle[E_RearLeft];
  V_WheelAngleConverted[E_RearRight]  = std::fmod((L_encoderWheelAngleRearRightRaw  * C_EncoderToAngle), 360) - K_WheelOffsetAngle[E_RearRight];

  V_Cnt_WheelDeltaDistanceCurr[E_FrontLeft]  = m_encoderFrontLeftDrive.GetPosition();
  V_Cnt_WheelDeltaDistanceCurr[E_FrontRight] = m_encoderFrontRightDrive.GetPosition();
  V_Cnt_WheelDeltaDistanceCurr[E_RearRight]  = m_encoderRearRightDrive.GetPosition();
  V_Cnt_WheelDeltaDistanceCurr[E_RearLeft]   = m_encoderRearLeftDrive.GetPosition();
  
  for (index = E_FrontLeft;
       index < E_RobotCornerSz;
       index = T_RobotCorner(int(index) + 1))
    {
    V_WheelAngleFwd[index] = V_WheelAngleConverted[index];

    if (V_WheelAngleFwd[index] > 180)
      {
      V_WheelAngleFwd[index] -= 360;
      }
    else if (V_WheelAngleFwd[index] < -180)
      {
      V_WheelAngleFwd[index] += 360;
      }

    /* Now we need to find the equivalent angle as if the wheel were going to be driven in the opposite direction, i.e. in reverse */
    if (V_WheelAngleFwd[index] >= 0)
      {
      V_WheelAngleRev[index] = V_WheelAngleFwd[index] - 180;
      }
    else
      {
      V_WheelAngleRev[index] = V_WheelAngleFwd[index] + 180;
      }
    /* Create a copy of the Angle Fwd, but in radians */
    V_Rad_WheelAngleFwd[index] = V_WheelAngleFwd[index] * (C_PI/180);

    V_M_WheelDeltaDistance[index]  = ((((V_Cnt_WheelDeltaDistanceCurr[index]  - V_Cnt_WheelDeltaDistancePrev[index])/  K_ReductionRatio)) * K_WheelCircufrence );
    V_Cnt_WheelDeltaDistancePrev[index]  = V_Cnt_WheelDeltaDistanceCurr[index];
    }

  V_WheelVelocity[E_FrontLeft]  = ((m_encoderFrontLeftDrive.GetVelocity()  / K_ReductionRatio) / 60) * K_WheelCircufrence;
  V_WheelVelocity[E_FrontRight] = ((m_encoderFrontRightDrive.GetVelocity() / K_ReductionRatio) / 60) * K_WheelCircufrence;
  V_WheelVelocity[E_RearRight]  = ((m_encoderRearRightDrive.GetVelocity()  / K_ReductionRatio) / 60) * K_WheelCircufrence;
  V_WheelVelocity[E_RearLeft]   = ((m_encoderRearLeftDrive.GetVelocity()   / K_ReductionRatio) / 60) * K_WheelCircufrence;

  V_ShooterSpeedCurr = m_encoderleftShooter.GetVelocity();
  }

