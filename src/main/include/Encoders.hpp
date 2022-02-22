/*
  Encoders.hpp

  Created on: Feb 25, 2020

  Author: 5561

  Updates:
  2022-02-15: Cleaned up file
*/

#include "Enums.hpp"

 extern double V_WheelAngleFwd[E_RobotCornerSz];
 extern double V_Rad_WheelAngleFwd[E_RobotCornerSz]; 
 extern double V_WheelAngleRev[E_RobotCornerSz];
 extern double V_WheelVelocity[E_RobotCornerSz];
 extern double V_M_WheelDeltaDistance[E_RobotCornerSz];
 extern double V_WheelAngleConverted[E_RobotCornerSz];
 extern double V_ShooterSpeedCurr;
 extern double V_LiftPostitionYD; // Position of the YD lift, in revolutions of the motor
 extern double V_LiftPostitionXD; // Position of the XD lift, in revolutions of the motor
 
 void Read_Encoders(double          L_encoderWheelAngleFrontLeftRaw,
                  double          L_encoderWheelAngleFrontRightRaw,
                  double          L_encoderWheelAngleRearLeftRaw,
                  double          L_encoderWheelAngleRearRightRaw,
                   rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                   rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                   rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                   rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                   rev::SparkMaxRelativeEncoder m_encoderrightShooter,
                   rev::SparkMaxRelativeEncoder m_encoderleftShooter,
                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                   rev::SparkMaxRelativeEncoder m_encoderLiftXD);

double DtrmnEncoderRelativeToCmnd(double L_JoystickCmnd,
                                  double L_EncoderReading);

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
                  rev::SparkMaxRelativeEncoder m_encoderleftShooter);