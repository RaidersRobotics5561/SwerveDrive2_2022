#include "Enums.hpp"

void Read_Encoders(bool L_RobotInit,
                   double          L_encoderWheelAngleFrontLeftRaw,
                   double          L_encoderWheelAngleFrontRightRaw,
                   double          L_encoderWheelAngleRearLeftRaw,
                   double          L_encoderWheelAngleRearRightRaw,
                   rev::SparkMaxRelativeEncoder m_encoderFrontLeftSteer,
                   rev::SparkMaxRelativeEncoder m_encoderFrontRightSteer,
                   rev::SparkMaxRelativeEncoder m_encoderRearLeftSteer,
                   rev::SparkMaxRelativeEncoder m_encoderRearRightSteer,
                   rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                   rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                   rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                   rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                   rev::SparkMaxRelativeEncoder m_encoderrightShooter,
                   rev::SparkMaxRelativeEncoder m_encoderleftShooter);

double DtrmnEncoderRelativeToCmnd(double L_JoystickCmnd,
                                  double L_EncoderReading);

 extern double V_WheelAngleFwd[E_RobotCornerSz];
 extern double V_Rad_WheelAngleFwd[E_RobotCornerSz]; 
 extern double V_WheelAngleRev[E_RobotCornerSz];
 extern double V_WheelAngleArb[E_RobotCornerSz];
 extern double V_WheelRelativeAngleRawOffset[E_RobotCornerSz];
 extern double V_WheelVelocity[E_RobotCornerSz];
 extern double V_WheelAnglePrev[E_RobotCornerSz];
 extern double V_WheelAngleLoop[E_RobotCornerSz];
 extern double V_WheelAngleRaw[E_RobotCornerSz];
 extern double V_ShooterSpeedCurr[E_RoboShooter];
 extern double V_WheelDeltaDistance[E_RobotCornerSz];
 extern double V_M_WheelDeltaDistance[E_RobotCornerSz];
 extern double V_Cnt_WheelDeltaDistanceCurr[E_RobotCornerSz];
 extern double V_Cnt_WheelDeltaDistancePrev[E_RobotCornerSz];
 