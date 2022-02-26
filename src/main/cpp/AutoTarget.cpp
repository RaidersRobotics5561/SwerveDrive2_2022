/*
  AutoTarget.cpp

  Created on: Feb 25, 2020
  Author: 5561
 */
#if 0
#include <math.h>

#include "Lookup.hpp"
#include "Const.hpp"

/******************************************************************************
 * Function:     AutoTargeting
 *
 * Description:  Auto targeting function.
 ******************************************************************************/
T_AutoTargetStates AutoTargeting(T_AutoTargetStates  L_CurrentState,
                                 bool                L_ActivateTarget, // two booleans, one for targeting motion
                                 bool                L_ActivateRollers, //another for rollers
                                 double              L_DriverAxis1, //need driver axis for the abort
                                 double              L_DriverAxis2,
                                 double              L_DriverAxis3,
                                 double              L_RawTargetVisionAngle,
                                 double              L_RawTargetVisionDistance,
                                 double              L_RobotAngle,
                                 double             *L_RobotTargetAngle,
                                 double              L_RollerSpeed,
                                 double             *L_RollerSpeedReq,
                                 double             *L_BeltPowerReq)
  {
//  double L_RobotTargetAngle = 0;
  double L_RollerSpeedError = 0;
  double L_RobotAngleError  = 0;
  bool   L_AbortTargeting   = false;



  if ((fabs(L_DriverAxis1) > 0) ||
      (fabs(L_DriverAxis2) > 0) ||
      (fabs(L_DriverAxis3) > 0))
    {
    L_AbortTargeting = true;
    }




  L_RobotAngleError       = fabs(L_RobotAngle       - *L_RobotTargetAngle);


  /* First, we need to locate the target.  In order to reliably do this, we need the following criteria met:
   * - Gyro says robot is pointing straight ahead
   * - Vision system is providing a distance
   * - Vision system is providing an angle
   *  */



  if (L_AbortTargeting == true)
    {
    L_CurrentState = E_NotActive;
    *L_RobotTargetAngle = 0.0;
    *L_RollerSpeedReq = 0.0;
    *L_BeltPowerReq = 0.0;
    }



  else if ((L_ActivateTarget == true) &&
           (L_RawTargetVisionAngle > K_TargetVisionAngleMin) &&
           (L_RawTargetVisionAngle < K_TargetVisionAngleMax) &&
           (L_RawTargetVisionDistance > K_TargetVisionDistanceMin) &&
           (L_RawTargetVisionDistance < K_TargetVisionDistanceMax) &&
           (L_CurrentState == E_NotActive))
    {
    /* Ok, we seem to be able to see the target and it is being requested that we become active.
     * We are also not currently active.
     * (or in carson words: checked that the targetting has been activated, the program is not already 
     * active, and our angle are within parameters)
      */ 
  
  

    *L_RobotTargetAngle  = K_TargetVisionAngleUpper;
    *L_BeltPowerReq = 0.0;

    DesiredRollerSpeed(L_RawTargetVisionDistance,
                       L_RawTargetVisionAngle,
                       L_RollerSpeedReq,
                       L_RollerSpeedReq);
    }

  L_CurrentState = E_RollerSpinUp; // mark we have spun up rollers
  }

  else if ((L_RobotAngleError <= K_TargetVisionAngleErrorMax) &&
           (L_RollerSpeedError <= K_TargetVisionUpperRollerErrorMax) && //this is not finished please use real error maxs and mins
           (L_RollerSpeedError <= K_TargetVisionLowerRollerErrorMax) &&
           (L_CurrentState > E_NotActive) &&
           (L_CurrentState < E_MoveBallsToRollers))
    {
    L_CurrentState = E_MoveBallsToRollers; // mark we are now moving our balls up
    *L_BeltPowerReq = 0.9;
    }

  return (L_CurrentState);
  }
#endif