/*
  AutoTarget.cpp

  Created on: Feb 25, 2020
  Author: 5561
 */


#include "Robot.h"

#include "control_pid.hpp"
#include "Encoders.hpp"
#include "DriveControl.hpp"
#include "Lookup.hpp"
#include "Enums.hpp"
#include <math.h>


/******************************************************************************
 * Function:     AutoTargeting
 *
 * Description:  Auto targeting function.
 ******************************************************************************/
T_AutoTargetStates AutoTargeting(T_AutoTargetStates  L_CurrentState,
                                 bool                L_ActivateTarget,
                                 bool                L_ActivateRollers,
                                 double              L_DriverAxis1,
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
     * We are also not currently active. */ 
  
  

    *L_RobotTargetAngle  = K_TargetVisionAngle;
    *L_BeltPowerReq = 0.0;

    /* Ok, let's go to the next step: */
    L_CurrentState = E_TargetFoundRotateBot;

  if (L_ActivateRollers == true){
  
  
  L_RollerSpeedError = fabs(L_RollerSpeed - *L_RollerSpeedReq);


    DesiredRollerSpeed(L_RawTargetVisionDistance,
                       L_RawTargetVisionAngle,
                       L_RollerSpeedReq,
                       L_RollerSpeedReq);
    }
  }

  else if ((L_RobotAngleError <= K_TargetVisionAngleErrorMax) &&
           (L_RollerSpeedError <= K_TargetVisionUpperRollerErrorMax) && //this is not finished please use real error maxs and mins
           (L_RollerSpeedError <= K_TargetVisionLowerRollerErrorMax) &&
           (L_CurrentState > E_NotActive) &&
           (L_CurrentState < E_MoveBallsToRollers))
    {
    L_CurrentState = E_MoveBallsToRollers;
    *L_BeltPowerReq = 0.9;
    }

  return (L_CurrentState);
  }
