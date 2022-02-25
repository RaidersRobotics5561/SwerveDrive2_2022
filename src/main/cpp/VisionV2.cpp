/*
  VisionV2.cpp

  Created on: Feb 2022
  Author: Carson

  Changes:
  
 */

#include <frc/DriverStation.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include "Const.hpp"

// all our favorite variables
bool   V_VisionTopTargetAquired = false;
double V_VisionTopYaw = 0;
double V_VisionTopTargetDistanceMeters = 0; // Distance from front of robot to top target
double V_VisionBottomTargetDistanceMeters = 0;
bool   V_VisionBottomTargetAquired = false;
double V_VisionBottomYaw = 0;
int    V_VisionBottomIndex = 0;


/******************************************************************************
 * Function:     VisionDashboard
 *
 * Description:  Initialize vision dashboard.
 ******************************************************************************/
void VisionDashboard()
  {

  }


/******************************************************************************
 * Function:     VisionInit
 *
 * Description:  Initialize vision and related variables.
 ******************************************************************************/
void VisionInit(frc::DriverStation::Alliance L_AllianceColor)
  {
  // gets flag from the driver station to choose between alliance colors
  if (L_AllianceColor == frc::DriverStation::Alliance::kRed)
    {
    V_VisionBottomIndex = 1; // 1 is the index for a red ball
    }
  else // if (L_AllianceColor == frc::DriverStation::Alliance::kBlue) -> must be either red or blue
    {
    V_VisionBottomIndex = 2; // 2 is the index for a blue ball
    }
  }


/******************************************************************************
 * Function:     VisionRun
 *
 * Description:  Contains the necessary code relative to processing the 
 *               vision output.
 ******************************************************************************/
void VisionRun(photonlib::PhotonPipelineResult pc_L_TopResult,
               photonlib::PhotonPipelineResult pc_L_BottomResult)
  {
  units::meter_t L_TopRange;
  photonlib::PhotonTrackedTarget L_TargetTop;
  photonlib::PhotonTrackedTarget L_TargetBottom;
  units::meter_t L_BottomRange;

  // Camera 1 - Top Target Detection:
  V_VisionTopTargetAquired = pc_L_TopResult.HasTargets(); //returns true if the camera has a target  

  if (V_VisionTopTargetAquired == true){
  L_TargetTop = pc_L_TopResult.GetBestTarget(); //gets the best target  

  V_VisionTopYaw = L_TargetTop.GetYaw(); // Yaw of the best target
  
  L_TopRange = photonlib::PhotonUtils::CalculateDistanceToTarget(
        K_VisionHeight1, K_VisionTargetHeight1, K_VisionCameraPitch1,
        units::degree_t{pc_L_TopResult.GetBestTarget().GetPitch()}); // first 3 variables are constants from Const.hpp  

    V_VisionTopTargetDistanceMeters = L_TopRange.value();
  } 
  
  // second camera for cargo detection
  V_VisionBottomTargetAquired = pc_L_BottomResult.HasTargets();

if (V_VisionTopTargetAquired == true){
  L_TargetBottom = pc_L_BottomResult.GetBestTarget();

  V_VisionBottomYaw = L_TargetBottom.GetYaw();  
  
  L_BottomRange = photonlib::PhotonUtils::CalculateDistanceToTarget(
        K_VisionHeight2, K_VisionTargetHeight2, K_VisionCameraPitch2,
        units::degree_t{pc_L_BottomResult.GetBestTarget().GetPitch()});

  V_VisionBottomTargetDistanceMeters = L_BottomRange.value(); 
  } 

 
  }