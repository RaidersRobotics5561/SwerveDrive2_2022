/*
  VisionV2.cpp

  Created on: Feb 2022
  Author: Carson

  Changes:
  
 */

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include "Const.hpp"
#ifdef VISION2

// all our favorite variables
double         V_VisionTopCamNumberTemp = 1;
int            V_VisionBottomIndex = 0;
T_CameraNumber V_VisionCamNumber[E_CamLocSz];

bool           V_VisionTargetAquired[E_CamLocSz];
double         V_VisionYaw[E_CamLocSz];
double         V_VisionTargetDistanceMeters[E_CamLocSz];

/******************************************************************************
 * Function:     VisionRobotInit
 *
 * Description:  Initializes the camera selector at robot init.
 ******************************************************************************/
void VisionRobotInit()
  {
  /* Place an input on the dash.  A value of 1 indicates top camera is cam 1, 2 is camera 2 */
  frc::SmartDashboard::PutNumber("Top Camera Number", V_VisionTopCamNumberTemp);
  }


/******************************************************************************
 * Function:     VisionInit
 *
 * Description:  Initialize vision and related variables.
 ******************************************************************************/
void VisionInit(frc::DriverStation::Alliance L_AllianceColor)
  {
  V_VisionTopCamNumberTemp = frc::SmartDashboard::GetNumber("Top Camera Number", V_VisionTopCamNumberTemp);

  if (fabs(V_VisionTopCamNumberTemp) < 1.5)
    {
    V_VisionCamNumber[E_CamTop] = E_Cam1;
    V_VisionCamNumber[E_CamBottom] = E_Cam2;
    }
  else
    {
    V_VisionCamNumber[E_CamTop] = E_Cam2;
    V_VisionCamNumber[E_CamBottom] = E_Cam1;
    }

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
  T_CameraLocation L_Index = E_CamTop;
  units::meter_t L_Range = 0_m;
  photonlib::PhotonTrackedTarget L_Target;
  photonlib::PhotonPipelineResult pc_L_Result[E_CamSz];

  pc_L_Result[E_Cam1] = pc_L_TopResult;
  pc_L_Result[E_Cam2] = pc_L_BottomResult;
  
  for (L_Index = E_CamTop;
       L_Index < E_CamLocSz;
       L_Index = T_CameraLocation(int(L_Index) + 1))
      {
      V_VisionTargetAquired[L_Index] = pc_L_Result[V_VisionCamNumber[L_Index]].HasTargets(); //returns true if the camera has a target  
    
      if (V_VisionTargetAquired[L_Index] == true)
        {
        L_Target = pc_L_Result[V_VisionCamNumber[L_Index]].GetBestTarget(); //gets the best target  
    
        V_VisionYaw[L_Index] = L_Target.GetYaw(); // Yaw of the best target
      
        L_Range = photonlib::PhotonUtils::CalculateDistanceToTarget(
                     K_VisionHeight[L_Index], K_VisionTargetHeight[L_Index], K_VisionCameraPitch[L_Index],
                     units::degree_t{pc_L_Result[V_VisionCamNumber[L_Index]].GetBestTarget().GetPitch()}); // first 3 variables are constants from Const.hpp  
        
        if (L_Range < 0_m)
          {
          L_Range = 0_m;
          }
        V_VisionTargetDistanceMeters[E_CamTop] = L_Range.value();
        }
      }

  // Set the interfaces used in the other places in software:
  }
#endif