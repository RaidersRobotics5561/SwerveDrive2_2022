/*
  VisionV2.hpp

   Created on: Feb 23, 2022
   Author: Biggs

   Contains content from vision.
 */

extern int    V_VisionBottomIndex;
extern bool   TopTargetAquired;
extern double TopYaw;
extern double V_TopTargetDistanceMeters;

void VisionRun(frc::DriverStation::Alliance    L_AllianceColor,
               photonlib::PhotonPipelineResult pc_L_TopResult,
               photonlib::PhotonPipelineResult pc_L_BottomResult,
               int                            *L_BottomInex);

void VisionDashboard();