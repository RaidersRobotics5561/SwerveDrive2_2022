/*
  VisionV2.hpp

   Created on: Feb 23, 2022
   Author: Carson

   Contains content from vision.
 */

extern bool   V_VisionTopTargetAquired;
extern double V_VisionTopYaw;
extern double V_VisionTopTargetDistanceMeters;
extern double V_VisionBottomTargetDistanceMeters;
extern bool   V_VisionBottomTargetAquired;
extern double V_VisionBottomYaw;
extern int    V_VisionBottomIndex;

void VisionInit(frc::DriverStation::Alliance L_AllianceColor);

void VisionRun(photonlib::PhotonPipelineResult pc_L_TopResult,
               photonlib::PhotonPipelineResult pc_L_BottomResult);

void VisionDashboard();