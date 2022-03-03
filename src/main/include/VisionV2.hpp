/*
  VisionV2.hpp

   Created on: Feb 23, 2022
   Author: Carson

   Contains content from vision.
 */

extern int    V_VisionBottomIndex;
extern bool   V_VisionTargetAquired[E_CamLocSz];
extern double V_VisionYaw[E_CamLocSz];
extern double V_VisionTargetDistanceMeters[E_CamLocSz];

void VisionRobotInit();

void VisionInit(frc::DriverStation::Alliance L_AllianceColor);

void VisionRun(photonlib::PhotonPipelineResult pc_L_TopResult,
               photonlib::PhotonPipelineResult pc_L_BottomResult);
