/*
  VisionV2.cpp

  Created on: Feb 2022
  Author: Carson

  Changes:
  
 */

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
// #include <math.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include <units/length.h>
#include <units/angle.h>

// constants for top target cam
const units::meter_t CAMERA_HEIGHT1 = 0.725_m; // 725 mm to camera lense
const units::meter_t TARGET_HEIGHT1 = 2.58_m; // bottom of tape to carpet 
const units::radian_t CAMERA_PITCH1 = 45_deg; // camera on a 45 degree tilt

// constants for bottom ball cam
const units::meter_t CAMERA_HEIGHT2 = 0.367_m;
const units::meter_t TARGET_HEIGHT2 = .12_m; // radius of the ball in cm
const units::radian_t CAMERA_PITCH2 = 50_deg;

// all our favorite variables
bool TopTargetAquired;
double TopYaw;
double V_TopTargetDistanceMeters; // Distance from front of robot to top target
bool BottomTargetAquired;
double BottomYaw;
int BottomIndex;
int V_VisionBottomIndex;


/******************************************************************************
 * Function:     VisionDashboard
 *
 * Description:  Initialize vision dashboard.
 ******************************************************************************/
void VisionDashboard()
  {
// puts all our favorite variables to the dashboard, all this stuff happens once on init
    frc::SmartDashboard::PutBoolean("Top Target?", TopTargetAquired);
    frc::SmartDashboard::PutNumber("Target Yaw", TopYaw);
    frc::SmartDashboard::PutNumber("Top Distance", V_TopTargetDistanceMeters);

    frc::SmartDashboard::PutBoolean("Bottom Target?", BottomTargetAquired);
    frc::SmartDashboard::PutNumber("Bottom Yaw", BottomYaw);
    frc::SmartDashboard::PutNumber("Bottom Index", BottomIndex);
  }


/******************************************************************************
 * Function:     VisionInit
 *
 * Description:  Initialize vision and related variables.  Only call on robot
 *               init, no need to run this multiple times.
 ******************************************************************************/
void VisionInit()
  {
//   Cam1.StartClientTeam(5561);
//   Cam2.StartClientTeam(5561);
//   photonlib::PhotonCamera Cam1{"Top"};
//   photonlib::PhotonCamera Cam2{"Bottom"};
  }


/******************************************************************************
 * Function:     VisionRun
 *
 * Description:  Contains the necessary code relative to processing the 
 *               vision output.
 ******************************************************************************/
void VisionRun(frc::DriverStation::Alliance    L_AllianceColor,
               photonlib::PhotonPipelineResult pc_L_TopResult,
               photonlib::PhotonPipelineResult pc_L_BottomResult,
               int                            *L_BottomInex)
  {
    TopTargetAquired = pc_L_TopResult.HasTargets(); //returns true if the camera has a target

    photonlib::PhotonTrackedTarget targetTop = pc_L_TopResult.GetBestTarget(); //gets the best target

    TopYaw = targetTop.GetYaw(); // Yaw of the best target
    
    units::meter_t TopRange = photonlib::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT1, TARGET_HEIGHT1, CAMERA_PITCH1,
          units::degree_t{pc_L_TopResult.GetBestTarget().GetPitch()}); // first 3 variables are constants from vision.hpp

    V_TopTargetDistanceMeters = TopRange.value();

    // second camera for cargo detection
    // photonlib::PhotonCamera Cam2{"Bottom"};
    // photonlib::PhotonPipelineResult resultBottom = pc_L_Camera2.GetLatestResult();

    photonlib::PhotonTrackedTarget targetBottom = pc_L_BottomResult.GetBestTarget();
    BottomTargetAquired = pc_L_BottomResult.HasTargets();
    BottomYaw = targetBottom.GetYaw();

    // gets flag from the driver station to choose between alliance colors
    if (L_AllianceColor == frc::DriverStation::Alliance::kRed){
      *L_BottomInex = 1; // 1 is the index for a red ball
    }
    else if (L_AllianceColor == frc::DriverStation::Alliance::kBlue)
    {
      *L_BottomInex = 2; // 2 is the index for a blue ball
    }
    
    units::meter_t BottomRange = photonlib::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT2, TARGET_HEIGHT2, CAMERA_PITCH2,
          units::degree_t{pc_L_BottomResult.GetBestTarget().GetPitch()});

    double BottomRangeDouble = BottomRange.value();

     // set the pipeline to whatever the logic gave

    frc::SmartDashboard::PutBoolean("Top Target?", TopTargetAquired); //puts those new values to dashboard
    frc::SmartDashboard::PutNumber("Top Yaw", TopYaw);
    frc::SmartDashboard::PutNumber("Top Distance", V_TopTargetDistanceMeters); // Probably can remove this...
    frc::SmartDashboard::PutNumber("Bottom Range", BottomRangeDouble);
    frc::SmartDashboard::PutBoolean("Bottom Target?", BottomTargetAquired);
    frc::SmartDashboard::PutNumber("Bottom Yaw", BottomYaw);
    frc::SmartDashboard::PutNumber("Bottom Index", BottomIndex); 
  }