#ifdef VISION1
#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
// #include <frc/livewindow/LiveWindow.h>
#include <networktables/NetworkTableInstance.h>
#include "networktables/NetworkTableEntry.h"

#include <math.h>
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
frc::DriverStation::Alliance AllianceColor;
// our favorite networktables
nt::NetworkTableInstance Cam1;
nt::NetworkTableInstance Cam2;




void VisionDashboard(){

// puts all our favorite variables to the dashboard, all this stuff happens once on init
    frc::SmartDashboard::PutBoolean("Top Target?", TopTargetAquired);
    frc::SmartDashboard::PutNumber("Target Yaw", TopYaw);
    frc::SmartDashboard::PutNumber("Top Distance", V_TopTargetDistanceMeters);

    frc::SmartDashboard::PutBoolean("Bottom Target?", BottomTargetAquired);
    frc::SmartDashboard::PutNumber("Bottom Yaw", BottomYaw);
    frc::SmartDashboard::PutNumber("Bottom Index", BottomIndex);



}

//actual vision things:
void VisionRun(){

    Cam2.StartClientTeam(5561);
    Cam1.StartClientTeam(5561);

    // first Camera is cam1 for auto target
    photonlib::PhotonCamera Cam1{"Top"};
    photonlib::PhotonPipelineResult resultTop = Cam1.GetLatestResult();
  
    TopTargetAquired = resultTop.HasTargets(); //returns true if the camera has a target

    photonlib::PhotonTrackedTarget targetTop = resultTop.GetBestTarget(); //gets the best target

    TopYaw = targetTop.GetYaw(); // Yaw of the best target

    frc::SmartDashboard::PutBoolean("Top Target?", TopTargetAquired); //puts those new values to dashboard
    frc::SmartDashboard::PutNumber("Top Yaw", TopYaw);
    
    units::meter_t TopRange = photonlib::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT1, TARGET_HEIGHT1, CAMERA_PITCH1,
          units::degree_t{resultTop.GetBestTarget().GetPitch()}); // first 3 variables are constants from vision.hpp

    double TopRangeDouble = TopRange.value();

    V_TopTargetDistanceMeters = TopRangeDouble;

    frc::SmartDashboard::PutNumber("Top Range", TopRangeDouble);
    frc::SmartDashboard::PutNumber("Top Distance", TopRangeDouble); // Probably can remove this...


    // second camera for cargo detection
    photonlib::PhotonCamera Cam2{"Bottom"};
    photonlib::PhotonPipelineResult resultBottom = Cam2.GetLatestResult();

    photonlib::PhotonTrackedTarget targetBottom = resultBottom.GetBestTarget();
    BottomTargetAquired = resultBottom.HasTargets();
    BottomYaw = targetBottom.GetYaw();

    AllianceColor = frc::DriverStation::GetInstance().GetAlliance();

    // gets flag from the driver station to choose between alliance colors
    if (AllianceColor == frc::DriverStation::Alliance::kRed){
      BottomIndex = 1; // 1 is the index for a red ball

    }
    else if (AllianceColor == frc::DriverStation::Alliance::kBlue)
    {
      BottomIndex = 2; // 2 is the index for a blue ball
    }
    
    units::meter_t BottomRange = photonlib::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT2, TARGET_HEIGHT2, CAMERA_PITCH2,
          units::degree_t{resultBottom.GetBestTarget().GetPitch()});

    double BottomRangeDouble = BottomRange.value();

    frc::SmartDashboard::PutNumber("Bottom Range", BottomRangeDouble);

    Cam2.SetPipelineIndex(BottomIndex);
     // set the pipeline to whatever the logic gave


    frc::SmartDashboard::PutBoolean("Bottom Target?", BottomTargetAquired);
    frc::SmartDashboard::PutNumber("Bottom Yaw", BottomYaw);
    frc::SmartDashboard::PutNumber("Bottom Index", BottomIndex); 


    
}
#endif