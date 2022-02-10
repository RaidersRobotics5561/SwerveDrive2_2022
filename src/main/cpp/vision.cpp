#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include "vision.hpp"
#include <math.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include <units/length.h>
// all our favorite variables
bool TargetAquired;
double TopYaw;
bool BottomTargetAquired;
double BottomYaw;
int BottomIndex;
frc::DriverStation::Alliance AllianceColor;
// our favorite networktables
nt::NetworkTableInstance Cam1;
nt::NetworkTableInstance Cam2;

void VisionDashboard(){

// puts all our favorite variables to the dashboard, all this stuff happens once on init
    frc::SmartDashboard::PutBoolean("Has Target?", TargetAquired);
    frc::SmartDashboard::PutNumber("Target Yaw", TopYaw);

    frc::SmartDashboard::PutBoolean("Bottom Has Target?", BottomTargetAquired);
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
  
    TargetAquired = resultTop.HasTargets(); //returns true if the camera has a target

    photonlib::PhotonTrackedTarget targetTop = resultTop.GetBestTarget(); //gets the best target

    TopYaw = targetTop.GetYaw(); // Yaw of the best target

    frc::SmartDashboard::PutBoolean("Top Target?", TargetAquired); //puts those new values to dashboard
    frc::SmartDashboard::PutNumber("Top Yaw", TopYaw);
    
    units::meter_t TopRange = photonlib::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT1, TARGET_HEIGHT1, CAMERA_PITCH1,
          units::degree_t{resultTop.GetBestTarget().GetPitch()}); // first 3 variables are constants from vision.hpp

    double TopRangeDouble = TopRange.value();

      frc::SmartDashboard::PutNumber("Top Range", TopRangeDouble);


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
    

    Cam2.SetPipelineIndex(BottomIndex);
     // set the pipeline to whatever the logic gave


    frc::SmartDashboard::PutBoolean("Bottom Has Target?", BottomTargetAquired);
    frc::SmartDashboard::PutNumber("Bottom Yaw", BottomYaw);
    frc::SmartDashboard::PutNumber("Bottom Index", BottomIndex); 
    

}