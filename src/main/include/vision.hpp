#include <networktables/NetworkTableInstance.h>
#include "networktables/NetworkTableEntry.h"
#include <photonlib/PhotonCamera.h>
#include <units/angle.h>
#include <units/length.h>


void VisionRun();
void VisionDashboard();

extern bool   TopTargetAquired;
extern double TopYaw;
extern double V_TopTargetDistanceMeters;


// constants for top target cam
const units::meter_t CAMERA_HEIGHT1 = 0.725_m; // 725 mm to camera lense
const units::meter_t TARGET_HEIGHT1 = 2.58_m; // bottom of tape to carpet 
const units::radian_t CAMERA_PITCH1 = 45_deg; // camera on a 45 degree tilt

// constants for bottom ball cam
const units::meter_t CAMERA_HEIGHT2 = 0.367_m;
const units::meter_t TARGET_HEIGHT2 = .12_m; // radius of the ball in cm
const units::radian_t CAMERA_PITCH2 = 50_deg;