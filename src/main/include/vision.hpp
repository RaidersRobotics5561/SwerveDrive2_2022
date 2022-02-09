#include <networktables/NetworkTableInstance.h>
#include "networktables/NetworkTableEntry.h"
#include <photonlib/PhotonCamera.h>
#include <units/angle.h>
#include <units/length.h>

// // extern void visionInit(std::shared_ptr<NetworkTable> ntTable0,
// //                 std::shared_ptr<NetworkTable> ntTable1,
// //                 nt::NetworkTableInstance inst);
// // extern void visionOff(std::shared_ptr<NetworkTable> ntTable0,
// //                std::shared_ptr<NetworkTable> ntTable1,
// //                nt::NetworkTableInstance inst,
// //                 bool ntStart1,
// //                 bool ntStart2,
// //                 bool ntVisionAngle,
// //                 bool ntVisionDistance);
// extern void visionRun(nt::NetworkTableEntry ntEntry,
//               double ntDistance,
//               int targetChoose,
//               bool ntVisionAngle,
//               bool ntVisionDistance,
//               double ntDesiredAngle,
//               double ntDesiredDistance);
// extern double AutoTarget(double ntEntry);
// extern double AutoShoot(nt::NetworkTableEntry targetYaw, 
//                         double targetDistance, 
//                         int TOPorBOTTOM);
// extern bool AutoMove(double ntDistance, 
//                      double distanceTOTAL);

//old chameleon vision stuff

// constants for top target cam
const units::meter_t CAMERA_HEIGHT1 = 0.725_m; // 725 mm to camera lense
const units::meter_t TARGET_HEIGHT1 = 2.58_m; // bottom of tape to carpet 
const units::radian_t CAMERA_PITCH1 = 45_deg; // camera on a 45 degree tilt

// constants for bottom ball cam
// const units::meter_t CAMERA_HEIGHT1 = 
const units::meter_t TARGET_HEIGHT1 = .12_m; // radius of the ball in cm
// const units::radian_t CAMERA_PITCH1 = 