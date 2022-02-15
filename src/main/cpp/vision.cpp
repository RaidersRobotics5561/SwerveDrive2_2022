#include "vision.hpp"
#include <math.h>
/******************************************************************************
 * Function:     visionInit
 *
 * Description:  This function sets up camera and lights.
 ******************************************************************************/
void visionInit(std::shared_ptr<nt::NetworkTable> ntTable0,
                std::shared_ptr<nt::NetworkTable> ntTable1,
                nt::NetworkTableInstance inst)
{
    ntTable1->PutNumber("ledControl", 1);
    ntTable0->PutBoolean("driverMode", false);
    inst.Flush();
}

/******************************************************************************
 * Function:     visionOff
 *
 * Description:  This function turns on driver mode and turns off lights.
 ******************************************************************************/
void visionOff(std::shared_ptr<nt::NetworkTable> ntTable0,
                std::shared_ptr<nt::NetworkTable> ntTable1,
                nt::NetworkTableInstance inst,
                bool ntStart1,
                bool ntStart2,
                bool ntVisionAngle,
                bool ntVisionDistance)
{
    ntTable1->PutNumber("ledControl", 5);
    ntTable0->PutBoolean("driverMode", true);
    inst.Flush();
    ntStart1 = false; ntStart2 = false;
    ntVisionAngle = false;
    ntVisionDistance = false;
}
    
/******************************************************************************
 * Function:     visionRun
 *
 * Description:  This function toggles vision loop.
 ******************************************************************************/
void visionRun(nt::NetworkTableEntry ntEntry,
              double ntDistance,
              int targetChoose,
              bool ntVisionAngle,
              bool ntVisionDistance,
              double ntDesiredAngle,
              double ntDesiredDistance)
{
    int autonChoose;
    
      switch (targetChoose)
      {
          case 0:
            if (abs(ntEntry.GetDouble(0)) > 1)
            {
                autonChoose = 1;
            }
            if (abs(ntEntry.GetDouble(0)) < 1)
            {
                autonChoose = 2;
            }
            break;

          case 1:
            if (abs(ntEntry.GetDouble(0)) > 5)
            {
                autonChoose = 0;
            }
            else if (abs(ntEntry.GetDouble(0)) > 1)
            {
                ntVisionDistance = false;
                autonChoose = 1;
            }
            if (abs(ntEntry.GetDouble(0)) < 1)
            {
                autonChoose = 2;
            }
            break;
      }
      
      switch (autonChoose)
      {
          case 0:
            if (ntVisionDistance == false)
            {
                ntDesiredDistance = floor(ntDistance);
                ntVisionDistance  = true;
            }
            // if (visionRequest == true)
            // {
            //     visionRequest = false;
            //     activeVisionDistance0 = false;
            // }
            break;
          
          case 1:
            if (ntVisionAngle == false)
            {
                ntDesiredAngle = (0.9 * ntEntry.GetDouble(0));
                ntVisionAngle  = true;
            }
            // if (visionRequest == true)
            // {
            //     visionRequest = false;
            //     activeVisionDistance0 = false;
            // }
            break;
          
          case 2:
            break;
      }

//ToDo: Biggs - Not sure if this is still needed??  This was in Robot.cpp
    // if(c_joyStick2.GetPOV() == 90 && V_pipelinecounterLatch == false)
    // { 
    //   pipelineCounter++;
    //   int pipelineChecker = (pipelineCounter % 2);
    //   if(pipelineChecker != 0)
    //   {
    //     vision0->PutNumber("pipeline", 1);
    //     inst.Flush();
    //   }
    //   else
    //   {
    //     vision0->PutNumber("pipeline", 0);
    //     inst.Flush();
    //   }
    //   V_pipelinecounterLatch = true;
    // }
    // else if(c_joyStick2.GetPOV() != 90)
    // {
    //   V_pipelinecounterLatch = false;
    // }
    
    // frc::SmartDashboard::PutNumber("pipeline", pipeline0.GetDouble(0));

        /*
      Finds distance from robot to specified target.
      Numerator depends upon camera height relative to target for target distance,
      and camera height relative to ground for ball distance.
      Make sure it's in meters.
    */
//    if(pipeline0.GetDouble(0) == 1)
//    {
//      distanceTarget     = 124.8 / tan((targetPitch0.GetDouble(0) + 15) * (C_Deg2Rad));
//    }
//    else
//    {
//      distanceTarget     = 157.8 / tan((targetPitch0.GetDouble(0) + 15) * (C_Deg2Rad));
//    }
   
//     //  distanceBall       = 47  / tan((targetPitch1.GetDouble(0)) * (-deg2rad));

//     //Finds robot's distance from target's center view.
//      distanceFromTargetCenter = (distanceTarget * sin((90 - targetYaw0.GetDouble(0)) * C_Deg2Rad) - 28.17812754);
//     //  distanceFromBallCenter   = distanceBall   * sin((90 - targetYaw1.GetDouble(0)) * deg2rad);

//     // frc::SmartDashboard::PutBoolean("testboolean", testboolean);
//     frc::SmartDashboard::PutNumber("distanceTarget", distanceTarget);
//     // frc::SmartDashboard::PutNumber("distanceFromTargetCenter", distanceFromTargetCenter);
//     frc::SmartDashboard::PutNumber("targetYaw", targetYaw0.GetDouble(0));
//     // frc::SmartDashboard::PutNumber("targetPitch", targetPitch0.GetDouble(1));
//     // frc::SmartDashboard::PutNumber("lidarDistance", lidarDistance.GetDouble(0));
}
