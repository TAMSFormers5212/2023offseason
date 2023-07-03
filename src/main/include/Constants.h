// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace ElectricalConstants {


//LEDS?
//vision or other accessories

}  // namespace OperatorConstants



namespace OIConstants {//Controller buttons 

constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;

//joystick controller

//axis


//buttons


//console controller

//axis

//buttons


}

namespace SwerveModuleConstants {//per swerve module
    constexpr double ktP = 0.0;
    constexpr double ktI = 0.0;
    constexpr double KtD = 0.0;
    
    constexpr double kdP = 0.0;
    constexpr double kdI = 0.0;
    constexpr double KdD = 0.0;

    constexpr double maxSpeed = 4.0;
    constexpr double driveRatio; //SDS Mk4 L1
    constexpr double steerRatio; //SDS Mk4 L1
    
    
    namespace topleft{
        double offset;


    }
    namespace topright{
        double offset;
        
    }
    namespace bottomleft{
        double offset;
        
    }
    namespace bottomright{
        double offset;
        
    }

}

namespace PoseConstants{
    
}
