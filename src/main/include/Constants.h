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

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;

//LEDS?
//vision or other accessories

}  // namespace OperatorConstants



namespace OIConstants {//Controller buttons 

//joystick controller

//axis


//buttons


//console controller

//axis

//buttons


}

namespace SwerveModuleConstants {//per swerve module
    constexpr double ktP;
    constexpr double ktI;
    constexpr double KtD;
    
    constexpr double kdP;
    constexpr double kdI;
    constexpr double KdD;

    constexpr double maxSpeed;
    constexpr double driveRatio;
    constexpr double steerRatio;
    
    
    namespace topleft{
        double offset;


    }
    namespace topright{
        
    }
    namespace bottomleft{
        
    }
    namespace bottomright{
        
    }

}

namespace PoseConstants{
    
}
