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

constexpr int kMXP = 2;


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
    constexpr double driveRatio = 1; //SDS Mk4 L1
    constexpr double steerRatio = 1; //SDS Mk4 L1
    
    
    namespace topleft{
        constexpr int driveMotor = 1;
        constexpr int turningMotor = 2;


        constexpr double offset = 0;


    }
    namespace topright{
        constexpr int driveMotor = 3;
        constexpr int turningMotor = 4;

        constexpr double offset = 0;
        
    }
    namespace bottomleft{
        constexpr int driveMotor = 5;
        constexpr int turningMotor = 6;
        constexpr double offset = 0;
        
    }
    namespace bottomright{
        constexpr int driveMotor = 7;
        constexpr int turningMotor = 8;
        constexpr double offset = 0;
        
    }

}

namespace PoseConstants{
    
}
