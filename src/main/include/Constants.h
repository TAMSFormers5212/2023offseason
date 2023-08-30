// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>
#include <string>
#include <array>

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
namespace Joystick{
constexpr int XAxis = 0;
constexpr int YAxis = 1;
constexpr int RotAxis = 2;
}


//buttons



//console controller

//axis

//buttons


}

namespace SwerveModuleConstants {//per swerve module
    constexpr double ktP = 0.0;
    constexpr double ktI = 0.0;
    constexpr double ktD = 0.0;
    constexpr double ktFF = 0.0;
    
    constexpr double kdP = 0.0;
    constexpr double kdI = 0.0;
    constexpr double kdD = 0.0;
    constexpr double kdFF = 0.0;

    constexpr auto maxSpeed = 4.0_mps; // arbitrary values
    constexpr auto maxRotation = 2.0_rad_per_s;
    constexpr double driveRatio = 1; //SDS Mk4 L1
    constexpr double steerRatio = 1; //SDS Mk4 L1

    namespace drivebase{
        constexpr units::meter_t WheelBase = 0.6096_m;
        constexpr units::meter_t TrackWidth = 0.5588_m;
    }
    
    
    namespace topleft{
        constexpr int driveMotor = 1;
        constexpr int turningMotor = 2;
        constexpr int absencoder = 1;

        constexpr double offset = 0;


    }
    namespace topright{
        constexpr int driveMotor = 3;
        constexpr int turningMotor = 4;
        constexpr int absencoder = 2;

        constexpr double offset = 0;
        
    }
    namespace bottomleft{
        constexpr int driveMotor = 5;
        constexpr int turningMotor = 6;
        constexpr int absencoder = 3;

        constexpr double offset = 0;
        
    }
    namespace bottomright{
        constexpr int driveMotor = 7;
        constexpr int turningMotor = 8;
        constexpr int absencoder = 4;

        constexpr double offset = 0;
        
    }

}

namespace PoseConstants{
    
}
