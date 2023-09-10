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


} 



namespace OIConstants {//Controller buttons 

constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;

//joystick controller

//axis
namespace Joystick{
    //Axis
constexpr int XAxis = 0; 
constexpr int YAxis = 1; 
constexpr int RotAxis = 2; 
constexpr int ThrottleSlider = 3;

    //Buttons
constexpr int Trigger = 1; 
constexpr int ButtonThree = 3;
}


//buttons



//console controller

//axis

//buttons


}

namespace SwerveModuleConstants {//per swerve module
    constexpr double ktP = 0.0; // Turning PID
    constexpr double ktI = 0.0;
    constexpr double ktD = 0.0;
    constexpr double ktFF = 0.0;
    
    constexpr double kdP = 0.0; // Driving Speed PID
    constexpr double kdI = 0.0;
    constexpr double kdD = 0.0;
    constexpr double kdFF = 0.0;

    constexpr auto maxSpeed = 4.0_mps; // arbitrary values
    constexpr auto maxRotation = 2.0_rad_per_s;
    constexpr double driveRatio = 1; //SDS Mk4 L1
    constexpr double steerRatio = 1; //SDS Mk4 L1

    namespace drivebase{
        constexpr units::meter_t WheelBase = 0.6096_m; // for kinematics
        constexpr units::meter_t TrackWidth = 0.5588_m;
    }
    
    
    namespace topleft{
        constexpr int driveMotor = 2; //CAN Port/NEO ID
        constexpr int turningMotor = 9; //CAN Port/NEO ID
        constexpr int absencoder = 3; //PWM Port

        constexpr double offset = 0;


    }
    namespace topright{
        constexpr int driveMotor = 5; //CAN Port/NEO ID
        constexpr int turningMotor = 6; //CAN Port/NEO ID
        constexpr int absencoder = 0; //PWM Port

        constexpr double offset = 0;
        
    }
    namespace bottomleft{
        constexpr int driveMotor = 3; //CAN Port/NEO ID
        constexpr int turningMotor = 4; //CAN Port/NEO ID
        constexpr int absencoder = 2; //PWM Port

        constexpr double offset = 0;
        
    }
    namespace bottomright{
        constexpr int driveMotor = 8; //CAN Port/NEO ID
        constexpr int turningMotor = 7; //CAN Port/NEO ID
        constexpr int absencoder = 1; //PWM Port

        constexpr double offset = 0;
        
    }

}

namespace ArmConstants{
    namespace shoulderConstants{

        constexpr double kP = 0.0; // PID
        constexpr double kI = 0.0;
        constexpr double kD = 0.0;
        constexpr double kFF = 0.0;
    }

    namespace elbowConstants{
        constexpr double kS = 0.0; // Feedforward
        constexpr double kG = 0.0;
        constexpr double kV = 0.0;
        constexpr double kA = 0.0;

        constexpr double kP = 0.0; // PID
        constexpr double kI = 0.0;
        constexpr double kD = 0.0;
        constexpr double kFF = 0.0;
    }
}


