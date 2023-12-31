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
#include <vector>
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
    namespace Joystick{
        //Axis
        constexpr int XAxis = 0; 
        constexpr int YAxis = 1; 
        constexpr int RotAxis = 2; 
        constexpr int ThrottleSlider = 3;
        //Buttons
        constexpr int Trigger = 1; 
        constexpr int ButtonThree = 3;

        constexpr double deadband = 0.3;
    }

    namespace Controller{//console controller
        //axis
        constexpr int leftXAxis = 0;
        constexpr int leftYAxis = 1;
        constexpr int rightXAxis = 4;
        constexpr int rightYAxis = 5;
        constexpr int leftTrigger = 2;
        constexpr int rightTrigger = 3;
        //buttons
        constexpr int A = 1;
        constexpr int B = 2;
        constexpr int X = 3;
        constexpr int Y = 4;
        constexpr int leftBumper = 5;
        constexpr int rightBumper = 6;
        constexpr int View = 7;
        constexpr int Menu = 8;
        constexpr int LPress = 9;
        constexpr int RPress = 10;
    }



}

namespace SwerveModuleConstants {//per swerve module
    constexpr double ktP = 0.2; // Turning PID
    constexpr double ktI = 0.0;
    constexpr double ktD = 0.0;
    constexpr double ktFF = 0.00001;
    
    constexpr double kdP = 0.6; // Driving Speed PID
    constexpr double kdI = 0.0;
    constexpr double kdD = 0.0;
    constexpr double kdFF = 0.00;

    constexpr auto maxSpeed = 4.0_mps; // arbitrary values
    constexpr auto maxRotation = 2.0_rad_per_s;
    constexpr double driveRatio = 8.14; //SDS Mk4 L1
    constexpr double steerRatio = 12.8; //SDS Mk4 L1
    constexpr units::inch_t wheelDiameter = 4_in;
    constexpr units::inch_t wheelCircumfrence = 12.57_in;
    constexpr units::inch_t centerDistance = 10.5_in;

    constexpr double kRampTimeSeconds = 0.1;

    namespace drivebase{
        constexpr units::meter_t WheelBase = 0.6096_m; // for kinematics
        constexpr units::meter_t TrackWidth = 0.5588_m;
    }
    
    
    namespace topleft{
        constexpr int driveMotor = 9; //CAN Port/NEO ID
        constexpr int turningMotor = 2; //CAN Port/NEO ID
        constexpr int absencoder = 3; //PWM Port

        constexpr double offset =  0;
//2.2009 

    }
    namespace topright{
        constexpr int driveMotor = 5; //CAN Port/NEO ID
        constexpr int turningMotor = 6; //CAN Port/NEO ID
        constexpr int absencoder = 0; //PWM Port

        constexpr double offset = -0.2;//5.04/(2*M_PI);//0.534055-0.828717-0.1;//(149.056526933-120.869343735-5)/360;
        // 4.07
    }
    namespace bottomleft{
        constexpr int driveMotor = 4; //CAN Port/NEO ID
        constexpr int turningMotor = 3; //CAN Port/NEO ID
        constexpr int absencoder = 2; //PWM Port

        constexpr double offset = 0;//(0.38+M_PI)/(2*M_PI);//0.6101363-0.58127+0.1;// (343.800190846-229.081278318-15)/360;
        //7.48
    }
    namespace bottomright{          
        constexpr int driveMotor = 7; //CAN Port/NEO ID
        constexpr int turningMotor = 8; //CAN Port/NEO ID
        constexpr int absencoder = 1; //PWM Port

        constexpr double offset = 0.2;//0.311/(2*M_PI);
        //2.253189
    }

}

namespace ArmConstants{

    namespace shoulderConstants{
        // constexpr int armMotorLeft = 12;
        constexpr int armMotorRight = 12;

        // constexpr double encoderOffset;

        constexpr double kP = 0.6; // PID
        constexpr double kI = 0.0;
        constexpr double kD = 0.1;
        constexpr double kFF = 0.00;
        // constexpr double maxAcel = 500;
        // constexpr double maxVelo = 1000;
        // constexpr double minVelo = -1000;
        // max free speed = 5500 rpm
        // gear ratio = 560? unsure 5x7x4x() 
        // i forgot - Tony
        constexpr double maxSpeed = 0.6; 
        constexpr double minSpeed = -0.6;

        constexpr int limitSwtich = 6;
    }

    namespace elbowConstants{
        constexpr int elbowMotor = 11;

        // constexpr double encoderOffset;

        constexpr double kP = 0.1; // PID
        constexpr double kI = 0.0;
        constexpr double kD = 0.0;
        constexpr double kFF = 0.0;
        // constexpr double maxVelo = 1000;
        // constexpr double minVelo = -1000;
        // constexpr double maxAcel = 1000;

        constexpr double maxSpeed = 0.9;
        constexpr double minSpeed = -0.6;
        constexpr int limitSwtich = 7;
    }

    namespace grabberConstants{
        constexpr int grabberMotor = 10;
        
        constexpr double kP = 0.0;
        constexpr double kI = 0.0;
        constexpr double kD = 0.0;
        constexpr double kFF = 0.0;
        //775pro: max free rpm - 18000, probable max rpm - 12000
        //wheel circumfrence: 12.57 inches, gear ratio - unknown
        //idk what gears are in the two planetaries - Tony
        constexpr double maxSpeed = 1.0;
        constexpr double minSpeed = -1.0;
    }

    namespace poseConstants{
        /*
        1. stow
        2. ground cone
        3. ground cube
        4. single station
        5. double station
        6. mid cone align
        7. mid cone score
        8. mid cube
        9. high cone align
        10. high cone score
        11. high cube
        */
        constexpr std::array<double, 8> elbowPositions = {0.0, // stow
                                                           -78.6689224243164, // ground pickup
                                                           0.0, // single station
                                                           -174.8501, // double station
                                                           -107.572, // mid cone 
                                                           -42.8091, // mid cube
                                                           -172.447, // high cone 
                                                           -87};// high cube

        constexpr std::array<double, 8> shoulderPositions = {0.0, // stow
                                                              18.0332124, // ground pickup
                                                              0.0, // single station
                                                              -101.0025, // double station
                                                              -75.738, // mid cone 
                                                              3.0952, // mid cube
                                                              -126.996, // high cone 
                                                              -69};// high cube
        
        constexpr int stow = 0;
        constexpr int groundpickup = 1;
        constexpr int singlestation = 2;
        constexpr int doublestation = 3;
        constexpr int midcone = 4;
        constexpr int midcube =5;
        constexpr int highcone = 6;
        constexpr int highcube = 7;
    }
}


