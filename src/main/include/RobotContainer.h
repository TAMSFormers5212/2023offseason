// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
// #include <path


// #include "Constants.h"
#include "subsystems/Swerve.h"
#include "subsystems/SwerveModule.h"
#include "subsystems/Superstructure.h"
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>



/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc::GenericHID m_driverController{0};
  // frc::GenericHID m_operatorController{1};

  // The robot's subsystems are defined here...
  Swerve m_drive;
  // Superstructure m_superStructure;

  // std::unordered_map<std::string, std::shared_ptr<frc2::Command>> m_eventMap;

  // pathplanner::SwerveAutoBuilder m_autoBuilder;



  void ConfigureBindings();
};