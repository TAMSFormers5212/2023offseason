// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>

#include "commands/Autos.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // m_drive = Swerve(0);
  
    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {  // onExecute
        // Right stick up on xbox is negative, right stick down is postive.
        // Right stick right on xbox is negative, right stick left is postive.
        // Left stick right is positive, left stick left is negative.
        double XAxis = -m_driverController.GetRawAxis(OIConstants::Joystick::XAxis);
        double YAxis = -m_driverController.GetRawAxis(OIConstants::Joystick::YAxis);
        double RotAxis = -m_driverController.GetRawAxis(OIConstants::Joystick::RotAxis);
        return m_drive.swerveDrive(
            std::abs(XAxis) < 0.025 ? 0.0 : XAxis,
            std::abs(YAxis) < 0.025 ? 0.0 : YAxis,
            std::abs(RotAxis) < 0.025 ? 0.0 : RotAxis,
            true
            );
      },
      {&m_drive}  // requirements
      ));

      m_superStructure.SetDefaultCommand(frc2::RunCommand(
        [this] {
          //
        },
        {&m_superStructure}
      ));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_drive, &m_superStructure);
}
