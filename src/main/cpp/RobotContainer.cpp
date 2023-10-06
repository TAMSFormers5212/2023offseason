// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/InstantCommand.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

#include "commands/Autos.h"

using namespace std;

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
        // cout<<XAxis<<" "<<YAxis<<" "<<RotAxis<<endl;
        // frc::SmartDashboard::PutNumber("X axis", XAxis);
        // frc::SmartDashboard::PutNumber("Y axis", YAxis);
        // frc::SmartDashboard::PutNumber("Rotation axis", RotAxis);
        double speedMultiplier = (1-m_driverController.GetRawAxis(OIConstants::Joystick::ThrottleSlider))*0.5;
        m_drive.swerveDrive(
            std::abs(XAxis) < OIConstants::Joystick::deadband ? 0.0 : XAxis*speedMultiplier,
            std::abs(YAxis) < OIConstants::Joystick::deadband ? 0.0 : YAxis*speedMultiplier,
            std::abs(RotAxis) < OIConstants::Joystick::deadband ? 0.0 : RotAxis*speedMultiplier,
            true
            );
      },
      {&m_drive}  // requirements
      ));

      m_superStructure.SetDefaultCommand(frc2::RunCommand(
        [this] {
          // return m_superStructure.goToPose(m_superStructure.getCurPose());
        m_superStructure.setGrabber(m_operatorController.GetRawAxis(OIConstants::Controller::leftTrigger)-m_operatorController.GetRawAxis(OIConstants::Controller::rightTrigger));
        
        if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::LPress)){
          m_superStructure.setManual(!m_superStructure.getManual());
        }
        if(m_superStructure.getManual()){
          m_superStructure.manualAdjust(
            m_operatorController.GetRawAxis(OIConstants::Controller::leftYAxis),
            m_operatorController.GetRawAxis(OIConstants::Controller::rightYAxis));
        }else{
          m_superStructure.goToPose(m_superStructure.getCurPose());
        }
        if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::A)){
          m_superStructure.setPose(m_superStructure.getPose("stow"));
        }else if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::B)){
          m_superStructure.setPose(m_superStructure.getPose("ground pickup"));
        }else if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::X)){
          m_superStructure.setPose(m_superStructure.getPose("single station"));
        }else if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::Y)){
          m_superStructure.setPose(m_superStructure.getPose("double station"));
        }else if(m_operatorController.GetPOV()>=0&&(m_operatorController.GetPOV()>315||m_operatorController.GetPOV()<=45)){
          m_superStructure.setPose(m_superStructure.getPose("high cone"));
        }else if(m_operatorController.GetPOV()>=0&&(m_operatorController.GetPOV()>45||m_operatorController.GetPOV()<=135)){
          m_superStructure.setPose(m_superStructure.getPose("high cube"));
        }else if(m_operatorController.GetPOV()>=0&&(m_operatorController.GetPOV()>135||m_operatorController.GetPOV()<=225)){
          m_superStructure.setPose(m_superStructure.getPose("mid cube"));
        }else if(m_operatorController.GetPOV()>=0&&(m_operatorController.GetPOV()>225||m_operatorController.GetPOV()<315)){
          m_superStructure.setPose(m_superStructure.getPose("mid cone"));
        }else if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::RPress)){
          m_superStructure.resetEncoders();
        }
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

  frc2::JoystickButton(&m_driverController, OIConstants::Joystick::Trigger).WhenReleased(
    frc2::InstantCommand(
      [this]{
        m_drive.resetAbsoluteEncoders();
      }, {&m_drive}
    )
  );

  frc2::JoystickButton(&m_driverController, OIConstants::Joystick::ButtonThree).WhenReleased(
    frc2::InstantCommand(
      [this]{
        m_drive.resetHeading();
      }, {&m_drive}
    )
  );



  //m_superStructure.setGrabber(m_operatorController.GetRawAxis(OIConstants::Controller::leftTrigger) - m_operatorController.GetRawAxis(OIConstants::Controller::rightTrigger));
          
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_drive, &m_superStructure);
}
