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
#include <frc2/command/StartEndCommand.h>

#include "commands/SetArmPose.h"
#include "commands/Autos/onechigh.h"

using namespace std;
using namespace ArmConstants::poseConstants;
using namespace tamsmath;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // m_drive = Swerve(0);

  
    m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {  // onExecute
        // Right stick up on xbox is negative, right stick down is postive.
        // Right stick right on xbox is negative, right stick left is postive.
        // Left stick right is positive, left stick left is negative.
        double XAxis = m_driverController.GetRawAxis(OIConstants::Joystick::XAxis);
        double YAxis = m_driverController.GetRawAxis(OIConstants::Joystick::YAxis);
        double RotAxis = m_driverController.GetRawAxis(OIConstants::Joystick::RotAxis);
        // cout<<XAxis<<" "<<YAxis<<" "<<RotAxis<<endl;
        // frc::SmartDashboard::PutNumber("X axis", XAxis);
        // frc::SmartDashboard::PutNumber("Y axis", YAxis);
        // frc::SmartDashboard::PutNumber("Rotation axis", RotAxis);
        double speedMultiplier = (1-m_driverController.GetRawAxis(OIConstants::Joystick::ThrottleSlider))*0.5;
        double throttle = (1-m_driverController.GetRawAxis(OIConstants::Joystick::ThrottleSlider))*0.5;
        XAxis*=speedMultiplier;
        YAxis*=speedMultiplier;
        RotAxis*=speedMultiplier;

        // m_drive.swerveDrive(
        //     std::abs(XAxis) < OIConstants::Joystick::deadband ? 0.0 : XAxis*speedMultiplier,
        //     std::abs(YAxis) < OIConstants::Joystick::deadband ? 0.0 : YAxis*speedMultiplier,
        //     std::abs(RotAxis) < OIConstants::Joystick::deadband/2 ? 0.0 : RotAxis*speedMultiplier,
        //     true
        //     );
        // if(m_driverController.GetRawButtonPressed(OIConstants::Joystick::ButtonThree)){ 
        //   // m_drive.SyncAbsoluteEncoders();
        // }
        if(m_driverController.GetRawButton(11)){
          m_drive.moveToAngle(cos(throttle*2*pi)*0.3, sin(throttle*2*pi)*0.3);
          
        }else{        
          m_drive.moveToAngle(cos(pi)*0.3, sin(pi)*0.3);
          frc::SmartDashboard::PutNumber("Throttle", throttle);
        }
      },

      {&m_drive}  // requirements
      ));

      // m_superStructure.SetDefaultCommand(frc2::RunCommand(
      //   [this] {
      //     // return m_superStructure.goToPose(m_superStructure.getCurPose());]
      //     // m_superStructure.setGrabber(m_operatorController.GetRawAxis(OIConstants::Controller::leftTrigger)-m_operatorController.GetRawAxis(OIConstants::Controller::rightTrigger));
          

      //     // if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::LPress)){
      //     //   m_superStructure.setManual(!m_superStructure.getManual());
      //     // }
      //     frc::SmartDashboard::PutBoolean("manual", m_superStructure.getManual());
          // if(m_superStructure.getManual()){
          //   m_superStructure.manualAdjust(
          //     m_operatorController.GetRawAxis(OIConstants::Controller::leftYAxis),
          //     m_operatorController.GetRawAxis(OIConstants::Controller::rightYAxis));
          // }
          // else{
          //   m_superStructure.goToPose(m_superStructure.getPose());
          // }

          // if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::View)){
          //   if(m_superStructure.getSoftLimitEnabled()){
          //     m_superStructure.setSoftLimitEnabled(false);
          //   }else{
          //     m_superStructure.setSoftLimitEnabled(true);
          //   }
          // }


        // if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::A)){
        //   m_superStructure.setPose(m_superStructure.getPose("stow"));
        // }else if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::B)){
        //   m_superStructure.setPose(m_superStructure.getPose("ground pickup"));
        // }else if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::X)){
        //   m_superStructure.setPose(m_superStructure.getPose("single station"));
        // }else if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::Y)){
        //   m_superStructure.setPose(m_superStructure.getPose("double station"));
        // }else if(m_operatorController.GetPOV()>=0&&(m_operatorController.GetPOV()>315||m_operatorController.GetPOV()<=45)){
        //   m_superStructure.setPose(m_superStructure.getPose("high cone"));
        //   // m_operatorController
        // }else if(m_operatorController.GetPOV()>=0&&(m_operatorController.GetPOV()>45||m_operatorController.GetPOV()<=135)){
        //   m_superStructure.setPose(m_superStructure.getPose("high cube"));
        // }else if(m_operatorController.GetPOV()>=0&&(m_operatorController.GetPOV()>135||m_operatorController.GetPOV()<=225)){
        //   m_superStructure.setPose(m_superStructure.getPose("mid cube"));
        // }else if(m_operatorController.GetPOV()>=0&&(m_operatorController.GetPOV()>225||m_operatorController.GetPOV()<=315)){
        //   m_superStructure.setPose(m_superStructure.getPose("mid cone"));
        // }else if(m_operatorController.GetRawButtonPressed(OIConstants::Controller::RPress)){
        //   m_superStructure.resetEncoders();
        // }
        
      //   },
      //   {&m_superStructure}
      // ));

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

  frc2::JoystickButton(&m_driverController, OIConstants::Joystick::Trigger).WhenHeld(
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

  // frc2::JoystickButton(&m_operatorController, OIConstants::Controller::leftBumper).WhileHeld(
  //   frc2::RunCommand{
  //     [this]{
  //       m_superStructure.setGrabber(0.1+0.5*m_operatorController.GetRawAxis(OIConstants::Controller::leftTrigger));
  //     },
  //     {&m_superStructure}
  //   } 
  // ); 

  
  // frc2::JoystickButton(&m_operatorController, OIConstants::Controller::rightBumper).WhileHeld(
  //   frc2::RunCommand{
  //     [this]{
  //       m_superStructure.setGrabber(-0.1-0.5*m_operatorController.GetRawAxis(OIConstants::Controller::rightTrigger));
  //     },
  //     {&m_superStructure}
  //   } 
  // ); 
  // frc2::JoystickButton(&m_operatorController, OIConstants::Controller::LPress).WhenPressed(
  //   frc2::InstantCommand{
  //     [this]{
  //       m_superStructure.setManual(!m_superStructure.getManual());
  //     },
  //     {&m_superStructure}
  //   } 
  // ); 

  // frc2::JoystickButton(&m_operatorController, 8).WhileHeld(
  //   frc2::RunCommand{
  //     [this]{
  //       m_superStructure.setGrabber(m_operatorController.GetRawAxis(OIConstants::Controller::leftTrigger)-m_operatorController.GetRawAxis(OIConstants::Controller::rightTrigger));
        
  //     },
  //     {&m_superStructure}
  //   } 
  // ); 


  // frc2::JoystickButton(&m_operatorController, 7).WhileHeld(
  //   frc2::RunCommand{
  //     [this]{
  //          m_superStructure.manualAdjust(
  //             m_operatorController.GetRawAxis(OIConstants::Controller::leftYAxis),
  //             m_operatorController.GetRawAxis(OIConstants::Controller::rightYAxis));
          
  //     },
  //     {&m_superStructure}
  //   } 
  // ); 

  // frc2::JoystickButton(&m_operatorController, OIConstants::Controller::RPress).WhenPressed(
  //   frc2::InstantCommand{
  //     [this]{
  //       m_superStructure.resetEncoders();
  //     }
  //   }
  // );



  // frc2::JoystickButton

//   frc2::JoystickButton(&m_operatorController, OIConstants::Controller::A).WhenPressed(
//     SetArmPose(&m_superStructure, armPose{shoulderPositions[stow], shoulderPositions[stow]})
//   );
//   frc2::JoystickButton(&m_operatorController, OIConstants::Controller::B).WhenPressed(
//     SetArmPose(&m_superStructure, armPose{shoulderPositions[groundpickup], elbowPositions[groundpickup]})
//   );
//   frc2::JoystickButton(&m_operatorController, OIConstants::Controller::X).WhenPressed(
//     SetArmPose(&m_superStructure, armPose{shoulderPositions[singlestation], elbowPositions[singlestation]})
//   );
//   frc2::JoystickButton(&m_operatorController, OIConstants::Controller::Y).WhenPressed(
//     SetArmPose(&m_superStructure, armPose{shoulderPositions[doublestation], elbowPositions[doublestation]})
//   );
//   // frc2::POVButton(&m_operatorController, 90)
//   frc2::POVButton(&m_operatorController, 0).WhenPressed(
//     SetArmPose(&m_superStructure, armPose{shoulderPositions[highcone], elbowPositions[highcone]})
//   );
//   frc2::POVButton(&m_operatorController, 90).WhenPressed(
//     SetArmPose(&m_superStructure, armPose{shoulderPositions[highcube], elbowPositions[highcube]})
//   );frc2::POVButton(&m_operatorController, 180).WhenPressed(
//     SetArmPose(&m_superStructure, armPose{shoulderPositions[midcube], elbowPositions[midcube]})
//   );
//   frc2::POVButton(&m_operatorController, 270).WhenPressed(
//     SetArmPose(&m_superStructure, armPose{shoulderPositions[midcone], elbowPositions[midcone]})
//   );
//   //m_superStructure.setGrabber(m_operatorController.GetRawAxis(OIConstants::Controller::leftTrigger) - m_operatorController.GetRawAxis(OIConstants::Controller::rightTrigger));
          
}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   return onechigh(&m_drive, &m_superStructure).ToPtr();
// }
