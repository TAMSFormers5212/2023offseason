

#include "subsystems/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <cmath>
#include <numbers>
#include <string>
#include <iostream>
#define M_PI 3.14159265358

using enum rev::CANSparkMaxLowLevel::MotorType;
using namespace SwerveModuleConstants;
using namespace rev;

  SwerveModule::SwerveModule(int driveMotorPort, int turningMotorPort, int encoderPort, double offset)
  : encoffset(offset),
    m_driveMotor(driveMotorPort, kBrushless), 
    m_turningMotor(turningMotorPort, kBrushless), 
    m_absoluteEncoder(encoderPort),
    m_moduleName(getName(driveMotorPort)),
    m_turningPIDController(ktP, ktI,ktD)
  {
    resetModule();
    // m_absoluteEncoder.SetOversampleBits(2);
    // m_absoluteEncoder.SetAverageBits(2);

    std::cout<<"Swerve Module "<< getName(driveMotorPort)<<" initalized correctly"<<std::endl; 
  }

  frc::SwerveModuleState SwerveModule::getState(){
    // return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},units::radian_t{m_turningEncoder.GetPosition()}};
    return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},units::radian_t{getAbsolutePosition()}};
    
  }

  frc::SwerveModulePosition SwerveModule::getPosition(){
    return {units::meter_t{m_driveEncoder.GetPosition()},units::radian_t{getAbsolutePosition()}};
  }

  void SwerveModule::resetModule(){
    //reset wheel positions
    resetDriveMotor();
    resetTurningMotor();
  }
  void SwerveModule::resetDriveMotor(){
    m_driveMotor.RestoreFactoryDefaults();
    m_driveController.SetP(kdP);
    m_driveController.SetI(kdI);
    m_driveController.SetD(kdD);
    m_driveController.SetFF(kdFF);
    m_driveMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_driveMotor.EnableVoltageCompensation(12.0);
    m_driveMotor.SetSmartCurrentLimit(25, 50);
    m_driveEncoder.SetPositionConversionFactor((SwerveModuleConstants::wheelCircumfrence/SwerveModuleConstants::driveRatio).value());
    m_driveEncoder.SetVelocityConversionFactor((SwerveModuleConstants::wheelCircumfrence/ SwerveModuleConstants::driveRatio / 60_s).value());
    
    resetDriveEncoder();
  }
  void SwerveModule::resetTurningMotor(){
    // m_turningMotor.RestoreFactoryDefaults();
    resetTurningEncoder();
    m_turningController.SetP(ktP);
    m_turningController.SetI(ktI);
    m_turningController.SetD(ktD);
    m_turningController.SetFF(ktFF);
    m_turningMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_turningMotor.EnableVoltageCompensation(12.0);
    m_turningMotor.SetSmartCurrentLimit(20, 30);
    m_turningController.SetPositionPIDWrappingEnabled(true);
    m_turningEncoder.SetPositionConversionFactor(2*M_PI/ SwerveModuleConstants::steerRatio);
    m_turningPIDController.EnableContinuousInput(-M_PI,M_PI);
    // m_turningEncoder.SetPosition(getAbsolutePosition()*M_2_PI);
    
  }

  void SwerveModule::resetDriveEncoder(){
    m_driveEncoder.SetPosition(0.0);
  }
  void SwerveModule::resetTurningEncoder(){  
    // double rotations = (m_absoluteEncoder.GetVoltage() / frc::RobotController::GetVoltage5V());
    // m_turningEncoder.SetPosition(rotations*2*M_PI);
    m_turningEncoder.SetPosition(getAbsolutePosition());
  }
  double SwerveModule::getDrivePosition(){
    return m_driveEncoder.GetPosition();
  }
  double SwerveModule::getTurningPosition(){
    return m_turningEncoder.GetPosition();
  }
  double SwerveModule::getDriveVelocity(){
    return m_driveEncoder.GetVelocity();
  }
  double SwerveModule::getTurningVelocity(){
    return m_turningEncoder.GetVelocity();
  }
  double SwerveModule::getAbsolutePosition(){ // get position of absolute encoder
    // double rotations = (m_absoluteEncoder.GetVoltage() / frc::RobotController::GetVoltage5V());
    // return rotations * 2 * M_PI;
    return m_absoluteEncoder.GetAbsolutePosition()*2*M_PI+encoffset;
  }

  std::string SwerveModule::getName(int driveMotorID){
    if(driveMotorID == topleft::driveMotor){
        return "top left";
    }else if(driveMotorID == topright::driveMotor){
        return "top right";
    }else if(driveMotorID == bottomleft::driveMotor){
        return "bottom left";
    }else if(driveMotorID == bottomright::driveMotor){
        return "bottom right";
    }else{
        return ""+driveMotorID;
    }
  }

  void SwerveModule::setState(const frc::SwerveModuleState state){
    frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(state, units::radian_t{getTurningPosition()});

    //since the driving motor is relative it doesn't wrap around 2 pi and 0. Therefore we need to calculate the position 
    //delta to be within those bounds. (frc 2363)

    frc::Rotation2d curAngle = units::radian_t{getAbsolutePosition()};

    double delta = std::fmod(std::fmod((optimizedState.angle.Radians().value() -
                                      curAngle.Radians().value() + M_PI),
                                     2 * M_PI) +
                               2 * M_PI,
                           2 * M_PI) -
                 M_PI;   // NOLINT  

    double adjustedAngle = delta + curAngle.Radians().value();
  // if(m_driveMotor.GetDeviceId()==bottomleft::driveMotor){
  //   frc::SmartDashboard::PutNumber("turn ref", adjustedAngle+encoffset);
  // }
    // if (m_driveMotor.GetDeviceId()==9){
    //   frc::SmartDashboard::PutNumber("TL Adjusted Angle",adjustedAngle+encoffset);

    // }
    // if (m_driveMotor.GetDeviceId()==5){
    //   frc::SmartDashboard::PutNumber("TR Adjusted Angle",adjustedAngle+encoffset);

    // }
    // if (m_driveMotor.GetDeviceId()==4){
    //   frc::SmartDashboard::PutNumber("BL Adjusted Angle",adjustedAngle+encoffset);

    // }
    // if (m_driveMotor.GetDeviceId()==7){
    //   frc::SmartDashboard::PutNumber("BR Adjusted Angle",adjustedAngle+encoffset);

    // }
    // m_turningEncoder.SetPosition(getAbsolutePosition());
    // m_turningMotor.Set(m_turningPIDController.Calculate(getAbsolutePosition(), adjustedAngle));
    frc::SmartDashboard::PutNumber("adjusted angle "+getName(m_driveMotor.GetDeviceId()), adjustedAngle);
  
    m_turningController.SetReference((adjustedAngle), CANSparkMax::ControlType::kPosition);
    // if (m_driveMotor.GetDeviceId()==9){
    //   frc::SmartDashboard::PutNumber("Encoder offset", encoffset);
    // }
    
    // m_driveController.SetReference(optimizedState.speed.value(), CANSparkMax::ControlType::kVelocity);
    m_driveMotor.Set(optimizedState.speed / SwerveModuleConstants::maxSpeed);
    // m_turningMotor.
    // frc::SmartDashboard::PutNumber(getName(m_driveMotor.GetDeviceId())+" voltage", m_driveMotor.GetBusVoltage());
  }

  
void SwerveModule::Periodic(){
  // frc::SmartDashboard::PutNumber(getName(this->m_driveMotor.GetDeviceId())+" position", )
}




