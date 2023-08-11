

#include "subsystems/SwerveModule.h"
#include <cmath>
#include <numbers>
#include <string>

using enum rev::CANSparkMaxLowLevel::MotorType;
using namespace SwerveModuleConstants;
using namespace rev;

  SwerveModule::SwerveModule(int driveMotorPort, int turningMotorPort, int encoderPort, double offset)
  : encoffset(offset),
    m_driveMotor(driveMotorPort, kBrushless), 
    m_turningMotor(turningMotorPort, kBrushless), 
    m_driveEncoder(m_driveMotor.GetEncoder()),
    m_turningEncoder(m_turningMotor.GetEncoder()),
    m_driveController(m_driveMotor.GetPIDController()),
    m_turningController(m_turningMotor.GetPIDController()),
    m_absoluteEncoder(encoderPort),
    m_moduleName(getName(driveMotorPort))
  {
    m_driveController.SetP(kdP);
    m_driveController.SetI(kdI);
    m_driveController.SetD(kdD);
    m_driveController.SetFF(kdFF);
    m_driveMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_driveMotor.EnableVoltageCompensation(12.0);
    m_driveMotor.SetSmartCurrentLimit(40);

    m_turningController.SetP(ktP);
    m_turningController.SetI(ktI);
    m_turningController.SetD(ktD);
    m_turningController.SetFF(ktFF);
    m_turningMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_turningMotor.EnableVoltageCompensation(12.0);
    m_turningMotor.SetSmartCurrentLimit(20);
  }

  frc::SwerveModuleState SwerveModule::getState(){
    return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},units::radian_t{m_turningEncoder.GetPosition()}};
  }

  frc::SwerveModulePosition SwerveModule::getPosition(){
    return {units::meter_t{m_driveEncoder.GetPosition()},units::radian_t{m_turningEncoder.GetPosition()}};
  }

  void SwerveModule::resetModule(){
    //reset wheel positions
    resetDriveMotor();
    resetTurningMotor();
  }
  void SwerveModule::resetDriveMotor(){

  }
  void SwerveModule::resetTurningMotor(){

  }
  void SwerveModule::resetDriveEncoder(){

  }
  void SwerveModule::resetTurningEncoder(){
    encoffset = m_absoluteEncoder.GetValue();
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

  
