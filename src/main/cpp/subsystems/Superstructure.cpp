#include "subsystems/Superstructure.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using enum rev::CANSparkMaxLowLevel::MotorType;
using namespace rev;
using namespace ArmConstants;

Superstructure::~Superstructure()
{
    std::cout<<"superstructure initalized correctly"<<std::endl;
}

Superstructure::Superstructure()
//   : m_shoulderMotorLeft(shoulderConstants::armMotorLeft, kBrushless),
    : m_shoulderMotorRight(shoulderConstants::armMotorRight, kBrushless),
    m_elbowMotor(elbowConstants::elbowMotor, kBrushless),
    m_relativeShoulderEncoder(m_shoulderMotorRight.GetEncoder()),
    m_relativeElbowEncoder(m_elbowMotor.GetEncoder()),
    m_shoulderController(m_shoulderMotorRight.GetPIDController()),
    m_elbowController(m_elbowMotor.GetPIDController()),
    // shoulderLimitSwtich(shoulderConstants::limitSwtich),
    // elbowLimitSwitch(elbowConstants::limitSwtich),
    m_grabberMotor(grabberConstants::grabberMotor)
{


    m_shoulderMotorRight.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_shoulderMotorRight.EnableVoltageCompensation(12.0);
    m_shoulderMotorRight.SetSmartCurrentLimit(40);
    m_shoulderMotorRight.SetSoftLimit(CANSparkMax::SoftLimitDirection::kForward, 26);
    m_shoulderMotorRight.SetSoftLimit(CANSparkMax::SoftLimitDirection::kReverse,-150);

    m_shoulderController.SetP(shoulderConstants::kP);
    m_shoulderController.SetI(shoulderConstants::kI);
    m_shoulderController.SetD(shoulderConstants::kD);
    m_shoulderController.SetFF(shoulderConstants::kFF);
    m_shoulderController.SetOutputRange(shoulderConstants::minSpeed, shoulderConstants::maxSpeed);
    // m_shoulderController.SetSmartMotionMaxAccel(shoulderConstants::maxAcel);
    // m_shoulderController.SetSmartMotionMaxVelocity(shoulderConstants::maxVelo);
    // m_shoulderController.SetSmartMotionAccelStrategy(CANPIDController::AccelStrategy::kTrapezoidal);
    
    // m_shoulderMotorLeft.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    // m_shoulderMotorLeft.EnableVoltageCompensation(12.0);
    // m_shoulderMotorLeft.SetSmartCurrentLimit(40);
    // m_shoulderMotorLeft.SetSoftLimit(CANSparkMax::SoftLimitDirection::kForward, 26);
    // m_shoulderMotorLeft.SetSoftLimit(CANSparkMax::SoftLimitDirection::kReverse,-150);


    // m_shoulderMotorRight.Follow(m_shoulderMotorLeft, true);

    m_elbowController.SetP(elbowConstants::kP);
    m_elbowController.SetI(elbowConstants::kI);
    m_elbowController.SetD(elbowConstants::kD);
    m_elbowController.SetFF(elbowConstants::kFF);
    m_elbowController.SetOutputRange(elbowConstants::minSpeed, elbowConstants::maxSpeed);
    // m_elbowController.SetSmartMotionMaxAccel(elbowConstants::maxAcel);
    // m_elbowController.SetSmartMotionMaxVelocity(elbowConstants::maxVelo);
    // m_elbowController.SetSmartMotionAccelStrategy(CANPIDController::AccelStrategy::kTrapezoidal);
    
    m_elbowMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_elbowMotor.EnableVoltageCompensation(12.0);
    m_elbowMotor.SetSmartCurrentLimit(40);

    m_pose.setShoulderPose(m_relativeShoulderEncoder.GetPosition());
    m_pose.setElbowPose(m_relativeElbowEncoder.GetPosition());


    std::cout<<"superstructure initalized correctly"<<std::endl;
}

armPose Superstructure::getCurPose(){
    m_pose.setElbowPose(m_relativeElbowEncoder.GetPosition());
    m_pose.setShoulderPose(m_relativeShoulderEncoder.GetPosition());
    return m_pose;
}

void Superstructure::goToPose(armPose pose){
    frc::SmartDashboard::PutNumber("goal shoulder", pose.getShoulderPose());
    frc::SmartDashboard::PutNumber("goal elbow", pose.getElbowPose());
    m_shoulderController.SetReference(pose.getShoulderPose(), rev::CANSparkMax::ControlType::kPosition);
    m_elbowController.SetReference(pose.getElbowPose(), rev::CANSparkMax::ControlType::kPosition);
}

void Superstructure::resetPose(){
    // m_shoulderController.SetReference(-10, rev::CANSparkMax::ControlType::kVelocity);
    // if(shoulderLimitSwtich.Get()){
    //     m_shoulderController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);
    //     m_relativeShoulderEncoder.SetPosition(0);
    // }
    m_relativeShoulderEncoder.SetPosition(0);
    m_relativeElbowEncoder.SetPosition(0);
}


void Superstructure::manualAdjust(double shoulder, double elbow){
    // m_shoulderController.SetReference(shoulder*shoulderConstants::maxVelo, rev::CANSparkMax::ControlType::kVelocity);
    // m_elbowController.SetReference(elbow*elbowConstants::maxVel, rev::CANSparkMax::ControlType::kVelocity);

    m_shoulderMotorRight.Set(shoulder*0.5);
    m_elbowMotor.Set(elbow*0.5);
}

void Superstructure::Periodic(){
    // frc::SmartDashboard::PutNumber("Shoulder Position", m_relativeShoulderEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Elbow Position", m_relativeElbowEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Shoulder Position", m_relativeShoulderEncoder.GetPosition());
}

void Superstructure::setGrabber(double speed){
    // if(m_grabberMotor.GetTemperature()>50){
        // m_grabberMotor.Set(VictorSPXControlMode::PercentOutput, 0);
    // }else{
        m_grabberMotor.Set(VictorSPXControlMode::PercentOutput, speed);
    // }
    frc::SmartDashboard::PutNumber("grabber speed", speed);
}

void Superstructure::setManual(bool man){
    manual = man;
}

bool Superstructure::getManual(){
    return manual;
}

void Superstructure::setSoftLimitEnabled(bool enable){
    softLimitEnabled = enable;
    m_elbowMotor.EnableSoftLimit(CANSparkMax::SoftLimitDirection::kForward,softLimitEnabled);
    m_elbowMotor.EnableSoftLimit(CANSparkMax::SoftLimitDirection::kReverse,softLimitEnabled);
    // m_shoulderMotor.EnableSoftLimit(CANSparkMax::SoftLimitDirection::kForward,softLimitEnabled);
    // m_shoulderMotorLeft.EnableSoftLimit(CANSparkMax::SoftLimitDirection::kReverse,softLimitEnabled);
    m_shoulderMotorRight.EnableSoftLimit(CANSparkMax::SoftLimitDirection::kForward,softLimitEnabled);
    m_shoulderMotorRight.EnableSoftLimit(CANSparkMax::SoftLimitDirection::kReverse,softLimitEnabled);
}

bool Superstructure::getSoftLimitEnabled(){
    return softLimitEnabled;
}

void Superstructure::setPose(armPose pose){
    this->m_pose = pose;
}

void Superstructure::resetEncoders(){
    m_relativeElbowEncoder.SetPosition(0);
    m_relativeShoulderEncoder.SetPosition(0);
}

armPose Superstructure::getPose(){
    return m_pose;
}