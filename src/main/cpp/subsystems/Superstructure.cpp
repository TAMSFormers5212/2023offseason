#include "subsystems/Superstructure.h"
#include <frc/smartdashboard/SmartDashboard.h>

using enum rev::CANSparkMaxLowLevel::MotorType;
using namespace rev;
using namespace ArmConstants;

Superstructure::~Superstructure()
{
}

Superstructure::Superstructure()
  : m_shoulderMotorLeft(shoulderConstants::armMotorLeft, kBrushless),
    m_shoulderMotorRight(shoulderConstants::armMotorRight, kBrushless),
    m_elbowMotor(elbowConstants::elbowMotor, kBrushless),
    m_relativeShoulderEncoder(m_shoulderMotorLeft.GetEncoder()),
    m_relativeElbowEncoder(m_elbowMotor.GetEncoder()),
    // m_shoulderController(m_shoulderMotorLeft.GetPIDController()),
    // m_elbowController(m_elbowMotor.GetPIDController()),
    shoulderLimitSwtich(shoulderConstants::limitSwtich)
{
    m_shoulderController.SetP(shoulderConstants::kP);
    m_shoulderController.SetI(shoulderConstants::kI);
    m_shoulderController.SetD(shoulderConstants::kD);
    m_shoulderController.SetFF(shoulderConstants::kFF);
    m_shoulderController.SetOutputRange(shoulderConstants::minSpeed, shoulderConstants::maxSpeed);
    m_shoulderController.SetSmartMotionMaxAccel(shoulderConstants::maxAcel);
    
    m_shoulderMotorLeft.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_shoulderMotorLeft.EnableVoltageCompensation(12.0);
    m_shoulderMotorLeft.SetSmartCurrentLimit(40);
    m_shoulderMotorLeft.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 26);

    m_shoulderMotorRight.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_shoulderMotorRight.EnableVoltageCompensation(12.0);
    m_shoulderMotorRight.SetSmartCurrentLimit(40);
    m_shoulderMotorLeft.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 26);

    m_shoulderMotorRight.Follow(m_shoulderMotorLeft, true);

    m_elbowController.SetP(elbowConstants::kP);
    m_elbowController.SetI(elbowConstants::kI);
    m_elbowController.SetD(elbowConstants::kD);
    m_elbowController.SetFF(elbowConstants::kFF);
    
    m_elbowMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_elbowMotor.EnableVoltageCompensation(12.0);
    m_elbowMotor.SetSmartCurrentLimit(40);

    m_pose.setShoulderPose(m_relativeShoulderEncoder.GetPosition());
    m_pose.setElbowPose(m_relativeElbowEncoder.GetPosition());

    frc::SmartDashboard::PutNumber("Shoulder Position", m_relativeShoulderEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Elbow Position", m_relativeElbowEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Shoulder Speed", m_shoulderController.GetSmartMotionMaxVelocity());

    for(int i =0;i<int(names->length());i++){
        poses.push_back(armPose(ArmConstants::poseConstants::shoulderPositions[i], ArmConstants::poseConstants::elbowPositions[i], i, names[i]));
    }
}

armPose Superstructure::getCurPose(){
    m_pose.setElbowPose(m_relativeElbowEncoder.GetPosition());
    m_pose.setShoulderPose(m_relativeShoulderEncoder.GetPosition());
    return m_pose;
}

void Superstructure::goToPose(armPose pose){
    m_shoulderController.SetReference(pose.getShoulderPose(), rev::CANSparkMax::ControlType::kSmartMotion);
    m_elbowController.SetReference(pose.getElbowPose(), rev::CANSparkMax::ControlType::kSmartMotion);
}

void Superstructure::resetPose(){
    m_shoulderController.SetReference(-10, rev::CANSparkMax::ControlType::kVelocity);
    if(shoulderLimitSwtich.Get()){
        m_shoulderController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);
        m_relativeShoulderEncoder.SetPosition(0);
    }
}

armPose Superstructure::getPose(std::string name){
    for(armPose pose: poses){
        if(name.compare(pose.getName())==0){
            return pose;
        }
    }
    return poses.at(0); // 0 is stowed position
}

void Superstructure::manualAdjust(double shoulder, double elbow){
    m_shoulderController.SetReference(shoulder, rev::CANSparkMax::ControlType::kVelocity);
    m_elbowController.SetReference(elbow, rev::CANSparkMax::ControlType::kVelocity);
}

void Superstructure::Periodic(){
    getCurPose();
}

void Superstructure::groundCone(){
    goToPose(getPose("ground cone"));
}