#include "subsystems/Superstructure/Superstructure.h"
#include <frc/smartdashboard/SmartDashboard.h>

using enum rev::CANSparkMaxLowLevel::MotorType;
using namespace rev;
using namespace ArmConstants;

Superstructure::Superstructure(int shoulderLeftPort, int shoulderRightPort, int elbowPort, int limitSwitchPort)
  : m_shoulderMotorLeft(shoulderLeftPort, kBrushless),
    m_shoulderMotorRight(shoulderRightPort, kBrushless),
    m_elbowMotor(elbowPort, kBrushless),
    // m_absoluteShoulderEncoder(shoulderEncoderPort),
    // m_absoluteElbowEncoder(elbowEncoderPort),
    m_relativeShoulderEncoder(m_shoulderMotorLeft.GetEncoder()),
    m_relativeElbowEncoder(m_elbowMotor.GetEncoder()),
    m_shoulderController(m_shoulderMotorLeft.GetPIDController()),
    m_elbowController(m_elbowMotor.GetPIDController()),
    shoulderLimitSwtich(limitSwitchPort)
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

    m_pose.set(m_relativeShoulderEncoder.GetPosition(),m_relativeElbowEncoder.GetPosition());

    frc::SmartDashboard::PutNumber("Shoulder Position", m_relativeShoulderEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Elbow Position", m_relativeElbowEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Shoulder Speed", m_shoulderController.GetSmartMotionMaxVelocity());
}

armPose Superstructure::getPose(){
    return m_pose;
}

void Superstructure::setPose(armPose pose){
    
}

void Superstructure::resetPose(){
    m_shoulderController.SetReference(-10, rev::CANSparkMax::ControlType::kVelocity);
    if(shoulderLimitSwtich.Get()){
        m_shoulderController.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);
        m_relativeShoulderEncoder.SetPosition(0);
    }
}
