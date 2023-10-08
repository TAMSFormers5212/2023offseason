#include "commands/SetArmPose.h"

#include "Constants.h"

using namespace ArmConstants;

SetArmPose::SetArmPose(Superstructure* arm, armPose pose)
    : m_arm(arm), m_pose(pose) {
  AddRequirements(arm);
}

void SetArmPose::Initialize() {
  m_arm->goToPose(m_pose);
}

void SetArmPose::End(bool interrupted) {}