#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>

#include "commands/Autos/onechigh.h"
#include "Constants.h"


onechigh::onechigh(Swerve* drive, Superstructure* arm) {    
    AddCommands(
        frc2::ParallelRaceGroup{
            frc2::RunCommand([arm]() {arm->goToPose(arm->getPose("high cube"));}),
            frc2::WaitCommand(4_s)
        },
        frc2::ParallelRaceGroup{
            frc2::RunCommand(
                [this, arm] {
                    arm->setGrabber(1.0);  
                },
                {}
            ),
            frc2::WaitCommand(2_s)
        },
        frc2::ParallelRaceGroup{
            frc2::RunCommand([arm]() {arm->goToPose(arm->getPose("stow"));}),
            frc2::RunCommand([drive]() {drive->swerveDrive(0, -0.5, 0, true);}),
            frc2::WaitCommand(1.5_s)
        }
    );
    
}