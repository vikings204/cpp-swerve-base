#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "RobotContainer.h"
#include "Robot.h"

RobotContainer::RobotContainer() {

}

void RobotContainer::ConfigureDefaultCommands() {

}

void RobotContainer::ConfigureButtonBindings() {

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // java code needs to work with new enums you alr know
    // add function in robot that gets value of auto, if that doesnt work then just get rid of this function and move entire thing
    //return pathplanner::PathPlannerAuto(Robot::AutoModeChooser.GetSelected().pathplannerName).ToPtr();
}