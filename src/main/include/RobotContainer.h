#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>

#include "Constants.h"

class RobotContainer {
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();
private:
    frc::XboxController DRIVER {Constants::Controller::DRIVER_PORT};
    frc::XboxController OPERATOR {Constants::Controller::DRIVER_PORT};

    void ConfigureDefaultCommands();
    void ConfigureButtonBindings();
};