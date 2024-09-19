#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include <networktables/GenericEntry.h>

#include "Constants.h"
#include "subsystems/SwerveSubsystem.h"

class RobotContainer {
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

	void CheckDriverStationUpdate();
private:
	SwerveSubsystem Swerve;

    frc::XboxController DRIVER {Constants::Controller::DRIVER_PORT};
    frc::XboxController OPERATOR {Constants::Controller::DRIVER_PORT};

    void ConfigureDefaultCommands();
    void ConfigureButtonBindings();

	enum ControlMode {
		SINGLE, COMPETITION
	};
	enum AutoMode {

	};
    frc::SendableChooser<ControlMode> ControlModeChooser;
	frc::SendableChooser<AutoMode> AutoModeChooser;

	nt::GenericEntry* finalSpeedModifierEntry;

	frc::DriverStation::Alliance alliance;
	std::string allianceString = "never init";
};