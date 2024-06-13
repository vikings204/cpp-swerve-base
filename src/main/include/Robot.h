#pragma once

#include <string>
#include <optional>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc/DriverStation.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

	enum ControlMode {
		SINGLE, COMPETITION
	};

	enum AutoMode {

	};

	frc::SendableChooser<ControlMode> ControlModeChooser;
	frc::SendableChooser<AutoMode> AutoModeChooser;

	frc::DriverStation::Alliance alliance;
	std::string allianceString = "never init";

private:
	std::optional<frc2::CommandPtr> autonomousCommand;
	RobotContainer robotContainer;

	void CheckDriverStationUpdate();
};
