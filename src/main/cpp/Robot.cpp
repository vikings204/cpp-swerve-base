#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/ShuffleBoard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>

void Robot::CheckDriverStationUpdate() {
        auto allianceOpt = frc::DriverStation::GetAlliance();

        if (allianceOpt.has_value()) {
            //robotContainer.PoseEstimation.setAlliance(frc::DriverStation::Alliance::kBlue);
            alliance = allianceOpt.value();
            if (allianceOpt == frc::DriverStation::Alliance::kBlue) {
				allianceString = "Blue";
			} else if (allianceOpt == frc::DriverStation::Alliance::kRed) {
				allianceString = "Red";
			}
        }
    }

void Robot::RobotInit() {
	ControlModeChooser.AddOption("Single Controller (Driver:usb1 Operator:usb1)", ControlMode::SINGLE);
	ControlModeChooser.SetDefaultOption("Competition (Driver:usb1 Operator:usb2)", ControlMode::COMPETITION);
	frc::Shuffleboard::GetTab("main").Add("control mode", ControlModeChooser).WithWidget(frc::BuiltInWidgets::kSplitButtonChooser).WithSize(2, 1);

	// java code, also need to add in the java style enum into cpp
	// for (AutoMode i : AutoMode.values()) {
	// 	AutoModeChooser.addOption(i.optionName(), i);
	// }

	frc::Shuffleboard::GetTab("main").Add("Auto Select", AutoModeChooser).WithSize(3, 1);
	CheckDriverStationUpdate();
	frc::Shuffleboard::GetTab("main").AddString("alliance", [this] {return allianceString;});
}

void Robot::RobotPeriodic() {
	frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
	autonomousCommand = robotContainer.GetAutonomousCommand();

	// schedule the autonomous command (example)
	if (autonomousCommand.has_value()) {
		autonomousCommand->Schedule();
	}
	CheckDriverStationUpdate();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
	if (autonomousCommand.has_value()) {
		autonomousCommand->Cancel();
	}
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif
