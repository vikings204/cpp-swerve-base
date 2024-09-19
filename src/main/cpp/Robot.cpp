#include "Robot.h"

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
	frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
	autonomousCommand = robotContainer.GetAutonomousCommand();

	// schedule the autonomous command (example)
	if (autonomousCommand.has_value()) {
		autonomousCommand->Schedule();
	}
	robotContainer.CheckDriverStationUpdate();
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
