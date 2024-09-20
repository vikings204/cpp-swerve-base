#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/ShuffleBoard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>

#include "RobotContainer.h"
#include "Robot.h"
#include "commands/TeleopSwerveCommand.h"

RobotContainer::RobotContainer() {
	ControlModeChooser.AddOption("Single Controller (Driver:usb1 Operator:usb1)", ControlMode::COMPETITION);
	ControlModeChooser.SetDefaultOption("Competition (Driver:usb1 Operator:usb2)", ControlMode::SINGLE);
	frc::Shuffleboard::GetTab("main").Add("control mode", ControlModeChooser).WithWidget(frc::BuiltInWidgets::kSplitButtonChooser).WithSize(2, 1);
    finalSpeedModifierEntry = frc::Shuffleboard::GetTab("config").Add("final speed modifier", 1.0).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithProperties({{"min", nt::Value::MakeDouble(0)},{"max", nt::Value::MakeDouble(1)}}).GetEntry();

	// java code, also need to add in the java style enum into cpp
	// for (AutoMode i : AutoMode.values()) {
	// 	AutoModeChooser.addOption(i.optionName(), i);
	// }

	frc::Shuffleboard::GetTab("main").Add("Auto Select", AutoModeChooser).WithSize(3, 1);
	CheckDriverStationUpdate();
	frc::Shuffleboard::GetTab("main").AddString("alliance", [this] {return allianceString;});
    
    ControlModeChooser.OnChange([this] (ControlMode mode) {
        if (mode == ControlMode::SINGLE) {
            OPERATOR = frc::XboxController(Constants::Controller::DRIVER_PORT);
        } else {
            OPERATOR = frc::XboxController(Constants::Controller::OPERATOR_PORT);
        }
        ConfigureDefaultCommands();
        ConfigureButtonBindings();
    });

    frc::Shuffleboard::GetTab("main").Add("swerve", Swerve);
    // frc::Shuffleboard::GetTab("main").Add("shooter", Shooter);

    // pathplanner::AutoBuilder::configureHolonomic(
    //     [this] { return getPose(); }, // Robot pose supplier
    //     [this](frc::Pose2d pose){ resetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
    //     [this] { return Swerve.GetSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //     [this](frc::ChassisSpeeds speeds){ Swerve.DriveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //     HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //         PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //         PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //         4.5_mps, // Max module speed, in m/s
    //         0.4_m, // Drive base radius in meters. Distance from robot center to furthest module.
    //         ReplanningConfig() // Default path replanning config. See the API for the options here
    //     ),
    //     [this] {return alliance == frc::DriverStation::Alliance::kRed;},
    //     &Swerve // Reference to this subsystem to set requirements
    // );

    // NamedCommands.registerCommand("intakeStart", new InstantCommand(() -> Shooter.receive(true), Shooter));
    // NamedCommands.registerCommand("zeroGyro", new InstantCommand(Swerve::zeroGyro, Swerve));
    // NamedCommands.registerCommand("intakeStop", new InstantCommand(() -> Shooter.receive(false), Shooter));
    // NamedCommands.registerCommand("shooterStart", new InstantCommand(() -> Shooter.flywheelSpeaker(true), Shooter));
    // NamedCommands.registerCommand("shooterStop", new InstantCommand(() -> Shooter.flywheelSpeaker(false), Shooter));
    // NamedCommands.registerCommand("bumpStart", new InstantCommand(() -> Shooter.intake(true, false), Shooter));
    // NamedCommands.registerCommand("bumpStop", new InstantCommand(() -> Shooter.intake(false, false), Shooter));
    // NamedCommands.registerCommand("lowerIntake", new InstantCommand(() -> LinearActuator.shift(false,true), LinearActuator));
    // NamedCommands.registerCommand("lowerIntakeStop", new InstantCommand(() -> LinearActuator.shift(false,false), LinearActuator));

    ConfigureDefaultCommands();
    ConfigureButtonBindings();
}

void RobotContainer::CheckDriverStationUpdate() {
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

void RobotContainer::ConfigureDefaultCommands() {
    Swerve.SetDefaultCommand(TeleopSwerveCommand{
        &Swerve,
        [this] { return -1 * DRIVER.GetLeftY(); },
        [this] { return -1 * DRIVER.GetLeftX(); },
        [this] { return -1 * DRIVER.GetRightX(); },
        [this] { return finalSpeedModifierEntry->GetDouble(0.0); }
    });
}

void RobotContainer::ConfigureButtonBindings() {

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // java code needs to work with new enums you alr know
    // add function in robot that gets value of auto, if that doesnt work then just get rid of this function and move entire thing
    //return pathplanner::PathPlannerAuto(Robot::AutoModeChooser.GetSelected().pathplannerName).ToPtr();
    return frc2::cmd::None();
}