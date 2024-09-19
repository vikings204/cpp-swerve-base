#include "subsystems/SwerveSubsystem.h"

#include <frc/shuffleboard/ShuffleBoard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>

using namespace Constants::Swerve;

SwerveSubsystem::SwerveSubsystem() :
gyro{PIGEON2_ID, "rio"},
modules{
    SwerveModule(0, Mod0::DRIVE_MOTOR_ID, Mod0::ANGLE_MOTOR_ID, Mod0::ANGLE_OFFSET),
    SwerveModule(1, Mod1::DRIVE_MOTOR_ID, Mod1::ANGLE_MOTOR_ID, Mod1::ANGLE_OFFSET),
    SwerveModule(2, Mod2::DRIVE_MOTOR_ID, Mod2::ANGLE_MOTOR_ID, Mod2::ANGLE_OFFSET),
    SwerveModule(3, Mod3::DRIVE_MOTOR_ID, Mod3::ANGLE_MOTOR_ID, Mod3::ANGLE_OFFSET)
}
{
    gyro.GetConfigurator().Apply(ctre::phoenix6::configs::Pigeon2Configuration{});
    ZeroGyro();

    for (int i = 0; i < 4; i++) {
        SwerveModule* mod = &modules[i];
        frc::Shuffleboard::GetTab("swerve").AddNumber("position: module " + std::to_string(mod->moduleNumber), [mod] {return mod->GetPosition().distance.value();});
        frc::Shuffleboard::GetTab("swerve").AddNumber("angle: module " + std::to_string(mod->moduleNumber), [mod] {return mod->GetPosition().angle.Degrees().value();}).WithWidget(frc::BuiltInWidgets::kDial).WithProperties({{"min", nt::Value::MakeInteger(-180)},{"max", nt::Value::MakeInteger(180)}});
        frc::Shuffleboard::GetTab("swerve").AddNumber("integrated angle: module " + std::to_string(mod->moduleNumber), [mod] {return mod->GetState().angle.Degrees().value();}).WithWidget(frc::BuiltInWidgets::kDial).WithProperties({{"min", nt::Value::MakeInteger(-180)},{"max", nt::Value::MakeInteger(180)}});
        frc::Shuffleboard::GetTab("swerve").AddNumber("velocity: module " + std::to_string(mod->moduleNumber), [mod] {return mod->GetState().speed.value();});
    }
    frc::Shuffleboard::GetTab("main").AddNumber("gyro angle", [this] {return GetYaw().Degrees().value();});
}

void SwerveSubsystem::Drive(frc::Translation2d translation, double rotation, bool fieldRelative, bool isOpenLoop) {
    wpi::array<frc::SwerveModuleState, 4U> swerveModuleStates = SWERVE_KINEMATICS.ToSwerveModuleStates(fieldRelative
        ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(translation.X() / 1_s, translation.Y() / 1_s, units::radians_per_second_t{rotation}, GetYaw())
        : frc::ChassisSpeeds{translation.X() / 1_s, translation.Y() / 1_s, units::radians_per_second_t{rotation}}
    );
    frc::SwerveDriveKinematics<4U>::DesaturateWheelSpeeds(&swerveModuleStates, MAX_SPEED);

    for (int i = 0; i < 4; i++) {
        modules[i].SetDesiredState(swerveModuleStates[i], isOpenLoop);
    }
}

frc::ChassisSpeeds SwerveSubsystem::GetSpeeds() {
    return SWERVE_KINEMATICS.ToChassisSpeeds(GetStates());
}

void SwerveSubsystem::DriveRobotRelative(frc::ChassisSpeeds robotRelativeSpeeds) {
    frc::ChassisSpeeds targetSpeeds = frc::ChassisSpeeds::Discretize(robotRelativeSpeeds, units::second_t{0.02});
    wpi::array<frc::SwerveModuleState, 4U> targetStates = SWERVE_KINEMATICS.ToSwerveModuleStates(targetSpeeds);
    SWERVE_KINEMATICS.DesaturateWheelSpeeds(&targetStates, MAX_SPEED);

    for (int i = 0; i < 4; i++) {
        modules[i].SetDesiredState(targetStates[i], false);
    }
}

std::array<frc::SwerveModuleState, 4> SwerveSubsystem::GetStates() {
    std::array<frc::SwerveModuleState, 4> states;
    for (int i = 0; i < 4; i++) {
        states[i] = modules[i].GetState();
    }
    return states;
}

std::array<frc::SwerveModulePosition, 4> SwerveSubsystem::GetPositions() {
    std::array<frc::SwerveModulePosition, 4> positions;
    for (int i = 0; i < 4; i++) {
        positions[i] = modules[i].GetPosition();
    }
    return positions;
}

void SwerveSubsystem::ZeroGyro() {
    gyro.SetYaw(0_deg);
}

frc::Rotation2d SwerveSubsystem::GetYaw() {
    return GYRO_INVERT ? frc::Rotation2d{units::degree_t{360 - gyro.GetAngle()}} : frc::Rotation2d{units::degree_t{gyro.GetAngle()}};
}

void SwerveSubsystem::Periodic() {
    // nothing lmao
}