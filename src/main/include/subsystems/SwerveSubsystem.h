#pragma once

#include "frc2/command/SubsystemBase.h"
#include "frc/geometry/Translation2d.h"
#include "ctre/phoenix6/Pigeon2.hpp"
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include "Constants.h"
#include "subsystems/SwerveModule.h"

class SwerveSubsystem : public frc2::SubsystemBase {
public:
    SwerveSubsystem();

    void Drive(frc::Translation2d translation, double rotation, bool fieldRelative, bool isOpenLoop);
    frc::ChassisSpeeds GetSpeeds();
    void DriveRobotRelative(frc::ChassisSpeeds robotRelativeSpeeds);
    std::array<frc::SwerveModuleState, 4> GetStates();
    std::array<frc::SwerveModulePosition, 4> GetPositions();

    void ZeroGyro();
    frc::Rotation2d GetYaw();

    void Periodic() override;

private:
    ctre::phoenix6::hardware::Pigeon2 gyro;
    SwerveModule modules[4];
};