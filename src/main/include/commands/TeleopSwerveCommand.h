#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>

#include "subsystems/SwerveSubsystem.h"

class TeleopSwerveCommand : public frc2::CommandHelper<frc2::Command, TeleopSwerveCommand> {
public:
    TeleopSwerveCommand(
        SwerveSubsystem* subsystem,
        std::function<double()> getTranslation,
        std::function<double()> getStrafe,
        std::function<double()> getRotation,
        std::function<double()> getFinalSpeedModifier
    );

    void Execute() override;
private:
    SwerveSubsystem* Swerve;
    std::function<double()> getTranslation;
    std::function<double()> getStrafe;
    std::function<double()> getRotation;
    std::function<double()> getFinalSpeedModifier;

    frc::SlewRateLimiter<units::meters> translationLimiter{3.0_m / 1_s};
    frc::SlewRateLimiter<units::meters> strafeLimiter{3.0_m / 1_s};
    frc::SlewRateLimiter<units::meters> rotationLimiter{3.0_m / 1_s};
};