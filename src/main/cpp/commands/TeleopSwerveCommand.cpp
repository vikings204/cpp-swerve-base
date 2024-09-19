#include "frc/MathUtil.h"

#include "commands/TeleopSwerveCommand.h"
#include "Constants.h"

using namespace Constants::Swerve;
using namespace Constants::Controller;

TeleopSwerveCommand::TeleopSwerveCommand(
    SwerveSubsystem* subsystem,
    std::function<double()> translation,
    std::function<double()> strafe,
    std::function<double()> rotation,
    std::function<double()> finalSpeedModifier
) :
    Swerve{subsystem},
    getTranslation{translation},
    getStrafe{strafe},
    getRotation{rotation},
    getFinalSpeedModifier{finalSpeedModifier}
{
    AddRequirements(Swerve);
}

void TeleopSwerveCommand::Execute() {
    auto translationVal = translationLimiter.Calculate(units::meter_t{frc::ApplyDeadband<double>(getTranslation(), DEADBAND)});
    auto strafeVal = strafeLimiter.Calculate(units::meter_t{frc::ApplyDeadband<double>(getStrafe(), DEADBAND)});
    auto rotationVal = rotationLimiter.Calculate(units::meter_t{frc::ApplyDeadband<double>(getRotation(), DEADBAND)});

    Swerve->Drive(frc::Translation2d{translationVal, strafeVal}.operator*(MAX_SPEED.value() * getFinalSpeedModifier()), rotationVal.value()*(MAX_ANGULAR_VELOCITY.value() * getFinalSpeedModifier()), false, true);
}