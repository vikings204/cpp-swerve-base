#include <iostream>

#include <frc/shuffleboard/ShuffleBoard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>

#include "subsystems/SwerveModule.h"

using namespace Constants::Swerve;

SwerveModule::SwerveModule(int _moduleNumber, int _driveMotorID, int _angleMotorID, frc::Rotation2d _angleOffset) :
angleMotor{_angleMotorID, rev::CANSparkMax::MotorType::kBrushless},
driveMotor{_driveMotorID, rev::CANSparkMax::MotorType::kBrushless},
driveEncoder{driveMotor.GetEncoder()},
integratedAngleEncoder{angleMotor.GetEncoder()},
driveController{driveMotor.GetPIDController()},
angleController{angleMotor.GetPIDController()},
feedforward{DRIVE_FF_S, DRIVE_FF_V, DRIVE_FF_A}
{
    moduleNumber = _moduleNumber;
    angleOffset = _angleOffset;

    ConfigAngleMotor();
    ConfigDriveMotor();

    lastAngle = GetState().angle;

    frc::Shuffleboard::GetTab("swervetest").AddNumber("angleEncoderCurrent Reading " + moduleNumber, [this] {return integratedAngleEncoder.GetPosition();}).WithWidget(frc::BuiltInWidgets::kDial).WithProperties({
        {"min", nt::Value::MakeDouble(0)},
        {"max", nt::Value::MakeDouble(360)}
    });
    frc::Shuffleboard::GetTab("swervetest").AddNumber("angleMotorAbsEncoder Reading " + moduleNumber, [this] {return angleMotor.GetAnalog(rev::SparkAnalogSensor::Mode::kAbsolute).GetVoltage();});
}

void SwerveModule::ConfigAngleMotor() {
    angleMotor.RestoreFactoryDefaults();

    angleMotor.SetSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
    angleMotor.SetInverted(ANGLE_INVERT);
    angleMotor.SetIdleMode(ANGLE_IDLE_MODE);
    angleMotor.EnableVoltageCompensation(VOLTAGE_COMPENSATION);

    integratedAngleEncoder.SetPositionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);

    angleController.SetP(ANGLE_PID_P);
    angleController.SetI(ANGLE_PID_I);
    angleController.SetD(ANGLE_PID_D);
    angleController.SetFF(ANGLE_PID_FF);

    // ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.BurnFlash();
    // Timer.delay(2);
    ResetToAbsolute();
}

void SwerveModule::ConfigDriveMotor() {
    driveMotor.RestoreFactoryDefaults();
    
    driveMotor.SetSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
    driveMotor.SetInverted(DRIVE_INVERT);
    driveMotor.SetIdleMode(DRIVE_IDLE_MODE);
    driveMotor.EnableVoltageCompensation(VOLTAGE_COMPENSATION);

    driveEncoder.SetVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION_FACTOR);
    driveEncoder.SetPositionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR);

    driveController.SetP(DRIVE_PID_P);
    driveController.SetI(DRIVE_PID_I);
    driveController.SetD(DRIVE_PID_D);
    driveController.SetFF(DRIVE_PID_FF);
    
    // ReduceCANUsage.SparkMax.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.BurnFlash();
    driveEncoder.SetPosition(0.0);
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop) {
    desiredState = frc::SwerveModuleState::Optimize(desiredState, frc::Rotation2d(units::degree_t{integratedAngleEncoder.GetPosition()}));

    frc::Rotation2d angle = (units::math::abs(desiredState.speed) <= (MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle;
    angleController.SetReference(angle.Degrees().value(), rev::CANSparkMax::ControlType::kPosition);
    //SetAngle(desiredState);

    if (isOpenLoop) {
            driveMotor.Set(desiredState.speed / MAX_SPEED);
        } else {
            driveController.SetReference(
                    desiredState.speed.value(),
                    rev::CANSparkBase::ControlType::kVelocity,
                    0,
                    feedforward.Calculate(desiredState.speed).value());
        }
    //SetSpeed(desiredState, isOpenLoop);
}

void SwerveModule::ResetToAbsolute() {
    float absolutePosition = angleMotor.GetAnalog(rev::SparkAnalogSensor::Mode::kAbsolute).GetPosition() - angleOffset.Degrees().value();
    std::cout << "Encoder" + std::to_string(moduleNumber) + "Absolute Position: " + std::to_string(absolutePosition);    
    std::cout << "Encoder " + std::to_string(moduleNumber) + " is Zerod";
    integratedAngleEncoder.SetPosition(0.0);
}

frc::Rotation2d SwerveModule::GetAngle() {
    return frc::Rotation2d{units::degree_t{integratedAngleEncoder.GetPosition()}};
}

frc::SwerveModuleState SwerveModule::GetState() {
    return frc::SwerveModuleState{units::meters_per_second_t{driveEncoder.GetVelocity()}, GetAngle()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    return frc::SwerveModulePosition{units::length::meter_t{driveEncoder.GetPosition()}, GetAngle()};
}