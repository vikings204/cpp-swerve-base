#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>
#include <units/math.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include "Constants.h"

class SwerveModule {
public:
    SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, frc::Rotation2d angleOffset);
    int moduleNumber;
    void SetDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop);
    void ResetToAbsolute();
    frc::Rotation2d GetAngle();
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();

private:
    frc::Rotation2d lastAngle;
    frc::Rotation2d angleOffset;
    double turningPDeg = 0;
    int turningPQuad = 0; // top left is 1 counterclockwise
    double turningTotalDeg = 0.0;
    rev::CANSparkMax driveMotor;
    rev::CANSparkMax angleMotor;
    rev::SparkRelativeEncoder driveEncoder;
    rev::SparkRelativeEncoder integratedAngleEncoder;
    rev::SparkAnalogSensor angleEncoder;
    rev::SparkPIDController driveController;
    rev::SparkPIDController angleController;
    frc::SimpleMotorFeedforward<units::meters> feedforward;

    void ConfigAngleMotor();
    void ConfigDriveMotor();
};