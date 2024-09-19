#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/acceleration.h>
#include <math.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <rev/CANSparkBase.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

namespace Constants {
    namespace Controller {
        const int DRIVER_PORT = 1;
        const int OPERATOR_PORT = 2;
        //const float DEADBAND = 0.1;
        const double DEADBAND = 0.1;

        // will eventually have keybinds and stuff
    }

    namespace Shooter {
        const rev::CANSparkBase::IdleMode IDLE_MODE = rev::CANSparkBase::IdleMode::kBrake;
        const bool SHOOTER_INVERT = true;
        const bool BUMP_INVERT = false;

        const int SHOOTER_MOTOR1_ID = 41;
        const int SHOOTER_MOTOR_2_ID = 42;
        const int INTAKE_MOTOR_ID = 43;

        const int CURRENT_LIMIT = 40;
        const float VOLTAGE_COMP = 16.0;

        const float SPEAKER_SPEED = 1.0;
        const float AMP_SPEED = .080;
        const float INTAKE_SPEED = 1.0;

        const int INTAKE_SENSOR_THRESHOLD = 200;
    }

    namespace Swerve {
        const float FAST_SPEED_MULTIPLIER = 1;
        const float NORMAL_SPEED_MULTIPLIER = 1;//.8;
        const float SLOW_SPEED_MULTIPLIER = .6;

        const float ANGLE_PID_FF = 0.0;
        const float DRIVE_PID_P = 1.0;
        const float DRIVE_PID_I = 0.0;
        const float DRIVE_PID_D = 0.01;
        const float DRIVE_PID_FF = 0.0;
        const units::volt_t DRIVE_FF_S {0.667};
        const units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second>>> DRIVE_FF_V {2.44};
        const units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second_squared>>> DRIVE_FF_A {0.27};
        // static constexpr auto DRIVE_FF_A = 0.27_V / 1_mps;
        const float ANGLE_PID_P = 0.01;
        const float ANGLE_PID_I = 0.0;
        const float ANGLE_PID_D = 0.0;

        /* Drivetrain Constants */
        const units::meter_t TRACK_WIDTH {23_in}; // same as wheelbase because it is a square
        const units::meter_t WHEEL_BASE {23_in};
        const units::meter_t WHEEL_DIAMETER {4.0_in};
        const units::meter_t WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

        const float DRIVE_GEAR_RATIO = 8.14;
        const float ANGLE_GEAR_RATIO = (150.0 / 7.0);
        const frc::SwerveDriveKinematics<4> SWERVE_KINEMATICS {
                frc::Translation2d{-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0},
                frc::Translation2d{WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0},
                frc::Translation2d{-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0},
                frc::Translation2d{WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0}
        };

        /* Drive Motor Conversion Factors */
        const float DRIVE_POSITION_CONVERSION_FACTOR = (WHEEL_DIAMETER.value() * M_PI) / DRIVE_GEAR_RATIO;
        const float DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60.0;
        const float ANGLE_POSITION_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;

        /* Swerve Voltage Compensation */
        const float VOLTAGE_COMPENSATION = 12.0;

        /* Swerve Current Limiting */
        const int DRIVE_CURRENT_LIMIT = 40;//30;
        const int ANGLE_CURRENT_LIMIT = 10;//5;

        /* Swerve Profiling Values */
        const units::meters_per_second_t MAX_SPEED {4.5}; // meters per second
        const units::radians_per_second_t MAX_ANGULAR_VELOCITY {8}; // radians per second

        /* Neutral Modes */
        const rev::CANSparkBase::IdleMode DRIVE_IDLE_MODE = rev::CANSparkBase::IdleMode::kBrake;
        const rev::CANSparkBase::IdleMode ANGLE_IDLE_MODE = rev::CANSparkBase::IdleMode::kBrake;

        /* Motor Inverts */
        const bool DRIVE_INVERT = true;
        const bool ANGLE_INVERT = true;

        const bool GYRO_INVERT = true; // Always ensure Gyro is CCW+ CW-
        const int PIGEON2_ID = 9;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        namespace Mod0 {
            const int DRIVE_MOTOR_ID = 11;
            const int ANGLE_MOTOR_ID = 21;
            const frc::Rotation2d ANGLE_OFFSET {142.75_deg};
        };

        /* Front Right Module - Module 1 */
        namespace Mod1 {
            const int DRIVE_MOTOR_ID = 12;
            const int ANGLE_MOTOR_ID = 22;
            const frc::Rotation2d ANGLE_OFFSET {115.05_deg};
        };

        /* Back Left Module - Module 2 */
        namespace Mod2 {
            const int DRIVE_MOTOR_ID = 10;
            const int ANGLE_MOTOR_ID = 20;
            const frc::Rotation2d ANGLE_OFFSET {258.66_deg};
        };

        /* Back Right Module - Module 3 */
        namespace Mod3 {
            const int DRIVE_MOTOR_ID = 13;
            const int ANGLE_MOTOR_ID = 23;
            const frc::Rotation2d ANGLE_OFFSET {77.9_deg};
        };
    }

    namespace Auto {
        const pathplanner::HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG {
                pathplanner::PIDConstants(2.0, 0, .04), // Translation constants
                pathplanner::PIDConstants(3.0, 0.1, .00), // Rotation constants
                Swerve::MAX_SPEED,
                .495_m, // Drive base radius (distance from center to the furthest module)
                pathplanner::ReplanningConfig()
        };
    }

    namespace LinearActuator {
        const int MOTOR_CAN_ID = 31;
        const int CURRENT_LIMIT = 10;

        const float PID_P = 1.0;
        const float PID_I = 0.0;
        const float PID_D = 0.0;
        const float PID_FF = 0.0;
    }
}