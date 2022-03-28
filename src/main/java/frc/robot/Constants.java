package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.modules.swervedrive.SwerveModuleConstants;

/**
 * Constants file.
 */

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int driverID = 0;
    public static final int operatorID = 1;

    /**
     * Motor CAN id's.
     */
    public static final class Motors {
        // Climber Motors
        public static final int outsideClimberMotorRightId = 13;
        public static final int outsideClimberMotorLeftId = 16;
        public static final int insideClimberMotorRightId = 14;
        public static final int insideClimberMotorLeftId = 15;
        // Shooter Motors
        public static final int shooterID = 10;
        public static final int hoodMotorID = 17;
        public static final int shooterServoID = 10;
        // Intake Motors
        public static final int intakeMotorNum = 9;
        // Inner Magazine Motor
        public static final int innerMagazineMotorID = 11;
        // Outer Magazine Motor
        public static final int outerMagazineMotorID = 18;
        // Turret Motors
        public static final int turretMotorID = 12;
    }

    /**
     * Pneumatics CAN id constants.
     */
    public static final class Pneumatics {
        // Climber pneumatics constants.
        public static final int climberOutsideForwardChannel = 2;
        public static final int climberOutsideReverseChannel = 4;
        public static final int climberInsideChannel = 0;
        public static final int climberInsideChannel2 = 1;
        // Intake constants
        public static final int intakeFowardChannel = 3;
    }

    public static final int magazineSensor = 9;

    /**
     * Vision constants for limelight calculations.
     */
    public static final class VisionConstants {
        public static final double deadPocket = 0.05;
        public static final double limelightHeight = 24;
        public static final double targetHeight = 103;
        public static final double limelightAngle = 30.5;
    }

    /**
     * Creates class and constants for Swerve drive.
     */
    public static final class Swerve {
        public static final edu.wpi.first.wpilibj.SPI.Port navXID =
            edu.wpi.first.wpilibj.SPI.Port.kMXP;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23);
        public static final double wheelBase = Units.inchesToMeters(23);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final boolean isFieldRelative = true;
        public static final boolean isOpenLoop = false;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.25;

        public static final double driveGearRatio = (8.14 / 1.0); // 6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKS = (0.667 / 12);
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4; // meters per second
        public static final double maxAngularVelocity = 4;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */

        /**
         * Front Left Module - Module 0.
         */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final double angleOffset = 191.25;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /**
         * Front Right Module - Module 1.
         */

        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 2;
            public static final double angleOffset = 290.127;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Back Left Module - Module 2.
         */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final double angleOffset = 122.344;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Back Right Module - Module 3.
         */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 3;
            public static final double angleOffset = 248.994;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    /**
     * Autonomous constants for swerve bot.
     */
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

    /**
     * Hood constants.
     */

    public static final class HoodConstants {
        public static final int hoodCANCoderID = 5;
        public static final int hoodServoID = 9;
        public static final int minAngle = 40;
        public static final int maxAngle = 55;
        public static final double servoSpeed = 0.2;
        public static final double maxPosition = 225.527;
        public static final double minPosition = 51.68;
        public static final boolean hoodCanCoderInvert = false;
    }

    /**
     * Constants for Shooter PID
     */

    public static final class ShooterPID {
        public static final double kP = 0.13744;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kShooterTargetRPS = 4100 / 60; // IN RPS NOT RPM
        public static final double kShooterToleranceRPS = 1; // IN RPS NOT RPM

        public static final int kUnitsPerRevolution = 2048;
        public static final double kSVolts = 0.63035;
        public static final double kVVoltSecondsPerRotation = 0.10877;
    }

    /**
     * Constants for Shooter Roller PID
     */

    public static final class ShooterRollerPID {
        public static final double kP = 0.11219;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kShooterRollerTargetRPS = 4100 / 60; // IN RPS NOT RPM
        public static final double kShooterRollerToleranceRPS = 2; // IN RPS NOT RPM

        public static final int kUnitsPerRevolution = 42;
        public static final double kSVolts = 0.19057;
        public static final double kVVoltSecondsPerRotation = 0.12795;
    }

    /**
     * Constants for Inner Magazine PID
     */
    public static final class InnerMagazinePID {
        public static final double kP = 0.17032;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kInnerMagazineTargetRPS = 600 / 60;
        public static final double kInnerMagazineToleranceRPS = 1; // IN RPS NOT RPM

        public static final int kUnitsPerRevolution = 2048;
        public static final double kSVolts = 0.87948;
        public static final double kVVoltSecondsPerRotation = 0.10969;
    }

    /**
     * Constants for Outer Magazine PID
     */
    public static final class OuterMagazinePID {
        public static final double kP = 0.012875;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kOuterMagazineTargetRPS = 600 / 60;
        public static final double kOuterMagazineToleranceRPS = 1; // IN RPS NOT RPM

        public static final int kUnitsPerRevolution = 42;
        public static final double kSVolts = 0.12998;
        public static final double kVVoltSecondsPerRotation = 0.015757;
    }
}
