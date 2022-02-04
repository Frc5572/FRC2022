package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.modules.swerveDrive.SwerveModuleConstants;

/**
 * Constants file.
 */

public final class Constants {
    public static final double stickDeadband = 0.1;

    /**
     * Creates class and constants for Swerve drive.
     */
    public static final class Swerve {
        public static final edu.wpi.first.wpilibj.SPI.Port navXID =
            edu.wpi.first.wpilibj.SPI.Port.kMXP;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(27);
        public static final double wheelBase = Units.inchesToMeters(27);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final boolean isFieldRelative = true;
        public static final boolean isOpenLoop = false;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

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
        public static final double driveKS = (0.667 / 12); // divide by 12 to convert from volts to
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 2; // meters per second
        public static final double maxAngularVelocity = 2;

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
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 3;
            public static final double angleOffset = 69.69;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /**
         * Front Left Module - Module 1.
         */

        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 2;
            public static final double angleOffset = 290.91;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Front Left Module - Module 2.
         */
        public static final class Mod2 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final double angleOffset = 122.43;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /**
         * Front Left Module - Module 3.
         */
        public static final class Mod3 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final double angleOffset = 11.95;
            public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    /**
     * Autonomous constants for swerve bot.
     */
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

    /**
     * Pneumatics CAN id cosntants.
     */
    public static final class Pneumatics {
        public static final int pcm1 = 0;
        public static final int pcm2 = 0;

        /**
         * Climber pneumatics constants.
         */
        public static final class Climber {
            public static final int outsideChannel = 0;
            public static final int insideChannel = 0;
        }

        /**
         * Intake constants
         */
        public static final class IntakeConstants {
            public static final int intakeMotorNum = 0;
            public static final int intakeModule = 0;
            public static final int intakeFowardChannel = 0;
        }

    }

    /**
     * Motor CAN id's.
     */
    public static final class Motors {
        public static final int outsideClimberMotor1Id = 0;
        public static final int outsideClimberMotor2Id = 0;
        public static final int insideClimberMotor1Id = 0;
        public static final int insideClimberMotor2Id = 0;
        public static final int shooterID = 0;
        public static final int shooterServoID = 0;
    }

    /**
     * Vision constants for limelight calculations.
     */

    public static final class VisionConstants {
        public static final double deadPocket = 0.2;
    }

}
