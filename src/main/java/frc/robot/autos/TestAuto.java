package frc.robot.autos;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(Swerve s_Swerve) {
        System.out.println("Example Auto !!");
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory MoveForward = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(Units.inchesToMeters(78) + Constants.Swerve.halfBaseWidth, 0),
                new Translation2d(Units.inchesToMeters(78), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(-82), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(-82), Units.inchesToMeters(0))),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
            config);

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController,
            0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(MoveForward,
            s_Swerve::getPose, Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
            s_Swerve::setModuleStates, s_Swerve);


        addCommands(new InstantCommand(() -> s_Swerve.resetOdometry(MoveForward.getInitialPose())),
            swerveControllerCommand);
    }
}
