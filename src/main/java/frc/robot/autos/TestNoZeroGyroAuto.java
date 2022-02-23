package frc.robot.autos;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Swerve;

/**
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class TestNoZeroGyroAuto extends AutoBase {
    Vision vision;
    Swerve swerve;

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     * @param vision vision subsystem
     */
    public TestNoZeroGyroAuto(Swerve swerve, Vision vision) {
        super(swerve);
        this.swerve = swerve;
        System.out.println("Limelight Auto !!");
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory firstHalfTrajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0.1, 0, new Rotation2d(0)),
                new Pose2d(0.1, 0.1, new Rotation2d(0))),
            config);

        SwerveControllerCommand firstHalfTraject = new SwerveControllerCommand(firstHalfTrajectory,
            swerve::getPose, Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
            swerve::setModuleStates, swerve);
        ZeroMotorsWaitCommand firstWait = new ZeroMotorsWaitCommand(swerve, 3);

        addCommands(firstHalfTraject, firstWait);
    }
}
