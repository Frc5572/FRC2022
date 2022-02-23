package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class P0 extends SequentialCommandGroup {
    Swerve swerve;

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public P0(Swerve swerve) {
        this.swerve = swerve;
        System.out.println("Test Auto !!");
        // TrajectoryConfig config =
        // new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // .setKinematics(Constants.Swerve.swerveKinematics);

        // Trajectory firstHalfTrajectory = TrajectoryGenerator.generateTrajectory(

        // List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, new Rotation2d(0)),
        // new Pose2d(1, 1, new Rotation2d(0))),
        // config);

        PathPlannerTrajectory P0 = PathPlanner.loadPath("P0", 1, 1);

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController,
            0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PPSwerveControllerCommand firstHalfTraject =
            new PPSwerveControllerCommand(P0, swerve::getPose, Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                swerve::setModuleStates, swerve);

        addCommands(new InstantCommand(() -> swerve.resetOdometry(P0.getInitialPose())),
            firstHalfTraject);
    }
}
