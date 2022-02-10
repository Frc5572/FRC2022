package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.subsystems.Swerve;

/**
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class TestAuto extends SequentialCommandGroup {
    Swerve swerve;

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public TestAuto(Swerve swerve) {
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

        PathPlannerTrajectory P2B2 = PathPlanner.loadPath("P2B2", 1, 1);

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController,
            0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand Auto = new PPSwerveControllerCommand(
            P2B2,
            swerve::getPose, Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
            swerve::setModuleStates, swerve);
        // PPSwerveControllerCommand firstHalfTraject = new PPSwerveControllerCommand(examplePath,
        // swerve::getPose, Constants.Swerve.swerveKinematics,
        // new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        // new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
        // swerve::setModuleStates, swerve);
        ZeroMotorsWaitCommand firstWait = new ZeroMotorsWaitCommand(swerve, 3);
        ZeroMotorsWaitCommand secondWait = new ZeroMotorsWaitCommand(swerve, .5);

        addCommands(new InstantCommand(() -> swerve.resetOdometry(P2B2.getInitialPose())),
            Auto);
    }
}
