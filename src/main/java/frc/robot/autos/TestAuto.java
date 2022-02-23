package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class TestAuto extends AutoBase {
    Swerve swerve;

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public TestAuto(Swerve swerve) {
        super(swerve);
        this.swerve = swerve;

        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Rusinski's", 1, 1);
        PPSwerveControllerCommand testCommand = new PPSwerveControllerCommand(examplePath,
            swerve::getPose, Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
            swerve::setModuleStates, swerve);

        addCommands(new InstantCommand(() -> swerve.resetOdometry(examplePath.getInitialPose())),
            testCommand);
    }
}
