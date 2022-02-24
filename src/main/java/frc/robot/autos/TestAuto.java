package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.modules.AutoBase;
import frc.robot.subsystems.Swerve;

/**
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class TestAuto extends AutoBase {

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public TestAuto(Swerve swerve) {
        super(swerve);

        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Rusinski's", 1, 1);
        PPSwerveControllerCommand testCommand = baseSwerveCommand(examplePath);

        addCommands(new InstantCommand(() -> swerve.resetOdometry(examplePath.getInitialPose())),
            testCommand);
    }
}
