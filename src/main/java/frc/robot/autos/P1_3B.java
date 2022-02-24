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
public class P1_3B extends AutoBase {

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public P1_3B(Swerve swerve) {
        super(swerve);

        PathPlannerTrajectory P1_3B = PathPlanner.loadPath("P1_4B", 1, 1);
        PPSwerveControllerCommand testCommand = baseSwerveCommand(P1_3B);

        addCommands(new InstantCommand(() -> swerve.resetOdometry(P1_3B.getInitialPose())),
            testCommand);
    }
}
