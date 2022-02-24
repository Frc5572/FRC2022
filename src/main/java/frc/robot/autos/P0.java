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
public class P0 extends AutoBase {
    Swerve swerve;

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public P0(Swerve swerve) {
        super(swerve);
        PathPlannerTrajectory P0 = PathPlanner.loadPath("P0", 1, 1);
        PPSwerveControllerCommand firstCommand = baseSwerveCommand(P0);

        addCommands(new InstantCommand(() -> swerve.resetOdometry(P0.getInitialPose())),
            firstCommand);

    }
}
