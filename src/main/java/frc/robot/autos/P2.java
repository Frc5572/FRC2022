package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.modules.AutoBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class P2 extends AutoBase {
    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public P2(Swerve swerve, Shooter shooter, Magazine magazine, Intake intake) {
        super(swerve);

        PathPlannerTrajectory P2 = PathPlanner.loadPath("P2_B2", 1, 1);
        PPSwerveControllerCommand testCommand = baseSwerveCommand(P2);

        addCommands(new ParallelCommandGroup(
            new InstantCommand(() -> swerve.resetOdometry(P2.getInitialPose())),
            new InstantCommand(() -> intake.intakeDeploy()), testCommand, new WaitCommand(2),
            new InstantCommand(() -> intake.intakeRetract()),
            new InstantCommand(() -> shooter.enable())
                .andThen(new WaitUntilCommand(() -> shooter.atSetpoint()),
                    new InstantCommand(() -> magazine.enable()), new WaitCommand(3))
                .andThen(new InstantCommand(() -> shooter.disable()),
                    new InstantCommand(() -> magazine.disable()))));
    }
}
