package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.AlignTurret;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.modules.AutoBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

/**
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class P_2B extends AutoBase {

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public P_2B(Swerve swerve, Shooter shooter, Magazine magazine, Intake intake, Turret turret,
        Vision vision) {
        super(swerve);
        addRequirements(shooter, magazine, intake, turret);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("P_2B", 1, 1);
        PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory);

        addCommands(new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
            new InstantCommand(() -> shooter.setSetpoint(Constants.ShooterPID.kShooterTargetRPS)),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(new WaitCommand(.6),
                        new InstantCommand(() -> turret.turretLeft())),
                    new AlignTurret(turret, vision)),
                new SequentialCommandGroup(new InstantCommand(() -> shooter.enable()),
                    new ZeroMotorsWaitCommand(swerve, 1),
                    new WaitUntilCommand(() -> shooter.atSetpoint()),
                    new InstantCommand(() -> magazine.enable()), new WaitCommand(3),
                    new ParallelCommandGroup(new InstantCommand(() -> intake.intakeDeploy()),
                        autoDrive, new InstantCommand(() -> shooter.setSetpoint(4700 / 60))),
                    new ZeroMotorsWaitCommand(swerve, 1),
                    new InstantCommand(() -> intake.intakeRetract()),
                    new ZeroMotorsWaitCommand(swerve, 3),
                    new InstantCommand(() -> shooter.disable()),
                    new InstantCommand(() -> magazine.disable()))

            ));

        //
        // new ParallelCommandGroup(new InstantCommand(() -> intake.intakeDeploy()), autoDrive,
        // new ParallelDeadlineGroup(new WaitCommand(.25),
        // new StartEndCommand(, () -> turret.turretStop(),
        // turret))),
        // new WaitCommand(2), new InstantCommand(() -> intake.intakeRetract()),
        // new InstantCommand(() -> shooter.enable()),
        // new WaitUntilCommand(() -> shooter.atSetpoint()),
        // new InstantCommand(() -> magazine.enable()), new WaitCommand(3),
        // new InstantCommand(() -> shooter.disable()),
        // new InstantCommand(() -> magazine.disable()));
    }
}
