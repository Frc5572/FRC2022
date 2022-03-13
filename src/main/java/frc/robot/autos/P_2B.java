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
import frc.robot.commands.AlignTurret;
import frc.robot.commands.ShooterRPM;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.modules.AutoBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OuterMagazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

/**
 * Autonomous that moves turret to align with target, excecutes trajectory to pick up ball directly
 * in front, then shoots both balls.
 */
public class P_2B extends AutoBase {

    Intake intake;
    Shooter shooter;
    InnerMagazine innerMagazine;
    OuterMagazine outerMagazine;

    /**
     * Autonomous that moves turret to align with target, excecutes trajectory to pick up ball
     * directly in front, then shoots both balls.
     *
     * @param swerve swerve subsystem
     * @param shooter shooter subsystem
     * @param innerMagazine inner magazine subsystem
     * @param outerMagazine outer magazine subsystem
     * @param intake intake subsystem
     * @param turret turret subsystem
     * @param vision vision subsystem
     */
    public P_2B(Swerve swerve, Shooter shooter, InnerMagazine innerMagazine,
        OuterMagazine outerMagazine, Intake intake, Turret turret, Vision vision) {
        super(swerve);
        this.shooter = shooter;
        this.outerMagazine = outerMagazine;
        this.innerMagazine = innerMagazine;
        this.intake = intake;
        addRequirements(shooter, innerMagazine, outerMagazine, intake, turret);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("P_2B", 1, 1);
        PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory);

        // ShooterRPM shooterCommand = new ShooterRPM(shooter, vision);
        addCommands(new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
            new InstantCommand(() -> turret.alignEnabled = true),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(new InstantCommand(() -> intake.intakeDeploy()),
                        autoDrive),
                    new ZeroMotorsWaitCommand(swerve, 1),
                    new InstantCommand(() -> intake.intakeRetract()),
                    new ZeroMotorsWaitCommand(swerve, 10)),
                new SequentialCommandGroup(new WaitCommand(2),
                    new WaitUntilCommand(() -> (shooter.getSetpoint() > 0 && shooter.atSetpoint())),
                    new InstantCommand(() -> {
                        innerMagazine.enable();
                        outerMagazine.enable();
                    })),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(new WaitCommand(.6),
                        new InstantCommand(() -> turret.turretLeft())),
                    new AlignTurret(turret, vision)),
                new ShooterRPM(shooter, 4500 / 60)));
    }

    @Override
    public void end(boolean interrupted) {
        innerMagazine.disable();
        outerMagazine.disable();
        shooter.disableShooter();
        intake.intakeRetract();
    }
}
