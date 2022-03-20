package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoAlignTurret;
import frc.robot.commands.MagazineRPM;
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
    Turret turret;

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
        this.turret = turret;
        addRequirements(shooter, innerMagazine, outerMagazine, intake, turret);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("P1_3B_part1", 4, 2);
        PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory);
        PathPlannerState initialState = trajectory.getInitialState();

        ParallelDeadlineGroup part1 = new ParallelDeadlineGroup(
            new SequentialCommandGroup(autoDrive, new ZeroMotorsWaitCommand(swerve),
                new WaitCommand(.5), new InstantCommand(() -> intake.intakeRetract()),
                new WaitCommand(.3), new InstantCommand(() -> intake.intakeDeploy()),
                new WaitCommand(.3), new InstantCommand(() -> intake.intakeRetract()),
                new InstantCommand(() -> outerMagazine.magazineStop()),
                new PrintCommand("Shooter is being weird"),
                new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
                new WaitCommand(1),
                new ParallelDeadlineGroup(new ZeroMotorsWaitCommand(swerve, 3),
                    new ParallelCommandGroup(new MagazineRPM(this.shooter, this.innerMagazine),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(() -> !this.innerMagazine.magSense.get()
                                && this.shooter.getSetpoint() > 0 && this.shooter.atSetpoint()),
                            new WaitCommand(3),
                            new InstantCommand(() -> this.outerMagazine.magazineUp(.6)))))),
            new ShooterRPM(shooter, 3700 / 60));

        addCommands(
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            new InstantCommand(() -> turret.alignEnabled = true),
            new InstantCommand(() -> intake.intakeDeploy()),
            new InstantCommand(() -> outerMagazine.magazineUp(.4)),
            new ParallelDeadlineGroup(part1, new AutoAlignTurret(turret, vision)));
        // new ParallelDeadlineGroup(
        // new SequentialCommandGroup(new ParallelCommandGroup(autoDrive),
        // new ZeroMotorsWaitCommand(swerve, 1),
        // new InstantCommand(() -> intake.intakeRetract()),
        // new ZeroMotorsWaitCommand(swerve, 10)),
        // new SequentialCommandGroup(new WaitCommand(2),
        // new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
        // new ParallelCommandGroup(new MagazineRPM(this.shooter, this.innerMagazine),
        // new SequentialCommandGroup(
        // new WaitUntilCommand(() -> !this.innerMagazine.magSense.get()
        // && this.shooter.getSetpoint() > 0 && this.shooter.atSetpoint()),
        // new WaitCommand(.5),
        // new InstantCommand(() -> this.outerMagazine.magazineUp(.4))))),
        // new AutoAlignTurret(turret, vision), new ShooterRPM(shooter, 4700 / 60)));
    }

    @Override
    public void end(boolean interrupted) {
        innerMagazine.disable();
        outerMagazine.magazineStop();
        shooter.disableShooter();
        intake.intakeRetract();
        turret.alignEnabled = false;
    }
}
