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
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class P1_3B extends AutoBase {

    Intake intake;
    Shooter shooter;
    InnerMagazine innerMagazine;
    OuterMagazine outerMagazine;
    Turret turret;

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public P1_3B(Swerve swerve, Shooter shooter, InnerMagazine innerMagazine,
        OuterMagazine outerMagazine, Intake intake, Turret turret, Vision vision) {
        super(swerve);
        this.shooter = shooter;
        this.innerMagazine = innerMagazine;
        this.outerMagazine = outerMagazine;
        this.intake = intake;
        this.turret = turret;

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("P1_3B_part1", 4, 2);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("P1_3B_part2", 2, 1);
        PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory);
        PPSwerveControllerCommand autoDrive2 = baseSwerveCommand(trajectory2);
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


        ParallelDeadlineGroup part2 = new ParallelDeadlineGroup(
            new SequentialCommandGroup(new InstantCommand(() -> intake.intakeDeploy()),
                new InstantCommand(() -> outerMagazine.magazineUp()), autoDrive2,
                new ZeroMotorsWaitCommand(swerve), new WaitCommand(1),
                new InstantCommand(() -> intake.intakeRetract()),
                // new InstantCommand(() -> outerMagazine.magazineStop()),
                new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
                new WaitCommand(1),
                new ParallelDeadlineGroup(new ZeroMotorsWaitCommand(swerve, 3),
                    new MagazineRPM(this.shooter, this.innerMagazine))),
            new ShooterRPM(shooter, 4000 / 60));

        addCommands(
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            new InstantCommand(() -> turret.alignEnabled = true),
            new InstantCommand(() -> intake.intakeDeploy()),
            new InstantCommand(() -> outerMagazine.magazineUp(.6)), new ParallelDeadlineGroup(
                new SequentialCommandGroup(part1, part2), new AutoAlignTurret(turret, vision)));
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
