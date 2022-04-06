package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.AutoAlignTurret;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.InnerMagIntake;
import frc.robot.commands.ShooterRPM;
import frc.robot.commands.TurnToAngle;
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
 * Autonomous that aligns limelight then executes a trajectory.
 */
public class P1_5B extends AutoBase {

    Intake intake;
    Shooter shooter;
    InnerMagazine innerMagazine;
    OuterMagazine outerMagazine;
    Turret turret;

    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public P1_5B(Swerve swerve, Shooter shooter, InnerMagazine innerMagazine,
        OuterMagazine outerMagazine, Intake intake, Turret turret, Vision vision) {
        super(swerve);
        this.shooter = shooter;
        this.innerMagazine = innerMagazine;
        this.outerMagazine = outerMagazine;
        this.intake = intake;
        this.turret = turret;

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("P1_3B_part1", 4, 2);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("P1_3B_part2", 2, 1);
        PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("P1_3B_part3", 2, 1);
        PathPlannerTrajectory trajectory4 = PathPlanner.loadPath("P1_3B_part4", 2, 1);
        PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory);
        PPSwerveControllerCommand autoDrive2 = baseSwerveCommand(trajectory2);
        PPSwerveControllerCommand autoDrive3 = baseSwerveCommand(trajectory3);
        PPSwerveControllerCommand autoDrive4 = baseSwerveCommand(trajectory4);
        PathPlannerState initialState = trajectory.getInitialState();

        SequentialCommandGroup part1 =
            new SequentialCommandGroup(autoDrive, new ZeroMotorsWaitCommand(swerve, 1))
                .deadlineWith(new StartEndCommand(() -> {
                    intake.intakeDeploy();
                    outerMagazine.magazineUp();
                }, () -> {
                    intake.intakeRetract();
                    outerMagazine.magazineStop();
                })).andThen(new FeedShooter(this.innerMagazine, this.outerMagazine, this.shooter)
                    .withTimeout(3));

        SequentialCommandGroup part2 =
            new TurnToAngle(swerve, 170, false).andThen(new TurnToAngle(swerve, 250, false))
                .andThen((autoDrive2.andThen(new ZeroMotorsWaitCommand(swerve, 3)
                    .withInterrupt(() -> innerMagazine.magSense.get())))
                        .deadlineWith(new StartEndCommand(() -> {
                            intake.intakeDeploy();
                            outerMagazine.magazineUp();
                        }, () -> {
                            intake.intakeRetract();
                            outerMagazine.magazineStop();
                        }), new InnerMagIntake(this.innerMagazine)))
                .andThen(new FeedShooter(this.innerMagazine, this.outerMagazine, this.shooter)
                    .withTimeout(2));

        ParallelDeadlineGroup part3 =
            (andThen(autoDrive3).andThen(new ZeroMotorsWaitCommand(swerve, 3)))
                .deadlineWith(new StartEndCommand(() -> {
                    intake.intakeDeploy();
                    outerMagazine.magazineUp();
                }, () -> {
                    intake.intakeRetract();
                    outerMagazine.magazineStop();
                }), new InnerMagIntake(this.innerMagazine));

        SequentialCommandGroup part4 = new TurnToAngle(swerve, 315, false).andThen(autoDrive4)
            .andThen(new ZeroMotorsWaitCommand(swerve))
            .andThen(new FeedShooter(this.innerMagazine, this.outerMagazine, this.shooter)
                .withTimeout(3));

        addCommands(
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            new InstantCommand(() -> this.turret.alignEnabled = true),
            new SequentialCommandGroup(part1.deadlineWith(new ShooterRPM(shooter, 2450 / 60)),
                part2.deadlineWith(new ShooterRPM(shooter, 2500 / 60)), part3,
                part4.deadlineWith(new ShooterRPM(shooter, 2500 / 60)))
                    .deadlineWith(new AutoAlignTurret(turret, vision)),
            new InstantCommand(() -> this.turret.alignEnabled = false));
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
