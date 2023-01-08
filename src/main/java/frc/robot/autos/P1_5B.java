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

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("P1_3B_part1", 6, 3);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("P1_3B_part2", 6, 3);
        PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("P1_3B_part3", 8, 3);
        PathPlannerTrajectory trajectory4 = PathPlanner.loadPath("P1_3B_part4", 6, 3);
        PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory);
        PPSwerveControllerCommand autoDrive2 = baseSwerveCommand(trajectory2);
        PPSwerveControllerCommand autoDrive3 = baseSwerveCommand(trajectory3);
        PPSwerveControllerCommand autoDrive4 = baseSwerveCommand(trajectory4);
        PathPlannerState initialState = trajectory.getInitialState();

        SequentialCommandGroup part1 =
            new SequentialCommandGroup(autoDrive, new ZeroMotorsWaitCommand(swerve, .5))
                .deadlineWith(new StartEndCommand(() -> {
                    intake.intakeDeploy();
                    outerMagazine.magazineUp();
                }, () -> {
                    outerMagazine.magazineStop();
                })).andThen(new FeedShooter(this.innerMagazine, this.outerMagazine, this.shooter,
                    this.intake).withTimeout(1.5));

        SequentialCommandGroup part2 = new TurnToAngle(swerve, 250, false)
            .andThen((autoDrive2.andThen(new ZeroMotorsWaitCommand(swerve, 3)
                .until(() -> innerMagazine.magSense.get())))
                    .deadlineWith(new StartEndCommand(() -> {
                        intake.intakeDeploy();
                        outerMagazine.magazineUp();
                    }, () -> {
                        outerMagazine.magazineStop();
                    }), new InnerMagIntake(this.innerMagazine)))
            .andThen(
                new FeedShooter(this.innerMagazine, this.outerMagazine, this.shooter, this.intake)
                    .withTimeout(.7));

        ParallelDeadlineGroup part3 = (autoDrive3.andThen(new ZeroMotorsWaitCommand(swerve, 1.5)))
            .deadlineWith(new StartEndCommand(() -> {
                intake.intakeDeploy();
                outerMagazine.magazineUp();
            }, () -> {
                outerMagazine.magazineStop();
            }), new InnerMagIntake(this.innerMagazine));

        SequentialCommandGroup part4 =
            autoDrive4.andThen(new ZeroMotorsWaitCommand(swerve)).andThen(
                new FeedShooter(this.innerMagazine, this.outerMagazine, this.shooter, this.intake)
                    .withTimeout(5));

        addCommands(new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            new InstantCommand(() -> this.turret.alignEnabled = true),
            new SequentialCommandGroup(part1.deadlineWith(new ShooterRPM(shooter, 2450 / 60)),
                part2.deadlineWith(new ShooterRPM(shooter, 2500 / 60)), part3,
                part4.deadlineWith(new ShooterRPM(shooter, 3200 / 60)))
                    .deadlineWith(new AutoAlignTurret(turret, vision)),
            new InstantCommand(() -> this.turret.alignEnabled = false),
            new InstantCommand(() -> endCommand()));
    }

    public void endCommand() {
        innerMagazine.disable();
        outerMagazine.magazineStop();
        shooter.disableShooter();
        intake.intakeRetract();
        turret.alignEnabled = false;
    }
}
