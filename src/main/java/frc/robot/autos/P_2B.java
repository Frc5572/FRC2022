package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.AutoAlignTurret;
import frc.robot.commands.FeedShooter;
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
 * Autonomous that moves turret to align with target, executes trajectory to pick up ball directly
 * in front, then shoots both balls.
 */
public class P_2B extends AutoBase {

    Intake intake;
    Shooter shooter;
    InnerMagazine innerMagazine;
    OuterMagazine outerMagazine;
    Turret turret;

    /**
     * Autonomous that moves turret to align with target, executes trajectory to pick up ball
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

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("P1_3B_part1", 6, 3);
        PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory);
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

        addCommands(new InstantCommand(() -> swerve.zeroGyro()),
            new InstantCommand(
                () -> swerve.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            new InstantCommand(() -> this.turret.alignEnabled = true),
            part1.deadlineWith(new ShooterRPM(shooter, 2450 / 60),
                new AutoAlignTurret(turret, vision)),
            new InstantCommand(() -> this.turret.alignEnabled = false),
            new InstantCommand(() -> endCommand()));
    }

    // new AutoAlignTurret(turret,vision))
    public void endCommand() {
        innerMagazine.disable();
        outerMagazine.magazineStop();
        shooter.disableShooter();
        intake.intakeRetract();
        turret.alignEnabled = false;
    }
}
