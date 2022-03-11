package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
public class P1_3B extends AutoBase {

    Intake intake;
    Shooter shooter;
    Magazine magazine;

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public P1_3B(Swerve swerve, Shooter shooter, Magazine magazine, Intake intake, Turret turret,
        Vision vision) {
        super(swerve);
        this.shooter = shooter;
        this.magazine = magazine;
        this.intake = intake;

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("P1_3B_part1", 1, 1);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("P1_3B_part2", 1, 1);
        PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory);
        PPSwerveControllerCommand autoDrive2 = baseSwerveCommand(trajectory2);

        SequentialCommandGroup part1 =
            new SequentialCommandGroup(autoDrive, new ZeroMotorsWaitCommand(swerve),
                new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
                new InstantCommand(() -> magazine.enable()), new ZeroMotorsWaitCommand(swerve, 3),
                new InstantCommand(() -> magazine.disable()));

        SequentialCommandGroup part2 =
            new SequentialCommandGroup(autoDrive2, new ZeroMotorsWaitCommand(swerve),
                new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
                new InstantCommand(() -> magazine.enable()), new ZeroMotorsWaitCommand(swerve, 4),
                new InstantCommand(() -> magazine.disable()));

        addCommands(new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
            new InstantCommand(() -> turret.alignEnabled = true),
            new InstantCommand(() -> this.shooter.setSetpoint(4500 / 60)),
            new InstantCommand(() -> this.shooter.enable()),
            new InstantCommand(() -> intake.intakeDeploy()),
            new ParallelDeadlineGroup(new SequentialCommandGroup(part1, part2),
                new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(.6),
                    new InstantCommand(() -> turret.turretLeft()),
                    new AlignTurret(turret, vision)))));



        // addCommands(new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
        // new InstantCommand(() -> shooter.setSetpoint(Constants.ShooterPID.kShooterTargetRPS)),
        // new ParallelCommandGroup(new SequentialCommandGroup(
        // // new ZeroMotorsWaitCommand(swerve, 1),
        // new WaitUntilCommand(() -> shooter.atSetpoint()),
        // new InstantCommand(magazine::enable), new WaitCommand(3),
        // new InstantCommand(magazine::disable),
        // // new ZeroMotorsWaitCommand(swerve, 1),
        // new ParallelCommandGroup(new InstantCommand(() -> intake.intakeDeploy()), autoDrive,
        // new InstantCommand(() -> shooter.setSetpoint(4700 / 60)),
        // new FunctionalCommand(magazine::enable, () -> {
        // }, interrupted -> magazine.disable(), () -> magazine.magSense.get(), magazine)),
        // new ZeroMotorsWaitCommand(swerve), new WaitUntilCommand(() -> shooter.atSetpoint()),
        // new InstantCommand(magazine::enable), new WaitCommand(5)),
        // new SequentialCommandGroup(
        // new ParallelDeadlineGroup(new WaitCommand(.6),
        // new InstantCommand(() -> turret.turretLeft())),
        // new AlignTurret(turret, vision))));
    }

    @Override
    public void end(boolean interrupted) {
        magazine.disable();
        shooter.disable();
        intake.intakeRetract();
    }
}
