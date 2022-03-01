package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.modules.AutoBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class P1_4B extends AutoBase {
    public P1_4B(Swerve swerve, Shooter shooter, Magazine magazine, Intake intake, Turret turret,
        Vision vision) {
        super(swerve);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("P1_4B", 1, 1);
        PathPlannerTrajectory trajectory_pt1 = PathPlanner.loadPath("P1_4B pt1", 1, 1);
        PathPlannerTrajectory trajectory_pt2 = PathPlanner.loadPath("P1_4B pt2", 1, 1);
        PathPlannerTrajectory trajectory_pt3 = PathPlanner.loadPath("P1_4B pt3", 1, 1);
        PathPlannerTrajectory trajectory_pt4 = PathPlanner.loadPath("P1_4B pt4", 1, 1);


        // PPSwerveControllerCommand autoDrive = baseSwerveCommand(trajectory);
        PPSwerveControllerCommand autoDrive_pt1 = baseSwerveCommand(trajectory_pt1);
        PPSwerveControllerCommand autoDrive_pt2 = baseSwerveCommand(trajectory_pt2);
        PPSwerveControllerCommand autoDrive_pt3 = baseSwerveCommand(trajectory_pt3);
        PPSwerveControllerCommand autoDrive_pt4 = baseSwerveCommand(trajectory_pt4);


        addCommands(new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
            new InstantCommand(() -> shooter.setSetpoint(Constants.ShooterPID.kShooterTargetRPS)),
            new ParallelCommandGroup(new SequentialCommandGroup(
                new ParallelDeadlineGroup(new WaitCommand(.6),
                    new InstantCommand(() -> turret.turretLeft())),
                new InstantCommand(shooter::enable), new FunctionalCommand(() -> {
                }, () -> turret.turretSet(vision.getTargetFound() ? vision.getAimValue() : 0),
                    interupt -> {
                    }, () -> false, turret)),

                new SequentialCommandGroup(
                    // new ZeroMotorsWaitCommand(swerve, 1),
                    new WaitUntilCommand(() -> shooter.atSetpoint()),
                    new InstantCommand(magazine::enable), new WaitCommand(3),
                    new InstantCommand(magazine::disable),
                    // new ZeroMotorsWaitCommand(swerve, 1),
                    new ParallelCommandGroup(new InstantCommand(() -> intake.intakeDeploy()),
                        autoDrive_pt1, new InstantCommand(() -> shooter.setSetpoint(4700 / 60)),
                        new FunctionalCommand(magazine::enable, () -> {
                        }, interrupted -> magazine.disable(), () -> magazine.magSense.get(),
                            magazine)),
                    autoDrive_pt2,
                    new ParallelCommandGroup(new InstantCommand(() -> intake.intakeDeploy()),
                        new SequentialCommandGroup(autoDrive_pt3, autoDrive_pt4),
                        new InstantCommand(() -> shooter.setSetpoint(4700 / 60)),
                        new FunctionalCommand(magazine::enable, () -> {
                        }, interrupted -> magazine.disable(), () -> magazine.magSense.get(),
                            magazine)),
                    new ZeroMotorsWaitCommand(swerve, 0)),
                new WaitUntilCommand(() -> shooter.atSetpoint()),
                new InstantCommand(magazine::enable), new WaitCommand(5),
                new InstantCommand(() -> intake.intakeRetract()),
                new InstantCommand(() -> shooter.disable()),
                new InstantCommand(() -> magazine.disable())));
    }
}
