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

public class P14B extends AutoBase {
    public P14B(Swerve swerve, Shooter shooter, Magazine magazine, Intake intake) {
        super(swerve);

        PathPlannerTrajectory p1b4 = PathPlanner.loadPath("P1_4B", 1, 1);
        PathPlannerTrajectory p1b4pt1 = PathPlanner.loadPath("P1_4B pt1", 1, 1);
        PathPlannerTrajectory p1b4pt2 = PathPlanner.loadPath("P1_4b pt2", 1, 1);
        PPSwerveControllerCommand testCommand = baseSwerveCommand(p1b4);
        PPSwerveControllerCommand grabSecond = baseSwerveCommand(p1b4pt1);
        PPSwerveControllerCommand grabThird = baseSwerveCommand(p1b4pt2);

        addCommands(new ParallelCommandGroup(
            new InstantCommand(() -> swerve.resetOdometry(p1b4.getInitialPose())),
            new InstantCommand(() -> shooter.enable()).andThen(
                new WaitUntilCommand(() -> shooter.atSetpoint()),
                new InstantCommand(() -> magazine.enable()), new WaitCommand(2))));
    }
}
