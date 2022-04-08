package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.OuterMagazine;
import frc.robot.subsystems.Shooter;

/**
 * Feed Shooter with mag motors
 */
public class FeedShooter extends SequentialCommandGroup {
    OuterMagazine outerMagazine;
    InnerMagazine innerMagazine;

    /**
     * Feed Shooter with mag motors
     *
     * @param innerMagazine Inner Magazine subsystem
     * @param outerMagazine Outer Magazine Subsystem
     * @param shooter Shooter Subsystem
     */
    public FeedShooter(InnerMagazine innerMagazine, OuterMagazine outerMagazine, Shooter shooter) {
        addRequirements(innerMagazine, outerMagazine);
        this.innerMagazine = innerMagazine;
        this.outerMagazine = outerMagazine;

        SequentialCommandGroup part1 =
            new SequentialCommandGroup(new PrintCommand("Shooter is being weird"),
                new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
                new WaitCommand(.2),
                (new WaitUntilCommand(() -> !innerMagazine.magSense.get()).withTimeout(.5)
                    .andThen(new WaitCommand(.2)))
                        .deadlineWith(new MagazineRPM(shooter, innerMagazine)),
                new InnerMagIntake(innerMagazine)
                    .alongWith(new InstantCommand(() -> outerMagazine.magazineUp(.6))));
        SequentialCommandGroup part2 = new SequentialCommandGroup(
            new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
            new WaitCommand(.2), new MagazineRPM(shooter, innerMagazine));

        addCommands(part1, part2);
    }

    @Override
    public void end(boolean interrupted) {
        this.innerMagazine.disable();
        this.outerMagazine.magazineStop();
    }
}
