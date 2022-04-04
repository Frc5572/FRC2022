package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.OuterMagazine;
import frc.robot.subsystems.Shooter;

/**
 * Shoot Balls
 */
public class FeedShooter extends SequentialCommandGroup {
    OuterMagazine outerMagazine;
    InnerMagazine innerMagazine;

    public FeedShooter(InnerMagazine innerMagazine, OuterMagazine outerMagazine, Shooter shooter) {
        addRequirements(shooter, innerMagazine, outerMagazine);
        this.innerMagazine = innerMagazine;
        this.outerMagazine = outerMagazine;

        SequentialCommandGroup part1 =
            new SequentialCommandGroup(new PrintCommand("Shooter is being weird"),
                new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
                new WaitCommand(.5), new MagazineRPM(shooter, innerMagazine).withTimeout(.5),
                new InnerMagIntake(innerMagazine)
                    .deadlineWith(new StartEndCommand(() -> outerMagazine.magazineUp(.6),
                        () -> outerMagazine.magazineStop(), outerMagazine)));
        SequentialCommandGroup part2 = new SequentialCommandGroup(
            new WaitUntilCommand(() -> shooter.getSetpoint() > 0 && shooter.atSetpoint()),
            new WaitCommand(.5), new MagazineRPM(shooter, innerMagazine));

        addCommands(part1, part2);
    }

    @Override
    public void end(boolean interrupted) {
        this.innerMagazine.disable();
        this.outerMagazine.magazineStop();
    }
}
