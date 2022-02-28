package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.InsideClimber;
import frc.robot.subsystems.OutsideClimber;

/**
 * Inside PC stands for "Inside Pneumatic Control". This command controls the pneumatics for the
 * inside arms.
 */
public class AutoClimb extends SequentialCommandGroup {

    public AutoClimb(InsideClimber insideClimber, OutsideClimber outsideClimber) {
        addRequirements(insideClimber, outsideClimber);

        // Stage 0
        SequentialCommandGroup stage0 =
            new SequentialCommandGroup(new InstantCommand(() -> insideClimber.enableClimbers()),
                new InstantCommand(() -> outsideClimber.enableClimbers()),
                new InstantCommand(() -> insideClimber.retractClimbers()),
                new InstantCommand(() -> outsideClimber.deployClimbers()), new ParallelCommandGroup(
                    new FunctionalCommand(() -> insideClimber.engageMotors(), () -> {
                    }, interupt -> insideClimber.stopMotors(),
                        () -> true /* replace with limit switch */, insideClimber),
                    new FunctionalCommand(() -> outsideClimber.engageMotors(), () -> {
                    }, interupt -> outsideClimber.stopMotors(),
                        () -> true /* replace with limit switch */, outsideClimber)),
                new FunctionalCommand(() -> insideClimber.retractMotors(), () -> {
                }, interupt -> insideClimber.stopMotors(),
                    () -> true /* replace with limit switch */, insideClimber));

        SequentialCommandGroup stage1 = new SequentialCommandGroup(
            new InstantCommand(() -> outsideClimber.retractClimbers()), new ParallelCommandGroup(
                new FunctionalCommand(() -> outsideClimber.retractMotors(), () -> {
                }, interupt -> outsideClimber.stopMotors(),
                    () -> true /* replace with limit switch */, outsideClimber),
                new SequentialCommandGroup(new WaitCommand(1),
                    new InstantCommand(() -> insideClimber.deployClimbers()), new WaitCommand(1))),
            new InstantCommand(() -> insideClimber.retractClimbers()), new ParallelCommandGroup(
                new FunctionalCommand(() -> insideClimber.retractMotors(), () -> {
                }, interupt -> insideClimber.stopMotors(),
                    () -> true /* replace with limit switch */, insideClimber),
                new SequentialCommandGroup(new WaitCommand(1),
                    new InstantCommand(() -> outsideClimber.deployClimbers()),
                    new WaitCommand(1))));

        addCommands(stage0, stage1, stage1);
    }
}
