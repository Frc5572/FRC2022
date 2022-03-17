package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Turret;

/**
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class AutoAlignTurret extends SequentialCommandGroup {


    /**
     * Align the turret from Auto starting position
     *
     * @param turret Turret subsystem
     * @param vision Vision subsystem
     */
    public AutoAlignTurret(Turret turret, Vision vision) {
        addRequirements(turret);
        addCommands(new ParallelDeadlineGroup(new WaitCommand(.6),
            new InstantCommand(() -> turret.turretLeft())), new AlignTurret(turret, vision));
    }
}
