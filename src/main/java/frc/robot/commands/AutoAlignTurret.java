package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Turret;

/**
 * Autonomous that aligns limelight then executes a trajectory.
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
        addCommands(new FunctionalCommand(() -> turret.turretLeft(), () -> {
        }, interrupt -> turret.turretStop(), () -> vision.getTargetFound(), turret).withTimeout(.6),
            new AlignTurret(turret, vision));
    }
}
