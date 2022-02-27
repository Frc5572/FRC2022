package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Turret;

/**
 * Command for aligning turret in TeleOp
 */

public class AlignTurret extends CommandBase {
    Turret turret;
    Vision vision;

    /**
     *
     * @param turret turret subsystem
     * @param vision vision subsystem
     */
    public AlignTurret(Turret turret, Vision vision) {
        this.turret = turret;
        this.vision = vision;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (this.turret.alignEnabled) {
            if (vision.getTargetFound()) {
                turret.turretSet(vision.getAimValue());
            } else {
                turret.turretSet(0);
            }
        } else {
            turret.turretSet(0);
        }
    }

}
