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
        if (this.turret.alignEnabled && vision.getTargetFound()) {
            this.turret.turretSet(vision.getAimValue());
        } else {
            this.turret.turretSet(0);
        }
        this.vision.setLEDMode(this.turret.alignEnabled);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
