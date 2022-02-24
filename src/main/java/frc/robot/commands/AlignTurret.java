package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Turret;

public class AlignTurret extends CommandBase {
    Turret turret;
    Vision vision;

    public AlignTurret(Turret turret, Vision vision) {
        this.turret = turret;
        this.vision = vision;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (this.turret.status) {
            if (vision.getTargetFound()) {
                turret.turretSet(vision.getAimValue());
            }
        } else {
            turret.turretSet(0);
        }
    }

}
