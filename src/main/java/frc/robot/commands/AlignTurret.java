package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Turret;

public class AlignTurret extends CommandBase {
    Turret turret;
    Vision vision;
    boolean status;

    public AlignTurret(Turret turret, Vision vision, boolean status) {
        this.turret = turret;
        this.vision = vision;
        this.status = status;
    }

    @Override
    public void execute() {
        if (vision.getTargetFound() && status) {
            turret.turretSet(vision.getAimValue());
        } else {
            turret.turretSet(0);
        }
    }

}
