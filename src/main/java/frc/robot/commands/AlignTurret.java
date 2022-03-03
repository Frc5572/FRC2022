package frc.robot.commands;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 * Command for aligning turret in TeleOp
 */

public class AlignTurret extends CommandBase {
    Turret turret;
    PhotonCamera camera;

    /**
     *
     * @param turret turret subsystem
     * @param vision vision subsystem
     */
    public AlignTurret(Turret turret, PhotonCamera camera) {
        this.turret = turret;
        this.camera = camera;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();
        if (this.turret.alignEnabled && result.hasTargets()) {
            turret.turretSet(result.getBestTarget().getYaw());
        } else {
            turret.turretSet(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
