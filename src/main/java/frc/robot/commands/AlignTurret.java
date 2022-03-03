package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

/**
 * Command for aligning turret in TeleOp
 */

public class AlignTurret extends CommandBase {
    Turret turret;
    PhotonCamera camera;
    double yaw = 0;
    double calculated = 0;
    double lastSpeed = 0;

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
        PhotonPipelineResult result = camera.getLatestResult();
        if (this.turret.alignEnabled && result.hasTargets()) {
            yaw = result.getBestTarget().getYaw();
            calculated = (yaw / 125) * 3;
            calculated = (Math.abs(calculated) <= Constants.VisionConstants.deadPocket) ? 0
                : (calculated >= .2) ? .2
                    : (Math.abs(calculated) - Math.abs(lastSpeed)) > .5 ? .5 : calculated;
            lastSpeed = calculated;
            System.out.println(calculated);
            turret.turretSet(calculated);
        } else {
            turret.turretSet(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
