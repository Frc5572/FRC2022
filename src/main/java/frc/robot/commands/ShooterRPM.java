package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

/**
 * Controls Shooter RPM based on
 */
public class ShooterRPM extends CommandBase {

    private Shooter shooter;
    private PhotonCamera camera;
    double curDisRPM = 0;
    double newDisRPM = 0;
    double distance = 0;

    /**
     *
     * @param shooter shooter subsystem
     * @param vision vision subsystem
     */
    public ShooterRPM(Shooter shooter, PhotonCamera camera) {
        this.shooter = shooter;
        this.camera = camera;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // System.out.println("STARTING SHOOTER");
        updateSetpoint();
        // System.out.println("Initial RPM: " + this.shooter.getSetpoint());
        this.shooter.enable();
    }

    @Override
    public void execute() {
        updateSetpoint();
        // System.out.println("SHOOTER SETPOINT: " + this.shooter.getSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        curDisRPM = 0;
        shooter.setSetpoint(curDisRPM);
        this.shooter.disable();
    }

    private void updateSetpoint() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            distance = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.VisionConstants.cameraHeightMeters,
                Constants.VisionConstants.targetHeightMeters,
                Constants.VisionConstants.cameraPitchRadians,
                Units.degreesToRadians(result.getBestTarget().getPitch())) / 12;
        }
        newDisRPM =
            3.452380952381 * Math.pow(distance, 3) - 61.7857142857143 * Math.pow(distance, 2)
                + 402.6190476190476 * Math.pow(distance, 1) + 2800;
        // newDisRPM = 3.45238 * Math.pow(distance, 3) - 51.42857 * Math.pow(distance, 2)
        // + 289.40476 * Math.pow(distance, 1) + 3300;
        // newDisRPM = 3.45238 * Math.pow(distance, 3) - 56 * Math.pow(distance, 2)
        // + 345 * Math.pow(distance, 1) + 2975;
        if (Math.abs(curDisRPM - newDisRPM) >= 100) {
            if (newDisRPM >= 6000) {
                curDisRPM = 6000;
            } else if (newDisRPM <= 3500) {
                curDisRPM = 3500;
            } else {
                curDisRPM = newDisRPM;
            }
            this.shooter.setSetpoint(curDisRPM / 60);
        }
    }
}
