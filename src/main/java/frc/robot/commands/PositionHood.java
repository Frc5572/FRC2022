package frc.robot.commands;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

/**
 * <p>
 * Hood command
 * </p>
 */

public class PositionHood extends CommandBase {
    Hood hood;
    PhotonCamera camera;

    /**
     *
     * @param hood hood subsystem
     * @param vision vision subsystem
     */
    public PositionHood(Hood hood, PhotonCamera camera) {
        this.hood = hood;
        this.camera = camera;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setHoodPosition();
    }

}
