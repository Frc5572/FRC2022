package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Hood;

/**
 * <p>
 * Hood command
 * </p>
 */

public class PositionHood extends CommandBase {
    Hood hood;
    Vision vision;

    /**
     *
     * @param hood hood subsystem
     * @param vision vision subsystem
     */
    public PositionHood(Hood hood, Vision vision) {
        this.hood = hood;
        this.vision = vision;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setHoodPosition();
    }

}
