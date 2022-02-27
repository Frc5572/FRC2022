package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Limelight;
import frc.robot.subsystems.Hood;

/**
 * <p>
 * Hood command
 * </p>
 */

public class PositionHood extends CommandBase {
    Hood hood;
    Limelight limelight;

    /**
     *
     * @param hood hood subsystem
     * @param limelight limelight subsystem
     */
    public PositionHood(Hood hood, Limelight limelight) {
        this.hood = hood;
        this.limelight = limelight;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setHoodPosition();
    }

}
