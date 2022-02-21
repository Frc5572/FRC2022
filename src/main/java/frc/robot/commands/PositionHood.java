package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

/**
 *
 * Hood command
 * </p>
 */

public class PositionHood extends CommandBase {
    Hood hood;
    double position;

    /**
     *
     * Hood command
     * </p>
     */

    public PositionHood(Hood hood, double position) {
        this.hood = hood;
        this.position = position;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setHoodPosition(position);
    }

}
