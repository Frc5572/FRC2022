package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD
import frc.robot.modules.Vision;
import frc.robot.subsystems.Hood;

/**
 *
 * Hood command
=======
import frc.robot.subsystems.Hood;

/**
 * <p>
 * Hood command
 * </p>
>>>>>>> origin/main
 */

public class PositionHood extends CommandBase {
    Hood hood;
<<<<<<< HEAD
    Vision vision;

    public PositionHood(Hood hood, Vision vision) {
        this.hood = hood;
        this.vision = vision;
=======
    double position;

    /**
     * <p>
     * Hood command
     * </p>
     */

    public PositionHood(Hood hood, double position) {
        this.hood = hood;
        this.position = position;
>>>>>>> origin/main
        addRequirements(hood);
    }

    @Override
    public void execute() {
<<<<<<< HEAD
        hood.setHoodPosition(vision.getHoodValue());
=======
        hood.setHoodPosition(position);
>>>>>>> origin/main
    }

}
