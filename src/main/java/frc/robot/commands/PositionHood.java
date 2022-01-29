package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Hood;

public class PositionHood extends CommandBase {
    Hood hood;

    public PositionHood(Vision vision, Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setHoodPosition();
    }

    @Override
    public void end(boolean interrupted) {
        hood.setHoodPosition(0);
    }

}
