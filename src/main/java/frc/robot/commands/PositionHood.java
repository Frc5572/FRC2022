package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class PositionHood extends CommandBase {
    Hood hood;
    double position;

    public PositionHood(Hood hood, double position) {
        this.hood = hood;
        this.position = position;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setHoodPosition(position);
    }

    // @Override
    // public void end(boolean interrupted) {
    // hood.hoodServo.set(0);
    // }

    // @Override
    // public boolean isFinished() {
    // return hood.getHoodCANCoderSet(position);
    // }

}
