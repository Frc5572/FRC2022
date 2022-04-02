package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;

public class setHoodPos2 extends CommandBase {
    private Hood hood;

    public setHoodPos2(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        // hood.setHoodPosition();
        this.hood.setOneNeg();
    }

    @Override
    public void end(boolean interrupt) {
        this.hood.setZero();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(HoodConstants.minPosition - this.hood.getCANCoderPos()) <= 5;
    }
}
