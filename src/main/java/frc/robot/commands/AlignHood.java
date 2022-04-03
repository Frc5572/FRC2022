package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class AlignHood extends CommandBase {
    private Hood hood;

    public AlignHood(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        double calculated =
            (this.hood.calculateHoodPosition(50) - this.hood.getCANCoderPos()) / 400;
        if (calculated > .15) {
            calculated = .15;
        } else if (calculated < -.15) {
            calculated = -.15;
        }
        this.hood.hoodSet(calculated);
        System.out.println("HOOD POWER: " + calculated);

        // System.out.println(this.hood.calculateHoodPosition(45));
    }

    @Override
    public void end(boolean interrupt) {
        this.hood.setZero();
    }
}
