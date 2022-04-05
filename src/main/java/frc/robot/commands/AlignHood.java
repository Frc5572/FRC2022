package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

/**
 * Aligns hood based on a CANCoder position
 */
public class AlignHood extends CommandBase {
    private Hood hood;
    private double maxCalc = 0.15;

    public AlignHood(Hood hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        double calculated =
            (Constants.HoodConstants.maxPosition - this.hood.getCANCoderPos()) / 400;
        if (calculated > maxCalc) {
            calculated = maxCalc;
        } else if (calculated < -maxCalc) {
            calculated = -maxCalc;
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
