package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

/**
 * Creates commands for the magazine to run.
 */
public class MagControl extends CommandBase {
    private Magazine magazine;

    /**
     * Initalizes the command for magazine and adds requirements to systems.
     */
    public MagControl(Magazine magazine) {
        this.magazine = magazine;
        addRequirements(magazine);
    }

    @Override
    public boolean isFinished() {
        return magazine.magSense.get();
    }
}
