package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

/**
 * Creates commands for the magazine to run.
 */
public class CheckMag extends CommandBase {
    private Magazine magazine;
    private Magazine magSense;


    /**
     * Initalizes the command for magazine and adds requirements to systems.
     */
    public CheckMag(Magazine magazine, Magazine magSense) {
        this.magazine = magazine;
        this.magSense = magSense;
        addRequirements(magazine);
        addRequirements(magSense);
    }

    @Override
    public void execute() {
        if (magazine.magSense.get()) {
            magazine.stopMagazine();
        }
    }

    @Override
    public boolean isFinished() {
        return magazine.magSense.get();
    }


}
