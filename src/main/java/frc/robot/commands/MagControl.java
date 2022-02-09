package frc.robot.commands;

import frc.robot.subsystems.Magazine;

/**
 * Creates commands for the magazine to run.
 */
public class MagControl extends ForceMag {
    private Magazine magazine;
    private ForceMag forceMag;

    /**
     * Initalizes the command for magazine and adds requirements to systems.
     */
    public MagControl(Magazine magazine, ForceMag forceMag) {
        super(magazine);
        this.forceMag = forceMag;
        this.magazine = magazine;
        addRequirements(magazine);
    }

    @Override
    public void execute() {
        magazine.startMagazine();
    }

    @Override
    public boolean isFinished() {
        return magazine.magSense.get();
    }
}
