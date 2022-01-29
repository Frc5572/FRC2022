package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class initMag extends CommandBase {
    private Magazine magazine;
    private Magazine magSense;


    public initMag(Magazine magazine, Magazine magSense) {
        this.magazine = magazine;
        this.magSense = magSense;
        addRequirements(magazine);
        addRequirements(magSense);
    }

    @Override
    public void execute() {
        magazine.startMagazine();
    }

    @Override
    public void end(boolean interrupted) {
        if (magazine.magSense.get()) {
            magazine.stopMagazine();
        }
    }

    @Override
    public boolean isFinished() {
        return magazine.magSense.get();
    }


}
