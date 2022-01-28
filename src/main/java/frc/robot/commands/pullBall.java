package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class pullBall extends CommandBase {
    private Magazine magazine;
    private DigitalInput magSense;


    public pullBall(Magazine magazine, DigitalInput magSense) {
        this.magazine = magazine;
        this.magSense = magSense;
        addRequirements(magazine);
    }

    @Override
    public void execute() {
        magazine.startIntake();
    }

    @Override
    public void end(boolean interrupted) {
        if (magSense.get()) {
            magazine.stopIntake();
        }
    }

    @Override
    public boolean isFinished() {
        return magSense.get();
    }


}
