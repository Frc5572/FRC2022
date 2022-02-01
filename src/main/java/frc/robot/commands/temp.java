package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

/**
 * Loads ball into shooter
 */
public class PullBall extends CommandBase {

    /**
     * Decalres both objects and magazine
     */
    private Magazine magazine;
    private DigitalInput sense;


    /**
     * Decalres both objects and magazine
     */
    public PullBall(Magazine magazine, DigitalInput sense) {

        this.magazine = magazine;
        this.sense = sense;
        addRequirements(magazine);
    }

    @Override
    public void execute() {
        magazine.up();
    }



    @Override
    public boolean isFinished() {
        return sense.get();

    }


}
