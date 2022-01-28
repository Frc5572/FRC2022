package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class pullBall extends CommandBase {
    private Magazine magazine;
    private DigitalInput sense;


    public pullBall(Magazine magazine, DigitalInput sense) {
        this.magazine = magazine;
        this.sense = sense;
        addRequirements(magazine);
    }

    @Override
    public void execute() {
        magazine.up();
    }

    @Override
    public void end(boolean interrupted) {
        magazine.stop();
    }

    @Override
    public boolean isFinished() {
        return sense.get();

    }


}
