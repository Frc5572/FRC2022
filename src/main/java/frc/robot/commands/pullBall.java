package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class pullBall extends CommandBase {
    private Magazine magazine;

    public pullBall(Magazine magazine) {
        this.magazine = magazine;
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
}
