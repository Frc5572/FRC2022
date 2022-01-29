package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Inside MC stands for "Inside Motor Control". This command controls the motors for the inside
 * arms.
 */
public class InsideMC extends CommandBase {
    private Climber insideMotors;

    public InsideMC(Climber insideMotors) {
        this.insideMotors = insideMotors;
        addRequirements(insideMotors);
    }

    @Override
    public void execute() {
        this.insideMotors.engageInsideMotors();
    }

    @Override
    public void end(boolean interrupt) {
        this.insideMotors.disengageInsideMotors();
    }
}
