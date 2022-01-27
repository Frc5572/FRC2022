package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Returns a Command to run the climber.
 */
public class EngageInside extends CommandBase {
    private Climber insideClimb;

    public EngageInside(Climber insideClimb) {
        this.insideClimb = insideClimb;
        addRequirements(insideClimb);
    }

    @Override
    public void execute() {
        this.insideClimb.engageInsideClimbers();
    }


    @Override
    public void end(boolean interrupted) {
        this.insideClimb.disengageInsideClimbers();
    }

}
