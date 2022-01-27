package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Returns a Command to run the climber.
 */
public class EngageOutside extends CommandBase {
    private Climber outisdeClimb;

    public EngageOutside(Climber outisdeClimb) {
        this.outisdeClimb = outisdeClimb;
        addRequirements(outisdeClimb);
    }

    @Override
    public void execute() {
        this.outisdeClimb.engageOutsideClimbers();
    }


    @Override
    public void end(boolean interrupted) {
        this.outisdeClimb.disengageOutsideClimbers();
    }

}
