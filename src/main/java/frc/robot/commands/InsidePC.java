package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Inside PC stands for "Inside Pneumatic Control". This command controls the pneumatics for the
 * inside arms.
 */
public class InsidePC extends CommandBase {
    private Climber insideArms;

    public InsidePC(Climber insideArms) {
        this.insideArms = insideArms;
        addRequirements(insideArms);
    }

    @Override
    public void execute() {
        this.insideArms.deployInsideClimbers();
    }

    @Override
    public void end(boolean interrupt) {
        this.insideArms.returnInsideClimbers();
    }
}
