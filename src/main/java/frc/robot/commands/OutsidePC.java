package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Outside PC stands for "Outside Pneumatic Control". This command controls the pneumatics for the
 * outside arms.
 */
public class OutsidePC extends CommandBase {
    private Climber outsideArms;

    public OutsidePC(Climber outsideArms) {
        this.outsideArms = outsideArms;
        addRequirements(outsideArms);
    }

    @Override
    public void execute() {
        this.outsideArms.deployOutsideClimbers();
    }

    @Override
    public void end(boolean interrupt) {
        this.outsideArms.returnOutsideClimbers();
    }
}
