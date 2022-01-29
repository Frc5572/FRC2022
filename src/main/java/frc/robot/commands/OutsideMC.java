package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Outside MC stands for "Outside Motor Control". This command controls the motors for the outside
 * arms.
 */
public class OutsideMC extends CommandBase {
    private Climber outsideMotors;

    public OutsideMC(Climber outsideMotors) {
        this.outsideMotors = outsideMotors;
        addRequirements(outsideMotors);
    }

    @Override
    public void execute() {
        this.outsideMotors.engageOutsideMotors();
    }

    @Override
    public void end(boolean interrupt) {
        this.outsideMotors.disengageOutsideMotors();
    }
}
