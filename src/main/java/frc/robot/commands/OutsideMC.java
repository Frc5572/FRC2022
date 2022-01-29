package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Outside MC stands for "Outside Motor Control". This command controls the motors for the outside
 * arms.
 */
public class OutsideMC extends CommandBase {
    private Climber climber;
    private boolean dir;

    public OutsideMC(Climber climber, boolean dir) {
        this.climber = climber;
        this.dir = dir;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (dir) {
            this.climber.engageOutsideMotors();
        } else {
            this.climber.disengageOutsideMotors();
        }
    }

    @Override
    public void end(boolean interrupt) {
        this.climber.stopOutsideMotors();
    }
}
