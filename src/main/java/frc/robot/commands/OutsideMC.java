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

    /**
     * Create and add requirements for OutsideMC command
     *
     * @param climber Passes the climber subsystem through
     * @param dir Passes boolean dir through
     */
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
            this.climber.retractOutsideMotors();
        }
    }

    @Override
    public void end(boolean interrupt) {
        this.climber.stopOutsideMotors();
    }
}
