package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InsideClimber;

/**
 * Inside MC stands for "Inside Motor Control". This command controls the motors for the inside
 * arms.
 */
public class InsideMC extends CommandBase {
    private InsideClimber climber;
    private boolean dir;

    /**
     * Create and add requirements for InsideMC command
     *
     * @param climber passes through climber subsystem
     * @param dir passes through boolean dir
     */
    public InsideMC(InsideClimber climber, boolean dir) {
        this.climber = climber;
        this.dir = dir;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (dir) {
            this.climber.engageInsideMotors();
        } else {
            this.climber.retractInsideMotors();
        }
    }

    @Override
    public void end(boolean interrupt) {
        this.climber.stopInsideMotors();
    }
}
