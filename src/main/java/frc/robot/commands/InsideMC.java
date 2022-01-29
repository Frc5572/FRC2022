package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Inside MC stands for "Inside Motor Control". This command controls the motors for the inside
 * arms.
 */
public class InsideMC extends CommandBase {
    private Climber climber;
    private boolean dir;

    /**
     * Create and add requirements for InsideMC command
     *
     * @param climber passes through climber subsystem
     * @param dir passes through boolean dir
     */
    public InsideMC(Climber climber, boolean dir) {
        this.climber = climber;
        this.dir = dir;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (dir) {
            this.climber.engageInsideMotors();
        } else {
            this.climber.disengageInsideMotors();
        }
    }

    @Override
    public void end(boolean interrupt) {
        this.climber.stopInsideMotors();
    }
}
