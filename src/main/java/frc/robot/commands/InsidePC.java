package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Inside PC stands for "Inside Pneumatic Control". This command controls the pneumatics for the
 * inside arms.
 */
public class InsidePC extends CommandBase {
    private Climber climber;
    private boolean status = false;

    public InsidePC(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (!status) {
            this.climber.deployInsideClimbers();
            status = true;
        } else {
            this.climber.retractInsideClimbers();
            status = false;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
