package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InsideClimber;

/**
 * Inside PC stands for "Inside Pneumatic Control". This command controls the pneumatics for the
 * inside arms.
 */
public class InsidePC extends CommandBase {
    private InsideClimber climber;
    private boolean status = false;

    public InsidePC(InsideClimber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if (status) {
            this.climber.deployClimbers();
        } else {
            this.climber.retractClimbers();
        }
        status = !status;
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
