package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OutsideClimber;

/**
 * Outside PC stands for "Outside Pneumatic Control". This command controls the pneumatics for the
 * outside arms.
 */
public class OutsidePC extends CommandBase {
    private OutsideClimber climber;
    private boolean status = false;

    public OutsidePC(OutsideClimber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (status) {
            this.climber.deployOutsideClimbers();
            status = false;
        } else {
            this.climber.retractOutsideClimbers();
            status = true;
        }
        System.out.println(status);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
