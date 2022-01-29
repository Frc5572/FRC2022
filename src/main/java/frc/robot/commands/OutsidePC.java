package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Outside PC stands for "Outside Pneumatic Control". This command controls the pneumatics for the
 * outside arms.
 */
public class OutsidePC extends CommandBase {
    private Climber climber;
    private boolean status = false;

    public OutsidePC(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (!status) {
            this.climber.deployOutsideClimbers();
            status = true;
        } else {
            this.climber.retractOutsideClimbers();
            status = false;
        }
        System.out.println(status);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
