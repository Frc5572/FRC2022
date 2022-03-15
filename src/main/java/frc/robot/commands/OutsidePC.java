package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
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
        if (status && !DriverStation.isDisabled() && !DriverStation.isEStopped()) {
            this.climber.deployClimbers();
        } else {
            this.climber.retractClimbers();
        }
        status = !status;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        // TODO Auto-generated method stub
        return true;
    }
}
